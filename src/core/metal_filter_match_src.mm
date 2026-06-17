// MSL helper source string — see header for design notes.
//
// The MSL `struct DeviceFilterDesc` layout MUST mirror
// `core/device_filter_desc.hpp` exactly (the host fills the buffer with the
// C++ layout; the kernel reinterprets the same bytes). Field order and types
// are checked by the parity harness — any drift produces mismatch on a non-
// empty Raypath/EntryExit canonical comparison.

#include "core/metal_filter_match_src.hpp"

#if defined(__APPLE__)

namespace lumice {

// clang-format off
const char* const kFilterMatchHelperSrc = R"METAL(
#include <metal_stdlib>
using namespace metal;

// --- DeviceFilterDesc mirror (must match src/core/device_filter_desc.hpp) ---
// Field-by-field bit-exact alignment of host layout. sizeof must be 120 bytes
// post-267.1b (Complex filter trailer added: or_clause_count reuses former
// _pad0 byte; and_term_counts[8] + sub_desc_start appended). Verified by
// host-side static_assert; MSL has no static_assert across strings, so the
// parity harness uploads a single host-built desc and the kernel reads it
// back — any layout drift surfaces as a mismatch.
struct DeviceFilterDesc {
  uchar  type;
  uchar  action;
  uchar  symmetry;
  uchar  d_applicable;
  int    sigma_a;
  int    fn_period;
  uchar  canonical_bytes[64];
  uchar  canonical_len;
  uchar  has_entry;
  uchar  has_exit;
  uchar  or_clause_count;        // Complex only; 0 for non-Complex (was _pad0)
  uint   min_len;
  uint   max_len;
  float  dir[3];
  float  radii_c;
  uint   crystal_id;
  uchar  and_term_counts[8];     // Complex only: # AND-terms per OR-clause
  uint   sub_desc_start;         // Complex only: flat start index in complex_sub_desc_buf
};

// Device filter type tags. Mirror kDeviceFilterType* in device_filter_desc.hpp.
constant uchar kDevFilterTypeNone      = 0;
constant uchar kDevFilterTypeRaypath   = 1;
constant uchar kDevFilterTypeEntryExit = 2;
constant uchar kDevFilterTypeDirection = 3;
constant uchar kDevFilterTypeCrystal   = 4;
constant uchar kDevFilterTypeComplex   = 5;

constant uchar kDevSymNone = 0;
constant uchar kDevSymP    = 1;
constant uchar kDevSymB    = 2;
constant uchar kDevSymD    = 4;
constant int   kDevFnPeriodHex = 6;
constant uint  kDevRecCap = 64;  // kMaxHits

// --- ReduceBuffer (E4 spike, byte-identical to lumice::detail::ReduceBuffer) -

static inline void PCanonicalShiftInPlace_dev(thread uchar* data, uint size) {
  int first_pri = -1;
  for (uint i = 0; i < size; i++) {
    uchar x = data[i];
    if (x < 3) { continue; }
    uchar pyr = x / 10;
    int pri = (int)(x % 10);
    if (first_pri < 0) { first_pri = pri; }
    pri = (pri + kDevFnPeriodHex - first_pri) % kDevFnPeriodHex + 3;
    data[i] = (uchar)(pyr * 10 + pri);
  }
}

static inline bool LexLess_dev(thread const uchar* a, thread const uchar* b, uint size) {
  for (uint i = 0; i < size; i++) {
    if (a[i] != b[i]) { return a[i] < b[i]; }
  }
  return false;
}

static inline void ReduceBuffer_dev(thread uchar* data, uint size, uchar symmetry,
                                    int sigma_a, bool d_applicable) {
  if (symmetry == kDevSymNone) { return; }
  if (symmetry & kDevSymP) {
    PCanonicalShiftInPlace_dev(data, size);
  }
  if ((symmetry & kDevSymD) && d_applicable) {
    uchar scratch[kDevRecCap];
    for (uint i = 0; i < size; i++) {
      uchar x = data[i];
      if (x < 3) { scratch[i] = x; continue; }
      uchar pyr = x / 10;
      int pri0 = (int)(x % 10) - 3;
      int new_pri0 = ((sigma_a - pri0) % kDevFnPeriodHex + kDevFnPeriodHex) % kDevFnPeriodHex;
      scratch[i] = (uchar)(pyr * 10 + new_pri0 + 3);
    }
    if (symmetry & kDevSymP) {
      PCanonicalShiftInPlace_dev(scratch, size);
    }
    if (LexLess_dev(scratch, data, size)) {
      for (uint i = 0; i < size; i++) { data[i] = scratch[i]; }
    }
  }
  if (symmetry & kDevSymB) {
    uchar scratch[kDevRecCap];
    bool changed = false;
    for (uint i = 0; i < size; i++) {
      uchar x = data[i];
      if (x <= 2) { scratch[i] = (uchar)(3 - x); changed = true; }
      else if (x >= 13 && x <= 18) { scratch[i] = (uchar)(x + 10); changed = true; }
      else if (x >= 23 && x <= 28) { scratch[i] = (uchar)(x - 10); changed = true; }
      else { scratch[i] = x; }
    }
    if (changed && LexLess_dev(scratch, data, size)) {
      for (uint i = 0; i < size; i++) { data[i] = scratch[i]; }
    }
  }
}

// --- ApplyGetFn ---
// Mirrors the GetFn remap in CopyContSliceToRootBuf (metal_trace_backend.mm
// :1670-1675). `crystal_slot` indexes into the prefix-sum offset table to
// locate this crystal's per-poly-index GetFn byte stripe. Writes
// face-numbers into `out` (caller-owned thread buffer).
static inline void ApplyGetFn_dev(thread const uchar* path, uint len,
                                  device const uchar* getfn_bytes,
                                  device const uint*  getfn_offsets,
                                  uint  crystal_slot,
                                  thread uchar* out) {
  uint base = getfn_offsets[crystal_slot];
  // Stripe end (next prefix sum entry) bounds the per-crystal table; we
  // trust path[k] < stripe_len because the kernel produces valid poly-indices
  // and the parity harness controls the test inputs.
  for (uint i = 0; i < len; i++) {
    out[i] = getfn_bytes[base + path[i]];
  }
}

// --- DeviceFilterMatch* per type ---
//
// Match semantics (no action_ applied here): returns true if the ray's
// path/dir/crystal-id satisfies the filter PREDICATE. Callers apply
// `action` (filter_in vs filter_out) at the gate site (mirrors host
// FilterSpec::Check on filter_spec.hpp:42-45).

static inline bool DeviceFilterMatchRaypath(device const DeviceFilterDesc& f,
                                            thread const uchar* path, uint path_len,
                                            device const uchar* getfn_bytes,
                                            device const uint*  getfn_offsets,
                                            uint  crystal_slot) {
  if (path_len != f.canonical_len) { return false; }
  uchar buf[kDevRecCap];
  ApplyGetFn_dev(path, path_len, getfn_bytes, getfn_offsets, crystal_slot, buf);
  if (f.fn_period < 0 || f.symmetry == kDevSymNone) {
    for (uint i = 0; i < path_len; i++) {
      if (buf[i] != f.canonical_bytes[i]) { return false; }
    }
    return true;
  }
  ReduceBuffer_dev(buf, path_len, f.symmetry, f.sigma_a, f.d_applicable != 0u);
  for (uint i = 0; i < path_len; i++) {
    if (buf[i] != f.canonical_bytes[i]) { return false; }
  }
  return true;
}

static inline bool DeviceFilterMatchEntryExit(device const DeviceFilterDesc& f,
                                              thread const uchar* path, uint path_len,
                                              device const uchar* getfn_bytes,
                                              device const uint*  getfn_offsets,
                                              uint  crystal_slot) {
  if (path_len == 0u) { return false; }
  if (path_len < f.min_len) { return false; }
  if (f.max_len != 0u && path_len > f.max_len) { return false; }
  if (!f.has_entry && !f.has_exit) { return true; }
  // Resolve the ends in face-number space (single ApplyGetFn for the two
  // bytes — buf small enough that we always remap the whole prefix/suffix).
  uchar ee[2];
  uint base = getfn_offsets[crystal_slot];
  uint ee_len = 0u;
  if (f.has_entry) { ee[ee_len++] = getfn_bytes[base + path[0]]; }
  if (f.has_exit)  { ee[ee_len++] = getfn_bytes[base + path[path_len - 1u]]; }
  if (f.fn_period < 0 || f.symmetry == kDevSymNone) {
    for (uint i = 0; i < ee_len; i++) {
      if (ee[i] != f.canonical_bytes[i]) { return false; }
    }
    return true;
  }
  uchar buf[kDevRecCap];
  for (uint i = 0; i < ee_len; i++) { buf[i] = ee[i]; }
  ReduceBuffer_dev(buf, ee_len, f.symmetry, f.sigma_a, f.d_applicable != 0u);
  if (ee_len != f.canonical_len) { return false; }
  for (uint i = 0; i < ee_len; i++) {
    if (buf[i] != f.canonical_bytes[i]) { return false; }
  }
  return true;
}

static inline bool DeviceFilterMatchDirection(device const DeviceFilterDesc& f,
                                              thread const float* ray_dir) {
  float d = f.dir[0] * ray_dir[0] + f.dir[1] * ray_dir[1] + f.dir[2] * ray_dir[2];
  return d > f.radii_c;
}

static inline bool DeviceFilterMatchCrystal(device const DeviceFilterDesc& f,
                                            uint ray_crystal_config_id) {
  return ray_crystal_config_id == f.crystal_id;
}

// Simple-only dispatch: None / Raypath / EntryExit / Direction / Crystal. Does
// NOT include the Complex branch — DeviceFilterMatchComplex calls this to
// match a sub-spec, so including Complex here would form a static call cycle
// (DeviceFilterMatch → Complex → Simple → ... ) which MSL rejects. The host
// builder guarantees Complex sub-specs are always Simple (mirrors
// ComplexSpec which uses SimpleSpecCreator on each sub-spec).
static inline bool DeviceFilterMatchSimple(device const DeviceFilterDesc& f,
                                           thread const uchar* path, uint path_len,
                                           device const uchar* getfn_bytes,
                                           device const uint*  getfn_offsets,
                                           uint  crystal_slot,
                                           thread const float* ray_dir,
                                           uint  ray_crystal_config_id) {
  if (f.type == kDevFilterTypeNone)     { return true; }
  if (f.type == kDevFilterTypeRaypath)  {
    return DeviceFilterMatchRaypath(f, path, path_len, getfn_bytes, getfn_offsets, crystal_slot);
  }
  if (f.type == kDevFilterTypeEntryExit) {
    return DeviceFilterMatchEntryExit(f, path, path_len, getfn_bytes, getfn_offsets, crystal_slot);
  }
  if (f.type == kDevFilterTypeDirection) {
    return DeviceFilterMatchDirection(f, ray_dir);
  }
  if (f.type == kDevFilterTypeCrystal)  {
    return DeviceFilterMatchCrystal(f, ray_crystal_config_id);
  }
  // Unknown / Complex (caller error in this dispatch).
  return false;
}

// Complex filter: OR over AND-clauses of Simple sub-specs. Empty Complex
// (or_clause_count == 0) returns `false`, matching host ComplexSpec::Match
// (empty filters_ → loop body never executes → falls through to return false,
// filter_spec.cpp:274-288). Do NOT change to pass-through `true` — that would
// silently disable any deferred-construction Complex filter.
static inline bool DeviceFilterMatchComplex(device const DeviceFilterDesc& f,
                                            device const DeviceFilterDesc* complex_sub_desc_buf,
                                            thread const uchar* path, uint path_len,
                                            device const uchar* getfn_bytes,
                                            device const uint*  getfn_offsets,
                                            uint  crystal_slot,
                                            thread const float* ray_dir,
                                            uint  ray_crystal_config_id) {
  if (f.or_clause_count == 0) { return false; }  // see comment above
  uint sub_idx = f.sub_desc_start;
  for (uint or_i = 0; or_i < (uint)f.or_clause_count; or_i++) {
    uint and_n = (uint)f.and_term_counts[or_i];
    bool and_ok = true;
    for (uint and_j = 0; and_j < and_n; and_j++) {
      if (!DeviceFilterMatchSimple(complex_sub_desc_buf[sub_idx],
                                   path, path_len, getfn_bytes, getfn_offsets,
                                   crystal_slot, ray_dir, ray_crystal_config_id)) {
        and_ok = false;
        // Short-circuit: skip the rest of this AND-clause so sub_idx lands at
        // the next OR-clause start. and_j sub-descs already consumed, jump the
        // remaining (and_n - and_j).
        sub_idx += (and_n - and_j);
        break;
      }
      sub_idx++;
    }
    if (and_ok) { return true; }
  }
  return false;
}

// Top-level dispatch — Simple types delegate to DeviceFilterMatchSimple;
// Complex delegates to DeviceFilterMatchComplex (which calls Simple, not back
// here, so the MSL static call graph stays acyclic).
static inline bool DeviceFilterMatch(device const DeviceFilterDesc& f,
                                     device const DeviceFilterDesc* complex_sub_desc_buf,
                                     thread const uchar* path, uint path_len,
                                     device const uchar* getfn_bytes,
                                     device const uint*  getfn_offsets,
                                     uint  crystal_slot,
                                     thread const float* ray_dir,
                                     uint  ray_crystal_config_id) {
  if (f.type == kDevFilterTypeComplex) {
    return DeviceFilterMatchComplex(f, complex_sub_desc_buf,
                                    path, path_len, getfn_bytes, getfn_offsets,
                                    crystal_slot, ray_dir, ray_crystal_config_id);
  }
  return DeviceFilterMatchSimple(f, path, path_len, getfn_bytes, getfn_offsets,
                                 crystal_slot, ray_dir, ray_crystal_config_id);
}

// Check = Match XOR (action == filter_out). Same algebra as
// filter_spec.hpp:42-45.
static inline bool DeviceFilterCheck(device const DeviceFilterDesc& f,
                                     device const DeviceFilterDesc* complex_sub_desc_buf,
                                     thread const uchar* path, uint path_len,
                                     device const uchar* getfn_bytes,
                                     device const uint*  getfn_offsets,
                                     uint  crystal_slot,
                                     thread const float* ray_dir,
                                     uint  ray_crystal_config_id) {
  bool m = DeviceFilterMatch(f, complex_sub_desc_buf,
                             path, path_len, getfn_bytes, getfn_offsets,
                             crystal_slot, ray_dir, ray_crystal_config_id);
  return (f.action == 0u) ? m : !m;
}
)METAL";
// clang-format on

}  // namespace lumice

#endif  // defined(__APPLE__)
