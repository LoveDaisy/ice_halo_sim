// MSL helper + test-kernel source strings — see header for design notes.
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
// Field-by-field bit-exact alignment of host layout. sizeof must be 108 bytes
// (verified by static_assert host-side; MSL has no static_assert across
// strings, so the parity harness uploads a single host-built desc and the
// kernel reads it back — any layout drift surfaces as a mismatch).
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
  uchar  _pad0;
  uint   min_len;
  uint   max_len;
  float  dir[3];
  float  radii_c;
  uint   crystal_id;
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

// Top-level dispatch — Complex returns pass-through `true` (plan §2).
static inline bool DeviceFilterMatch(device const DeviceFilterDesc& f,
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
  // Complex (5) — out of scope this task; pass-through true (plan §2).
  return true;
}

// Check = Match XOR (action == filter_out). Same algebra as
// filter_spec.hpp:42-45.
static inline bool DeviceFilterCheck(device const DeviceFilterDesc& f,
                                     thread const uchar* path, uint path_len,
                                     device const uchar* getfn_bytes,
                                     device const uint*  getfn_offsets,
                                     uint  crystal_slot,
                                     thread const float* ray_dir,
                                     uint  ray_crystal_config_id) {
  bool m = DeviceFilterMatch(f, path, path_len, getfn_bytes, getfn_offsets,
                             crystal_slot, ray_dir, ray_crystal_config_id);
  return (f.action == 0u) ? m : !m;
}
)METAL";

const char* const kFilterMatchTestKernelSrc = R"METAL(
struct FilterMatchTestParams {
  uint n;
  uint face_seq_cap;
  uint check_mode;  // 0 = Match (ignore action), 1 = Check (apply action)
};

// One thread per ray. Reads (path, path_len, crystal_slot, crystal_config_id,
// ray_dir, filter_idx) and writes a single uchar 0/1 to `out_match`.
//
// Buffer layout (mirrors plan D6 binding contract — both this kernel and the
// host harness must agree on indices):
//   0  filter_desc_buf   : DeviceFilterDesc[n_filters]
//   1  getfn_offsets_buf : uint[n_crystals + 1]
//   2  getfn_bytes_buf   : uchar[]
//   3  ray_path_buf      : uchar[n * face_seq_cap]
//   4  ray_path_len_buf  : uchar[n]
//   5  ray_crystal_slot  : uint16[n]
//   6  ray_crystal_cid   : uint16[n]
//   7  ray_dir_buf       : float[n * 3]
//   8  ray_filter_idx    : uint[n]
//   9  out_match_buf     : uchar[n]
//   10 params            : FilterMatchTestParams
kernel void filter_match_test_kernel(
    device const DeviceFilterDesc* filter_desc   [[buffer(0)]],
    device const uint*             getfn_offsets [[buffer(1)]],
    device const uchar*            getfn_bytes   [[buffer(2)]],
    device const uchar*            ray_path      [[buffer(3)]],
    device const uchar*            ray_path_len  [[buffer(4)]],
    device const ushort*           ray_cslot     [[buffer(5)]],
    device const ushort*           ray_cid       [[buffer(6)]],
    device const float*            ray_dir       [[buffer(7)]],
    device const uint*             ray_filter    [[buffer(8)]],
    device uchar*                  out_match     [[buffer(9)]],
    constant FilterMatchTestParams& prm          [[buffer(10)]],
    uint tid [[thread_position_in_grid]]) {
  if (tid >= prm.n) { return; }
  uint len = (uint)ray_path_len[tid];
  if (len > kDevRecCap) { len = kDevRecCap; }
  uchar path_local[64];  // kDevRecCap
  uint base = tid * prm.face_seq_cap;
  for (uint i = 0; i < len; i++) {
    path_local[i] = ray_path[base + i];
  }
  float dir_local[3];
  dir_local[0] = ray_dir[tid * 3 + 0];
  dir_local[1] = ray_dir[tid * 3 + 1];
  dir_local[2] = ray_dir[tid * 3 + 2];
  uint fi = ray_filter[tid];
  device const DeviceFilterDesc& f = filter_desc[fi];
  bool result;
  if (prm.check_mode == 0u) {
    result = DeviceFilterMatch(f, path_local, len, getfn_bytes, getfn_offsets,
                               (uint)ray_cslot[tid], dir_local, (uint)ray_cid[tid]);
  } else {
    result = DeviceFilterCheck(f, path_local, len, getfn_bytes, getfn_offsets,
                               (uint)ray_cslot[tid], dir_local, (uint)ray_cid[tid]);
  }
  out_match[tid] = result ? 1u : 0u;
}
)METAL";
// clang-format on

}  // namespace lumice

#endif  // defined(__APPLE__)
