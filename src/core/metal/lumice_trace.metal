#include <metal_stdlib>
using namespace metal;

// Shared single-source math cores (projection forward + Fresnel). Identical
// bodies are compiled into projection.cpp on the host side; the embed-script
// expands these includes inline for the `newLibraryWithSource` fallback path.
#include "../shared/projection_shared.h"
#include "../shared/optics_shared.h"
#include "../shared/traversal_shared.h"
// PCG hash + orientation/triangle samplers are single-sourced in pcg_shared.h
// (scrum-cuda-backend-complete 296.4) so CUDA's transit_multi_ms_kernel and
// MSL's transit_root_kernel cannot drift on the orientation math. The shader
// keeps its historical unqualified call sites via the `using namespace lm_pcg`
// directive below.
#include "../shared/pcg_shared.h"
// task-metal-device-fused-consumer (S1): host/MSL/CUDA XYZ accumulation
// primitive. AccumXyzToPixel is the only call site for atomic adds into
// xyz_image and landed_weight; cross-batch host-side Neumaier sits in
// RenderConsumer (host #ifdef branch). Single-source contract: any future
// CUDA emit gate (S2) uses the same call.
#include "../shared/accum_shared.h"
using namespace lm_pcg;

// --- DeviceFilterDesc mirror (must match src/core/device_filter_desc.hpp) ---
// Field-by-field bit-exact alignment of host layout. task-device-flat-and-terms:
// `and_term_counts[8]` inline array removed (moved to a separate flat buffer
// `and_term_counts_buf` indexed via `and_terms_start`); `or_clause_count`
// widened to ushort so 4096-clause physical Complex filters fit. Verified by
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
  uchar  _pad_reserved;          // former or_clause_count uint8 slot; padding-only now
  uint   min_len;
  uint   max_len;
  float  dir[3];
  float  radii_c;
  uint   crystal_id;
  uint   sub_desc_start;         // Complex only: flat start index in complex_sub_desc_buf
  uint   and_terms_start;        // Complex only: flat start index in and_term_counts_buf
  ushort or_clause_count;        // Complex only; 0 for non-Complex
  ushort _pad_or_tail;           // trailing padding
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

// task-358.1 Step 4: MUST match host kMaxColorClassesDevice
// (metal_trace_backend.mm). Previously this was a bare `16` literal in
// KernelParams below, referenced by a comment claiming a named
// `kMaxColorClassesDeviceMsl` sibling that didn't actually exist — code-
// review-01 Suggestion #2 flagged the dangling reference. This is now that
// named sibling; raising the cap = bump both this and kMaxColorClassesDevice
// together (no compiler can check MSL vs. C++ across the two languages, so
// the sync is still comment/discipline-enforced, just against a real symbol).
constant uint  kMaxColorClassesDeviceMsl = 16;

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
// Remaps poly-index path to face-number space via the per-crystal GetFn byte
// table (built on host by `BuildDeviceGetFnBytes`, see
// core/device_filter_desc.{hpp,cpp}). `crystal_slot` indexes into the prefix-
// sum offset table to locate this crystal's stripe. Writes face-numbers into
// `out` (caller-owned thread buffer).
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
                                            device const uchar* and_term_counts_buf,
                                            thread const uchar* path, uint path_len,
                                            device const uchar* getfn_bytes,
                                            device const uint*  getfn_offsets,
                                            uint  crystal_slot,
                                            thread const float* ray_dir,
                                            uint  ray_crystal_config_id) {
  if (f.or_clause_count == 0) { return false; }  // see comment above
  uint sub_idx = f.sub_desc_start;
  for (uint or_i = 0; or_i < (uint)f.or_clause_count; or_i++) {
    uint and_n = (uint)and_term_counts_buf[f.and_terms_start + or_i];
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
                                     device const uchar* and_term_counts_buf,
                                     thread const uchar* path, uint path_len,
                                     device const uchar* getfn_bytes,
                                     device const uint*  getfn_offsets,
                                     uint  crystal_slot,
                                     thread const float* ray_dir,
                                     uint  ray_crystal_config_id) {
  if (f.type == kDevFilterTypeComplex) {
    return DeviceFilterMatchComplex(f, complex_sub_desc_buf, and_term_counts_buf,
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
                                     device const uchar* and_term_counts_buf,
                                     thread const uchar* path, uint path_len,
                                     device const uchar* getfn_bytes,
                                     device const uint*  getfn_offsets,
                                     uint  crystal_slot,
                                     thread const float* ray_dir,
                                     uint  ray_crystal_config_id) {
  bool m = DeviceFilterMatch(f, complex_sub_desc_buf, and_term_counts_buf,
                             path, path_len, getfn_bytes, getfn_offsets,
                             crystal_slot, ray_dir, ray_crystal_config_id);
  return (f.action == 0u) ? m : !m;
}

// task-331.5 (raypath-color foundation): stride of the per-gate-slot component
// bit table (gate_component_bits[gate_slot * kDevComponentStride + k]). MUST
// equal host `kDeviceFilterMaxOrClauses` (device_filter_desc.hpp): a Complex
// filter has at most that many OR-summands; simple filters use only index 0.
constant uint kDevComponentStride = 8u;

// Per-summand match mask (mirror of host ComplexSpec::MatchSummandMask /
// FilterSpec::MatchSummandMask, filter_spec.cpp:307 / filter_spec.hpp:58).
// Returns a mask whose bit k is set iff OR-summand k matched — EVERY clause is
// evaluated (no cross-clause short-circuit) so the collapse-to-boolean gate
// does not discard the per-summand info the component mask needs. `out_matched`
// receives the pre-action collapse boolean (== DeviceFilterMatch). Semantics:
//   - Complex : bit or_i set iff its AND-clause fully matched.
//   - None    : device emits mask 0, matched=true. NOTE (task-339.1): host
//               NoneSpec now yields a whole-crystal bit (mask 0b1); the device
//               intentionally still emits 0 here — None whole-crystal bits are
//               host-only until scrum-3c. No longer "mirrors NoneSpec".
//   - other simple : 1 summand → bit 0 iff matched (mirrors non-None simple).
static inline uint DeviceFilterSummandMask(device const DeviceFilterDesc& f,
                                           device const DeviceFilterDesc* complex_sub_desc_buf,
                                           device const uchar* and_term_counts_buf,
                                           thread const uchar* path, uint path_len,
                                           device const uchar* getfn_bytes,
                                           device const uint*  getfn_offsets,
                                           uint  crystal_slot,
                                           thread const float* ray_dir,
                                           uint  ray_crystal_config_id,
                                           thread bool* out_matched) {
  if (f.type == kDevFilterTypeComplex) {
    uint mask = 0u;
    uint sub_idx = f.sub_desc_start;
    // task-device-flat-and-terms: color-path helper, so `or_i <
    // kDevComponentStride` stays as an explicit cap (color bit map + uint mask
    // are still stride-limited to kDeviceFilterMaxOrClauses summands).
    for (uint or_i = 0u; or_i < (uint)f.or_clause_count && or_i < kDevComponentStride; or_i++) {
      uint and_n = (uint)and_term_counts_buf[f.and_terms_start + or_i];
      bool and_ok = true;
      for (uint and_j = 0u; and_j < and_n; and_j++) {
        if (!DeviceFilterMatchSimple(complex_sub_desc_buf[sub_idx],
                                     path, path_len, getfn_bytes, getfn_offsets,
                                     crystal_slot, ray_dir, ray_crystal_config_id)) {
          and_ok = false;
          sub_idx += (and_n - and_j);  // skip rest of this AND-clause
          break;
        }
        sub_idx++;
      }
      if (and_ok) { mask |= (1u << or_i); }
    }
    *out_matched = (mask != 0u);
    return mask;
  }
  if (f.type == kDevFilterTypeNone) {
    *out_matched = true;   // None passes but contributes 0 summands
    return 0u;
  }
  bool m = DeviceFilterMatchSimple(f, path, path_len, getfn_bytes, getfn_offsets,
                                   crystal_slot, ray_dir, ray_crystal_config_id);
  *out_matched = m;
  return m ? 1u : 0u;
}

// --- Former kKernelSrc literal boundary --------------------------------------
// Everything below was once a separate MSL string (kKernelSrc) concatenated
// after the filter-match helper above. They now compile as one translation unit
// (single .metal → metallib, single newLibraryWithSource fallback), so the
// redundant `#include <metal_stdlib>` / `using namespace metal;` that headed the
// old literal were removed — the directives at the top of this file cover it.
// -----------------------------------------------------------------------------

constant float  kFloatEps  = 1e-5f;
constant ushort kInvalidId = 0xffffu;
constant uint   kRecCap    = 64u;

// scrum-267 task-fused-emit-gate Step 5 (296.4 hoist): PCG hash + PcgStream +
// pcg_uniform now live in ../shared/pcg_shared.h (single-sourced with CUDA).
// `using namespace lm_pcg;` near the top of this file imports them unqualified
// so the trace_layer_kernel emit gate / gen_root_kernel / transit_root_kernel
// call sites keep their original spellings.

// scrum-268.8 (DR-3): per-ray wavelength pool entry. The host samples M
// wavelengths uniformly over [380, 780] nm once per (crystal, illuminant) ci
// dispatch and uploads this table; kernels look up per-ray optics by wl_idx.
// Layout MUST mirror the C++ WlEntry struct below kKernelSrc (sizeof == 20).
struct WlEntry {
  float n_idx;
  float spd_weight;
  float cmf_x;
  float cmf_y;
  float cmf_z;
};

// Merged exit-stats accumulator for trace_layer_kernel. One 8-byte struct
// bound at buffer(15) (count + w_sum atomics), freeing buffer(16) for the
// per-ray pool-shape offset carrier (`r_pool_shape`) that the K-shape geometry
// pool feeds in. Field order MUST match the host mirror `struct ExitStats` in
// metal_trace_backend.mm (see the static_assert there). Both fields sit at
// natural 4-byte alignment; total sizeof == 8.
struct ExitStats {
  atomic_uint  count;
  atomic_float w_sum;
};

struct KernelParams {
  // scrum-268.8 (DR-3): per-batch n_idx / cie_x/y/z removed. trace kernel now
  // reads per-ray optics from wl_pool[wl_idx] (see WlEntry above + buffer
  // bindings in DispatchLayer).
  uint  max_hits;
  uint  poly_cnt;
  uint  num_rays;
  uint  img_w;
  uint  img_h;
  uint  ms_mode;
  uint  out_cap;
  // Buffer-egress (exit seam, scrum-258.1): kernel's final-exit branch writes
  // {world_dir(3), weight} to exit_d/exit_w at a session-level atomic slot
  // (exit_slot). Capacity bound is host-supplied (ComputeOutCap).
  uint  exit_cap;
  // Exit metadata (scrum-258.2): rich {dir, weight, path, crystal_id,
  // ms_layer_idx} record. crystal_id = ci for this dispatch; ms_layer_idx =
  // host-side ms_idx (only the final layer writes exit slots in ms_mode==0).
  // face_seq_cap = min(max_hits, ExitFaceSeq::kCap=15) — per-slot stride of
  // exit_face_seq_data_buf; <15 saves device memory + readback bandwidth.
  uint  crystal_id;
  uint  face_seq_cap;
  uint  ms_layer_idx;
  // Emit-gate (scrum-267 task-fused-emit-gate Step 2). Read only by the
  // ms_mode==1 emit gate; ms_mode==0 dispatches leave these zero / host fills
  // a benign default.
  //   ms_prob            : MS continuation probability for THIS layer (gate
  //                        keeps the ray when pcg_uniform() < ms_prob)
  //   gate_seed          : PCG seed for the gate's prob draw — derived from
  //                        gen_seed_ XOR (ms_layer_idx, crystal_id) nonce so
  //                        successive dispatches see independent prob streams
  //   filter_desc_max_ci : stride for gate_slot = ms_layer_idx * stride + ci
  //                        (mirrors EnsureFilterBuffers' max_ci layout)
  //   crystal_config_id  : DeviceFilterMatchCrystal compares against this; for
  //                        non-CrystalSpec filters the kernel never reads it
  float ms_prob;
  uint  gate_seed;
  uint  filter_desc_max_ci;
  uint  crystal_config_id;
  // Unified render projection (315.3): host predigests all trig-heavy setup
  // (per-type scale, dual-fisheye r_scale/overlap, rectangular az0, camera rot)
  // into this POD, filled by BeginSession via BuildProjParams. The exit tail
  // calls lm_proj::ProjectExitToPixel(proj, world_exit...) — single source with
  // host CPU (scatter_accum.hpp) and CUDA. Replaces the former loose az0 /
  // proj_type / r_scale / max_abs_dz fields.
  lm_proj::ProjParams proj;
  // task-358.3 (renamed from capture_component after Fork-C retirement): when
  // non-zero, the emit gate appends (this_mask, weight) of every emitted ray
  // to the capture ring. `this_mask` is now purely Design-2 colour bits (the
  // Fork-C physical-bit produce branch has been deleted). 0 in production →
  // the append branch is skipped; carried-mask cross-layer flow stays on.
  // Test-only.
  uint capture_ray_mask;
  // task-358.1 (metal-color-parity, Design-2 ColorGateTable migration):
  //   has_color_groups        : 0 → skip color pass entirely (AC4 zero-cost).
  //                             1 → run the color pass (production path when
  //                             raypath_color is configured).
  //   color_desc_offset       : gate_filter_desc index where the color region
  //                             starts (= physical n_slot). Access:
  //                               color_desc[color_slot] =
  //                                 gate_filter_desc[color_desc_offset +
  //                                                  gate_slot * color_max_groups_per_slot + g]
  //   color_bits_offset       : gate_component_bits index where the color
  //                             bit-map region starts (= n_slot * K, where K =
  //                             kDeviceFilterMaxOrClauses).
  //   color_max_groups_per_slot: per-slot group budget (host kColorMaxGroupsPer
  //                             Slot; MUST match `kColorMaxGroupsPerSlot`
  //                             sibling below).
  uint has_color_groups;
  uint color_desc_offset;
  uint color_bits_offset;
  uint color_max_groups_per_slot;
  // task-358.1 Step 4 (AC3 device-side Y-lane accumulation):
  //   color_class_count   : number of active color classes (0 → skip lane
  //                         accumulation, class_lane_buf is a 4-byte dummy).
  //   color_class_bits[]  : per-class OR-union of member component bits (matches
  //                         host ColorClass.member_bits_; ulong for MSL uint64).
  //   color_class_combine[]: per-class combinator; 0 = kAny ((mask & bits) != 0),
  //                         1 = kAll ((mask & bits) == bits).
  // See host kMaxColorClassesDevice; MUST match kMaxColorClassesDeviceMsl above.
  uint  color_class_count;
  ulong color_class_bits[kMaxColorClassesDeviceMsl];
  uchar color_class_combine[kMaxColorClassesDeviceMsl];
  // task-device-flat-and-terms: byte offset (in bytes) into the
  // complex_sub_desc_buf allocation where the AND-term counts region begins.
  // The kernel reinterprets `gate_sub_desc_buf + and_term_counts_base_offset`
  // as a `device const uchar*` (both regions share buffer 27 to stay within
  // Metal's 30-buffer per-stage cap). Zero when no Complex filter is present.
  uint  and_term_counts_base_offset;
};

kernel void trace_layer_kernel(
    device const float*    root_d   [[buffer(0)]],
    device const float*    root_p   [[buffer(1)]],
    device const float*    root_w   [[buffer(2)]],
    device const ushort*   root_tf  [[buffer(3)]],
    device const float*    poly_n   [[buffer(4)]],
    device const float*    poly_d   [[buffer(5)]],
    device const float*    centroid [[buffer(6)]],
    constant KernelParams& prm      [[buffer(7)]],
    device atomic_float*   image    [[buffer(8)]],
    device float*          out_d    [[buffer(9)]],
    // scrum-268.8 (DR-3): slots 10 and 12 reclaimed from the retired out_p /
    // out_tf writes (dead data — transit_root_kernel resamples entry point +
    // face on the next-layer crystal; ReadbackExitRays never read cont_p /
    // cont_tf). slot 10 now binds the wavelength pool, 12 binds the per-ray
    // root wavelength index that the trace kernel reads at entry.
    // scrum-268.8 (DR-3): wl_pool routes through the `constant` address space
    // so per-thread divergent WlEntry reads hit the shader-side cache rather
    // than the slower device-memory gather path. Pool size ≤ 255 × 20B = 5100B
    // fits well below Metal's 64KB constant buffer ceiling.
    constant WlEntry*      wl_pool       [[buffer(10)]],
    device float*          out_w         [[buffer(11)]],
    device const uint*     root_wl_idx   [[buffer(12)]],
    device atomic_uint*    counter  [[buffer(13)]],
    device float*          rec_sink [[buffer(14)]],
    // Merged {count, w_sum} atomics (see `struct ExitStats` above); buffer(16)
    // reserved for the K-shape pool's per-ray pool-shape offset carrier.
    device ExitStats*      exit_stats [[buffer(15)]],
    device const float*    root_rot [[buffer(17)]],
    // S1 device-fused: slot 18 is now the per-session landed-weight scalar
    // (total weight of in-bounds filter-pass emitted rays, used for
    // normalisation). Slots 18-23, 28, 30 previously held exit-record
    // buffers; those are replaced by on-device projection + XYZ accumulation
    // into `image` (AccumXyzToPixel from accum_shared.h).
    device atomic_float*   landed_weight [[buffer(18)]],
    // scrum-267 task-fused-emit-gate Step 4b: device-side emit gate state.
    // The ms_mode==1 path now calls DeviceFilterCheck and draws a PCG prob to
    // decide continuation vs mid-exit on-device; the former cont_crystal_id /
    // cont_face_seq_* buffers (24-26) used by the host hop are removed and the
    // gate buffers move into the freed slots. NOTE: plan §3 reserved 27-31 for
    // these five buffers; Metal's per-stage buffer index ceiling is 30 (`buffer
    // attribute parameter out of bounds: must be between 0 and 30`), so the
    // actual binding is 24-28 (see progress.md DECISION "buffer 槽位 27-31 调
    // 整为 24-28"). All inline references below use the actual 24-28 slots.
    //   24 : filter desc array, indexed by ms_layer_idx * filter_desc_max_ci + ci
    //   25 : per-slot prefix-sum offsets into gate_getfn_bytes
    //   26 : flat GetFn(poly_idx) byte stream
    //   27 : Complex filter sub-spec flat buffer
    //   28 : (freed — was exit_ms_layer; removed in S1 device-fused)
    device const DeviceFilterDesc* gate_filter_desc    [[buffer(24)]],
    device const uint*             gate_getfn_offsets  [[buffer(25)]],
    device const uchar*            gate_getfn_bytes    [[buffer(26)]],
    device const DeviceFilterDesc* gate_sub_desc_buf   [[buffer(27)]],
    // scrum-268.8 (DR-3): cont_wl_idx propagates the photon's lifetime
    // wavelength tag into the continuation ring for the next layer.
    device uint*                   cont_wl_idx         [[buffer(29)]],
    // task-331.5 (raypath-color foundation): per-ray uint64 component mask.
    //   root_component  (19, read)  : mask carried INTO this layer (0 on layer
    //                                 0; transit copies cont→root for layer≥1).
    //   cont_component  (20, write) : mask carried OUT to the continuation ring
    //                                 (lock-stepped with cont_d/cont_w/cont_wl_idx).
    //   gate_component_bits (21,read): summand→component-bit table, keyed by
    //                                 gate_slot * kDevComponentStride + summand.
    //   exit_comp_mask/w (22/23,write) + exit_comp_cnt (28,atomic): capture ring
    //                                 for emitted (mid-exit + final) rays; drained
    //                                 host-side for CPU parity (test-only).
    device const ulong*            root_component      [[buffer(19)]],
    device ulong*                  cont_component      [[buffer(20)]],
    device const uchar*            gate_component_bits [[buffer(21)]],
    device ulong*                  exit_comp_mask      [[buffer(22)]],
    device float*                  exit_comp_w         [[buffer(23)]],
    device atomic_uint*            exit_comp_cnt       [[buffer(28)]],
    // task-358.1 Step 4 (AC3 device-side Y-lane accumulation): per-color-class
    // atomic-float accumulator. Layout is column-major-by-class:
    //   class_lane_buf[class_idx * (img_w * img_h) + (py * img_w + px)]
    // The host allocates class_count * W * H atomic_floats (or a 4-byte dummy
    // when class_count==0 so this binding stays non-nil). Each emit adds
    // cmf_y * cw to every class whose predicate matches the ray's this_mask;
    // read back + folded into RenderConsumer::lane_y_ each drain window. See
    // plan §4 Step 4 for the accumulation semantics + rollback contract.
    device atomic_float*           class_lane_buf      [[buffer(30)]],
    uint tid [[thread_position_in_grid]]) {
  if (tid >= prm.num_rays) { return; }
  // task-331.5: load the carried component mask once (constant for the whole
  // in-crystal path, exactly like the wl_idx lifetime tag below).
  ulong carried_component = root_component[tid];

  // task-device-flat-and-terms: the AND-term counts region shares buffer 27
  // with the Complex sub-desc region (Metal's per-stage 30-buffer cap left no
  // room for a separate binding). Host `EnsureFilterBuffers` packs
  // `[DeviceFilterDesc...][uint8 and_term_counts...]` into one allocation and
  // supplies the byte offset via `KernelParams::and_term_counts_base_offset`.
  device const uchar* gate_and_term_counts =
      reinterpret_cast<device const uchar*>(gate_sub_desc_buf) + prm.and_term_counts_base_offset;

  float dx = root_d[tid * 3u + 0u];
  float dy = root_d[tid * 3u + 1u];
  float dz = root_d[tid * 3u + 2u];
  float ox = root_p[tid * 3u + 0u];
  float oy = root_p[tid * 3u + 1u];
  float oz = root_p[tid * 3u + 2u];
  float w  = root_w[tid];
  ushort to_face = root_tf[tid];

  // Per-ray crystal->world rotation (row-major; world = m*v, mirroring
  // Rotation::Apply on the CPU). Applied to the exit direction before
  // projection so the seam returns world-space rays (invariant 6). The
  // orientation is constant for the whole ray path, so load it once.
  float m[9];
  for (uint k = 0u; k < 9u; k++) { m[k] = root_rot[tid * 9u + k]; }

  // scrum-268.8 (DR-3): per-ray optics via pool lookup. wl_idx is the
  // photon's lifetime tag — set at gen_root, carried through transit, never
  // resampled inside the trace. cie_* accumulates into image via wl_pool
  // entries instead of the deleted prm.cie_x/y/z fields.
  const uint    wl_idx = root_wl_idx[tid];
  const WlEntry wle    = wl_pool[wl_idx];
  const float   n_idx  = wle.n_idx;
  const float   cmf_x  = wle.cmf_x;
  const float   cmf_y  = wle.cmf_y;
  const float   cmf_z  = wle.cmf_z;
  const uint  poly_cnt = prm.poly_cnt;

  const int   iw_i      = int(prm.img_w);
  const int   ih_i      = int(prm.img_h);
  // 315.3: thread-local copy of the POD projection params. ProjectExitToPixel
  // takes a `thread const ProjParams&` (LM_THREAD), but prm lives in the
  // `constant` address space — MSL cannot bind a constant object to a thread
  // reference, so copy once per thread and pass the local to both exit blocks.
  lm_proj::ProjParams proj_local = prm.proj;

  ushort path[kRecCap];
  uint   rec_len = 0u;

  // task-metal-ms-prob-gate-per-exit: single persistent PcgStream shared by
  // both prob-gate call sites (mid-layer emit gate + final-layer gate).
  // `pcg_uniform` bumps `s.slot++` internally, so consecutive draws on the
  // same ray consume disjoint slots — the per-exit independence that CUDA
  // (`cuda_trace_backend.cu:775-778`, "gate_stream advances on each
  // pcg_uniform draw") and legacy CPU (`simulator.cpp:544`, per-emit
  // `rng.GetUniform()`) already provide. `prm.ms_mode` is a dispatch-level
  // constant, so only one of the two consumer branches runs per dispatch —
  // no cross-branch interleaving concern. `gate_seed` is host-derived from
  // `gen_seed_ XOR (ms_layer_idx, crystal_id)` (see `metal_trace_backend.mm`
  // `KernelParams` comment), so dispatches at different layers already draw
  // disjoint values without needing a per-branch nonce shift.
  PcgStream gate_stream;
  gate_stream.seed       = prm.gate_seed;
  gate_stream.global_idx = tid;
  gate_stream.slot       = 0u;

  for (uint hit = 0u; hit < prm.max_hits; hit++) {
    if (to_face == kInvalidId) { break; }
    if (rec_len < kRecCap) { path[rec_len] = to_face; rec_len += 1u; }

    float nx = poly_n[to_face * 3u + 0u];
    float ny = poly_n[to_face * 3u + 1u];
    float nz = poly_n[to_face * 3u + 2u];
    float cos_theta = dx * nx + dy * ny + dz * nz;
    float rr = (cos_theta > 0.0f) ? n_idx : (1.0f / n_idx);
    float dd = (1.0f - rr * rr) / (cos_theta * cos_theta) + rr * rr;
    bool  is_tir = dd <= 0.0f;
    float w_refl = lm_optics::GetReflectRatio(max(dd, 0.0f), rr) * w;
    float w_refr = is_tir ? -1.0f : (w - w_refl);
    float rdx = dx - 2.0f * cos_theta * nx;
    float rdy = dy - 2.0f * cos_theta * ny;
    float rdz = dz - 2.0f * cos_theta * nz;
    float sd  = sqrt(max(dd, 0.0f));
    float fdx, fdy, fdz;
    if (is_tir) {
      fdx = rdx; fdy = rdy; fdz = rdz;
    } else {
      fdx = rr * dx - (rr - sd) * cos_theta * nx;
      fdy = rr * dy - (rr - sd) * cos_theta * ny;
      fdz = rr * dz - (rr - sd) * cos_theta * nz;
    }

    ushort cont_face = kInvalidId;
    float c_dx = 0.0f, c_dy = 0.0f, c_dz = 0.0f;
    float c_ox = 0.0f, c_oy = 0.0f, c_oz = 0.0f;
    float c_w  = 0.0f;

    for (uint ch = 0u; ch < 2u; ch++) {
      float cdx = (ch == 0u) ? rdx : fdx;
      float cdy = (ch == 0u) ? rdy : fdy;
      float cdz = (ch == 0u) ? rdz : fdz;
      float cw  = (ch == 0u) ? w_refl : w_refr;
      if (cw < 0.0f) { continue; }

      // Polygon-slab traversal via the cross-backend single-source
      // intersect (lm_traversal::SlabFaceT, see core/shared/traversal_shared.h).
      // t_far is initialised to 1e30f — equal to SlabFaceT's sentinel for
      // non-candidate faces — so `t < t_far` naturally skips them without
      // an explicit denom-gate at the call site.
      // From-face exclusion: Metal uses the post-loop eps_thr relaxed
      // threshold (see below) rather than an explicit fi==to_face skip; this
      // is equivalent to CUDA's skip on convex crystals (see traversal_shared.h
      // header for the equivalence condition and non-convex caveat).
      float t_far = 1e30f;
      int   far_face = -1;
      for (uint fi = 0u; fi < poly_cnt; fi++) {
        float fnx = poly_n[fi * 3u + 0u];
        float fny = poly_n[fi * 3u + 1u];
        float fnz = poly_n[fi * 3u + 2u];
        float fd  = poly_d[fi];
        float t = lm_traversal::SlabFaceT(cdx, cdy, cdz, ox, oy, oz, fnx, fny, fnz, fd);
        if (t < t_far) {
          t_far = t; far_face = int(fi);
        }
      }
      float eps_thr = (to_face != kInvalidId && far_face != int(to_face)) ? -kFloatEps : kFloatEps;
      if (far_face >= 0 && t_far > eps_thr) {
        cont_face = ushort(far_face);
        c_dx = cdx; c_dy = cdy; c_dz = cdz;
        c_ox = ox + t_far * cdx;
        c_oy = oy + t_far * cdy;
        c_oz = oz + t_far * cdz;
        c_w  = cw;
      } else {
        if (prm.ms_mode == 1u) {
          // scrum-267 task-fused-emit-gate Step 5: device emit gate.
          //   filter_pass + continue → write cont buffers (frame transit on host)
          //   filter_pass + !continue → write exit buffer (mid-exit, pre-gated)
          //   filter_fail              → drop (no write)
          // Direction conversion to world space mirrors legacy CollectData
          // (simulator.cpp:442): crystal_rot_.Apply(r.d_) before FilterSpec
          // ::Check. DeviceFilterMatchDirection compares against world-space
          // ray_dir; non-direction filters do not read it but receiving world
          // ensures future filter additions stay consistent with the host
          // semantic.
          float wcx = m[0] * cdx + m[1] * cdy + m[2] * cdz;
          float wcy = m[3] * cdx + m[4] * cdy + m[5] * cdz;
          float wcz = m[6] * cdx + m[7] * cdy + m[8] * cdz;
          // Build a path buffer in face-index space (ushort path → uchar
          // DeviceFilterMatch expects). Hex face indices fit in uint8 with
          // headroom (poly_cnt ≤ 32 in practice), so the narrowing is safe.
          uchar path_local[kDevRecCap];
          uint  gate_len = min(rec_len, kDevRecCap);
          for (uint k = 0u; k < gate_len; k++) { path_local[k] = uchar(path[k]); }
          uint gate_slot = prm.ms_layer_idx * prm.filter_desc_max_ci + prm.crystal_id;
          float ray_dir_w[3] = { wcx, wcy, wcz };
          // 7th argument INTENTIONALLY passes `gate_slot` (NOT prm.crystal_id
          // as in plan §4 Step 5 pseudo-code). `DeviceFilterCheck` forwards it
          // to `DeviceFilterMatchRaypath` where it indexes
          // `gate_getfn_offsets[slot..slot+1]` to locate the per-orbit GetFn
          // byte stream. `EnsureFilterBuffers` lays out offsets keyed by
          // `slot = mi * max_ci + ci` (= `gate_slot`), so passing
          // `prm.crystal_id` would point at layer-0's orbit table for every
          // layer and silently mis-match from ms_layer_idx ≥ 1. The plan
          // pseudo-code conflated "which crystal" with "where in the slot
          // layout" — gate_slot is the correct slot identity here. (See
          // progress.md DECISION "DeviceFilterCheck 第 7 参数=gate_slot 非
          // prm.crystal_id" for the full derivation; multi-MS filter parity
          // proves this is the working form. Do NOT "fix" back to crystal_id.)
          bool filter_pass = DeviceFilterCheck(
              gate_filter_desc[gate_slot], gate_sub_desc_buf, gate_and_term_counts,
              path_local, gate_len,
              gate_getfn_bytes, gate_getfn_offsets,
              gate_slot, ray_dir_w, prm.crystal_config_id);
          if (filter_pass) {
            // task-358.3: Fork-C physical-bits produce branch removed. `this_mask`
            // now starts from the carried mask and is OR-accumulated by ONLY the
            // Design-2 color pass below (production truth). The append-to-capture
            // branch further down is gated by capture_ray_mask (test-only).
            ulong this_mask = carried_component;
            // task-358.1 (metal-color-parity, Design-2 ColorGateTable):
            // production per-raypath color pass. Gated by prm.has_color_groups
            // so no raypath_color session → single branch skip (AC4 zero-cost).
            // Each color group is a synthesized ComplexFilterParam whose
            // OR-summands correspond to placement.predicates_; the summand
            // mask (bit k = "summand k matched") is looked up against the
            // color-bit map to build the OR-accumulated component mask.
            // Mirrors CPU simulator.cpp CollectData:519-543 semantics.
            if (prm.has_color_groups != 0u) {
              for (uint g = 0u; g < prm.color_max_groups_per_slot; g++) {
                uint color_slot_idx = gate_slot * prm.color_max_groups_per_slot + g;
                // Empty groups are stored as zero-init DeviceFilterDesc
                // (type=None); DeviceFilterSummandMask returns mask=0 for
                // that shape → the inner loop is a no-op. No extra branch
                // needed here.
                bool matched_col = false;
                uint c_smask = DeviceFilterSummandMask(
                    gate_filter_desc[prm.color_desc_offset + color_slot_idx],
                    gate_sub_desc_buf, gate_and_term_counts,
                    path_local, gate_len, gate_getfn_bytes, gate_getfn_offsets,
                    gate_slot, ray_dir_w, prm.crystal_config_id, &matched_col);
                for (uint k = 0u; k < kDevComponentStride; k++) {
                  if ((c_smask & (1u << k)) != 0u) {
                    uchar bit = gate_component_bits[
                        prm.color_bits_offset +
                        color_slot_idx * kDevComponentStride + k];
                    if (bit < 64u) { this_mask |= (1ul << (ulong)bit); }
                  }
                }
              }
            }
            // Per-exit independent prob draw via the persistent gate_stream
            // hoisted above the hit loop — each `pcg_uniform` call advances
            // `slot`, so consecutive exits on the same ray consume disjoint
            // draws (was per-ray-constant before task-metal-ms-prob-gate-per-
            // exit, which turned MS survival into "per-ray all-or-nothing").
            bool do_continue = (pcg_uniform(gate_stream) < prm.ms_prob);
            if (do_continue) {
              // scrum-268.8: out_p / out_tf retired (transit_root_kernel
              // resamples entry point + face on the next-layer crystal so
              // these writes were dead data). cont_wl_idx now propagates
              // the photon's lifetime wavelength tag into the continuation
              // ring; transit_root_kernel reads it back into root_wl_idx_out
              // for the next layer's trace kernel to consume.
              uint slot = atomic_fetch_add_explicit(counter, 1u, memory_order_relaxed);
              if (slot < prm.out_cap) {
                out_d[slot * 3u + 0u] = wcx;
                out_d[slot * 3u + 1u] = wcy;
                out_d[slot * 3u + 2u] = wcz;
                out_w[slot] = cw;
                cont_wl_idx[slot] = wl_idx;
                // task-331.5: carry the OR-accumulated mask with its ray. Kept
                // lock-stepped with cont_d/cont_w/cont_wl_idx — the host handle
                // swap + shuffle gather must move this in lockstep (LANDMINE).
                cont_component[slot] = this_mask;
              }
              // scrum-267 task-fused-emit-gate: post-gate semantics — exit_cnt
              // / exit_wsum now tally "filter_pass polygon-exits" (gate dropped
              // filter_fail rays above; legacy meaning was "all polygon-exits").
              // Diagnostic-only counters; not consumed by parity tests.
              atomic_fetch_add_explicit(&exit_stats->count, 1u, memory_order_relaxed);
              atomic_fetch_add_explicit(&exit_stats->w_sum, cw, memory_order_relaxed);
            } else {
              // Mid-exit: filter_pass && !do_continue → device-fused XYZ accumulation.
              // 315.3: single-source projection via lm_proj::ProjectExitToPixel
              // (shared with host CPU scatter_accum.hpp + CUDA). Pass the WORLD
              // exit dir (wcx,wcy,wcz) — the function negates internally to the
              // sky direction, matching ScatterOutgoingToXyz which feeds the
              // outgoing world dir `d`. Each returned hit is atomically added;
              // landed_weight only bumps on bump_landed (primary, not overlap).
              lm_proj::ProjResult pr_m = lm_proj::ProjectExitToPixel(proj_local, wcx, wcy, wcz);
              for (int hi = 0; hi < pr_m.count; hi++) {
                int px_m = pr_m.hits[hi].px;
                int py_m = pr_m.hits[hi].py;
                if (px_m >= 0 && px_m < iw_i && py_m >= 0 && py_m < ih_i) {
                  uint pix_m = uint(py_m) * prm.img_w + uint(px_m);
                  AccumXyzToPixel(image, pix_m, cmf_x, cmf_y, cmf_z, cw);
                  if (pr_m.hits[hi].bump_landed) {
                    atomic_fetch_add_explicit(landed_weight, cw, memory_order_relaxed);
                  }
                  // task-358.1 Step 4 (AC3 device-side Y-lane accumulation):
                  // fan this ray's Y (cmf_y * cw) into each active color class
                  // whose predicate matches this_mask. Mirrors CPU
                  // RenderConsumer::AccumulateColorClassLanes semantics.
                  // Zero-cost when color_class_count == 0 (single branch skip).
                  if (prm.color_class_count != 0u) {
                    uint pix_stride = prm.img_w * prm.img_h;
                    float y_val = cmf_y * cw;
                    for (uint c = 0u; c < prm.color_class_count; c++) {
                      ulong bits = prm.color_class_bits[c];
                      if (bits == 0ul) { continue; }
                      ulong matched = this_mask & bits;
                      bool satisfied = (prm.color_class_combine[c] == 0u)
                                           ? (matched != 0ul)
                                           : (matched == bits);
                      if (satisfied) {
                        atomic_fetch_add_explicit(&class_lane_buf[c * pix_stride + pix_m],
                                                  y_val, memory_order_relaxed);
                      }
                    }
                  }
                }
              }
              // task-358.3 (renamed from capture_component): append this
              // emitted ray's (this_mask, weight) to the capture ring for the
              // host-side CPU parity harness.
              if (prm.capture_ray_mask != 0u) {
                uint cslot = atomic_fetch_add_explicit(exit_comp_cnt, 1u, memory_order_relaxed);
                if (cslot < prm.out_cap) {
                  exit_comp_mask[cslot] = this_mask;
                  exit_comp_w[cslot]    = cw;
                }
              }
              // Diagnostic counters (not consumed by parity tests).
              atomic_fetch_add_explicit(&exit_stats->count, 1u, memory_order_relaxed);
              atomic_fetch_add_explicit(&exit_stats->w_sum, cw, memory_order_relaxed);
            }
          }
          // filter_fail: implicit drop (no buffer write, no atomic counter
          // bump — energy disappears, matching legacy filter_out semantics).
        } else {
          // Return the exit direction from crystal-local to world space
          // (invariant 6 / DESIGN D2): world = m * cd, row-major, matching
          // CPU CollectData's crystal_rot_.Apply(r.d_). Without this the
          // per-ray-random local frame scatters the halo into a band.
          float wx = m[0] * cdx + m[1] * cdy + m[2] * cdz;
          float wy = m[3] * cdx + m[4] * cdy + m[5] * cdz;
          float wz = m[6] * cdx + m[7] * cdy + m[8] * cdz;
          // S1 device-fused: final-layer filter + prob gate (mirrors ms_mode==1
          // emit gate). scrum-302: the final-layer prob draw runs ON DEVICE.
          // Legacy CollectData (simulator.cpp:468) draws rng for every filter-pass
          // outgoing candidate at EVERY layer; at the final layer rng<prob means
          // "would continue" but there is no next layer → the ray is dropped, so
          // only the (1-prob) remainder is emitted. prm.ms_prob carries the real
          // final-layer prob (backend DispatchLayer passes last_ms_prob_). Omitting
          // this draw made the device emit 100% → metal/legacy energy = 1/(1-prob).
          uchar path_local_f[kDevRecCap];
          uint  gate_len_f = min(rec_len, kDevRecCap);
          for (uint k = 0u; k < gate_len_f; k++) { path_local_f[k] = uchar(path[k]); }
          uint  gate_slot_f = prm.ms_layer_idx * prm.filter_desc_max_ci + prm.crystal_id;
          float ray_dir_w_f[3] = { wx, wy, wz };
          bool filter_pass_f = DeviceFilterCheck(
              gate_filter_desc[gate_slot_f], gate_sub_desc_buf, gate_and_term_counts,
              path_local_f, gate_len_f,
              gate_getfn_bytes, gate_getfn_offsets,
              gate_slot_f, ray_dir_w_f, prm.crystal_config_id);
          // Final-layer prob draw — reuse the persistent gate_stream hoisted
          // above the hit loop (same one the ms_mode==1 branch consumes; only
          // one branch runs per dispatch because ms_mode is a dispatch-level
          // constant). Each pcg_uniform bumps slot so consecutive final-layer
          // exits on the same ray draw disjoint values.
          bool prob_drop_f = (pcg_uniform(gate_stream) < prm.ms_prob);
          if (filter_pass_f && !prob_drop_f) {
            // task-358.3: Fork-C physical-bits produce branch removed (see
            // ms_mode==1 mirror above). `this_mask_f` starts from the carried
            // mask; only the Design-2 color pass below accumulates into it.
            ulong this_mask_f = carried_component;
            // task-358.1: final-layer Design-2 color pass (mirror of the
            // ms_mode==1 color pass above). MUST stay symmetric with the
            // mid-layer branch — this_mask_f vs this_mask, gate_slot_f vs
            // gate_slot, path_local_f vs path_local, ray_dir_w_f vs ray_dir_w.
            // A silent asymmetry here re-introduces the exact class of bug the
            // two landmine-guard tests protect against.
            if (prm.has_color_groups != 0u) {
              for (uint g = 0u; g < prm.color_max_groups_per_slot; g++) {
                uint color_slot_idx_f = gate_slot_f * prm.color_max_groups_per_slot + g;
                bool matched_col_f = false;
                uint c_smask_f = DeviceFilterSummandMask(
                    gate_filter_desc[prm.color_desc_offset + color_slot_idx_f],
                    gate_sub_desc_buf, gate_and_term_counts,
                    path_local_f, gate_len_f, gate_getfn_bytes, gate_getfn_offsets,
                    gate_slot_f, ray_dir_w_f, prm.crystal_config_id, &matched_col_f);
                for (uint k = 0u; k < kDevComponentStride; k++) {
                  if ((c_smask_f & (1u << k)) != 0u) {
                    uchar bit = gate_component_bits[
                        prm.color_bits_offset +
                        color_slot_idx_f * kDevComponentStride + k];
                    if (bit < 64u) { this_mask_f |= (1ul << (ulong)bit); }
                  }
                }
              }
            }
            // task-358.3 (renamed from capture_component): final-layer capture-
            // ring append. `this_mask_f` now carries only Design-2 colour bits
            // (Fork-C physical-bit branch retired); the append itself is still
            // gated on capture_ray_mask so production dispatches skip it.
            if (prm.capture_ray_mask != 0u) {
              uint cslot = atomic_fetch_add_explicit(exit_comp_cnt, 1u, memory_order_relaxed);
              if (cslot < prm.out_cap) {
                exit_comp_mask[cslot] = this_mask_f;
                exit_comp_w[cslot]    = cw;
              }
            }
            // 315.3: single-source projection via lm_proj::ProjectExitToPixel
            // (shared with host CPU scatter_accum.hpp + CUDA). Pass the WORLD
            // exit dir (wx,wy,wz) — the function negates internally to the sky
            // direction, matching ScatterOutgoingToXyz which feeds the outgoing
            // world dir `d`. Each returned hit (primary + optional dual-fisheye
            // overlap) is atomically added; landed_weight only bumps on
            // bump_landed (primary, not overlap — parity with Pass 2).
            lm_proj::ProjResult pr_f = lm_proj::ProjectExitToPixel(proj_local, wx, wy, wz);
            for (int hi = 0; hi < pr_f.count; hi++) {
              int px_f = pr_f.hits[hi].px;
              int py_f = pr_f.hits[hi].py;
              if (px_f >= 0 && px_f < iw_i && py_f >= 0 && py_f < ih_i) {
                uint pix_f_lin = uint(py_f) * prm.img_w + uint(px_f);
                uint pix = pix_f_lin * 3u;
                atomic_fetch_add_explicit(&image[pix + 0u], cmf_x * cw, memory_order_relaxed);
                atomic_fetch_add_explicit(&image[pix + 1u], cmf_y * cw, memory_order_relaxed);
                atomic_fetch_add_explicit(&image[pix + 2u], cmf_z * cw, memory_order_relaxed);
                if (pr_f.hits[hi].bump_landed) {
                  atomic_fetch_add_explicit(landed_weight, cw, memory_order_relaxed);
                }
                // task-358.1 Step 4: final-layer per-class Y-lane accumulation.
                // MUST stay symmetric with the mid-exit path above (this_mask_f
                // vs this_mask, pix_f_lin vs pix_m). Includes overlap-ring hits
                // (bump_landed=false) to match CPU RenderConsumer's Pass 2
                // AccumulateColorClassLanes semantics — overlap contributes to
                // lane Y without contributing to landed_weight.
                if (prm.color_class_count != 0u) {
                  uint pix_stride = prm.img_w * prm.img_h;
                  float y_val = cmf_y * cw;
                  for (uint c = 0u; c < prm.color_class_count; c++) {
                    ulong bits = prm.color_class_bits[c];
                    if (bits == 0ul) { continue; }
                    ulong matched = this_mask_f & bits;
                    bool satisfied = (prm.color_class_combine[c] == 0u)
                                         ? (matched != 0ul)
                                         : (matched == bits);
                    if (satisfied) {
                      atomic_fetch_add_explicit(&class_lane_buf[c * pix_stride + pix_f_lin],
                                                y_val, memory_order_relaxed);
                    }
                  }
                }
              }
            }
            atomic_fetch_add_explicit(&exit_stats->count, 1u, memory_order_relaxed);
            atomic_fetch_add_explicit(&exit_stats->w_sum, cw, memory_order_relaxed);
          }
          // filter_fail: implicit drop — no pixel write, no diagnostic counter bump.
        }
      }
    }

    if (cont_face == kInvalidId) {
      to_face = kInvalidId;
    } else {
      dx = c_dx; dy = c_dy; dz = c_dz;
      ox = c_ox; oy = c_oy; oz = c_oz;
      w = c_w;
      to_face = cont_face;
    }
  }

  float rec_csum = 0.0f;
  for (uint k = 0u; k < rec_len; k++) {
    rec_csum += float(path[k]);
  }
  rec_sink[tid] = rec_csum;
}

// ===================== Device root-gen (task-260.2) ==========================
//
// Counter-based PCG root-ray generator. Replaces host-side InitRayFirstMs +
// memcpy upload when single-crystal + spec.seed != 0 + root_ray_count fits in
// uint32_t. Parity strategy = statistical (ds_corr ≥ 0.99) — host mt19937 and
// device PCG streams are not bitwise alignable.
//
// PCG stream contract:
//   per-thread global root index = gen_ray_base + tid
//   draw counter = mix(gen_seed, global_idx, slot); slot bumps per draw
// gen_ray_base is the running root_ray_count BEFORE this dispatch, so two
// successive batches of the same session never reuse the same (gen_seed,
// global_idx) tuple. Determinism = single-worker (server.cpp:184 enforces
// sim_seed != 0 → worker_count = 1).

// kLatPath* / kDist* discriminators, kMaxTriPerKernel, kMaxRejectionAttempts,
// GenRootKernelParams, pcg_gaussian / pcg_get_dist / normalize_latitude /
// sample_lat_lon_roll, axis_angle_rotation_9 / chain_left_mul_9 /
// build_crystal_rotation_9 / apply_inverse_mat9 — all single-sourced in
// ../shared/pcg_shared.h (scrum-cuda-backend-complete 296.4). The `using
// namespace lm_pcg;` directive near the top of this file imports the
// unqualified names that gen_root_kernel / transit_root_kernel / sample_sph_cap
// call against.

// sample_sph_cap (SampleSphCapPoint, geo3d.cpp:171-205) is single-sourced in
// ../shared/pcg_shared.h (296.6), available unqualified via `using namespace
// lm_pcg`.

// SampleTrianglePoint / RandomSample (categorical) — single-sourced in
// ../shared/pcg_shared.h (scrum-cuda-backend-complete 296.4). Available
// unqualified via the `using namespace lm_pcg;` directive at the top of this
// file.

kernel void gen_root_kernel(
    device float*           root_d        [[buffer(0)]],
    device float*           root_p        [[buffer(1)]],
    device float*           root_w        [[buffer(2)]],
    device ushort*          root_tf       [[buffer(3)]],
    device float*           root_rot      [[buffer(4)]],
    device const float*     tri_vtx       [[buffer(5)]],
    device const float*     tri_norm      [[buffer(6)]],
    device const float*     tri_area      [[buffer(7)]],
    device const ushort*    tri_to_poly   [[buffer(8)]],
    constant GenRootKernelParams& gp      [[buffer(9)]],
    // scrum-268.8 (DR-3): per-ray wavelength pool + per-ray wl_idx output.
    // Pool entries provide the spd_weight (replacing the deleted gp.ray_weight)
    // and feed the trace kernel's per-ray optics on the next stage.
    // scrum-268.8 (DR-3): match trace_layer_kernel's `constant` binding so the
    // shader-side cache is consistent across both pool readers.
    constant WlEntry*       wl_pool       [[buffer(10)]],
    device uint*            root_wl_idx   [[buffer(11)]],
    // scrum-328.2 Step 1 attempt-count observability (test-only). The buffer
    // is ALWAYS bound (Metal validation forbids nil bindings); the .x flag in
    // attempts_ctrl selects whether the kernel actually writes. Production:
    // ctrl.x==0 → the branch below skips both the pointer arg and the write.
    device int*             lat_attempts  [[buffer(12)]],
    constant uint2&         attempts_ctrl [[buffer(13)]],
    // 330.2 S3b: unified area-measure inverse-CDF latitude LUT (read only when
    // gp.lat_path == kLatPathLutInverseCdf). Always bound (Metal forbids nil).
    device const float*     lat_lut_theta [[buffer(14)]],
    device const float*     lat_lut_cdf   [[buffer(15)]],
    device const float*     lat_lut_flip  [[buffer(16)]],
    // K-shape geometry pool (see UploadCrystalPool in metal_trace_backend.mm).
    // pool_shape_table[s] = uint4{poly_off, poly_cnt, tri_off, tri_cnt} for
    // pool slot s in the shared poly_*/tri_* buffers. root_pool_shape[tid]
    // stores this ray's chosen (poly_off, poly_cnt) — trace_layer_kernel reads
    // it back at buffer(16) to know which slice of poly_n/poly_d/centroid to
    // walk for its ray-polygon intersection loop.
    device const uint4*     pool_shape_table [[buffer(19)]],
    device uint2*           root_pool_shape  [[buffer(20)]],
    uint tid [[thread_position_in_grid]])
{
  if (tid >= gp.num_rays) {
    return;
  }
  // task-260.5 Step 3: categorical_sample(n=0) underflows uint and returns
  // 0xffffffff, which then indexes tri_vtx / tri_to_poly OOB. tri_count==0
  // should be impossible (host EnsureTriBuffers asserts tri_cnt > 0) but
  // a defensive early-out here makes the contract local and avoids a GPU
  // hang/crash if the host invariant ever breaks.
  if (gp.tri_count == 0u) {
    return;
  }
  // task-gpu-rng-ray-index-uint64: `gp.gen_ray_base` is only the low 32 bits
  // of the 64-bit host ray-base; `gp.gen_ray_base_hi` is the high 32 bits.
  // pcg_advance_hi carry-detects whether (base_lo + tid) crossed the 2^32
  // boundary so this thread's actual "hi epoch" may be gp.gen_ray_base_hi or
  // gp.gen_ray_base_hi + 1. pcg_seed_with_high(seed, hi) is identity when
  // hi==0 → in-range sessions (< 2^32 rays) remain bit-exact with the pre-fix
  // stream; hi!=0 mixes pcg_hash(hi) into the seed so wrapped rays cannot
  // collapse onto lower-epoch rays' streams. `global_idx` is base_lo + tid
  // (uint32 wrap-allowed): its post-wrap value is exactly the correct lo half
  // of the wrapped epoch, so the (mixed_seed, global_idx, slot) tuple remains
  // unique in the full 64-bit space.
  uint hi_epoch = pcg_advance_hi(gp.gen_ray_base, gp.gen_ray_base_hi, tid);
  uint mixed_seed = pcg_seed_with_high(gp.gen_seed, hi_epoch);
  uint global_idx = gp.gen_ray_base + tid;
  PcgStream stream;
  stream.seed = mixed_seed;
  stream.global_idx = global_idx;
  stream.slot = 0u;

  // Per-ray wavelength index (photon lifetime tag → root_wl_idx, later
  // consumed by the trace kernel to look up n_idx / cmf_* from
  // wl_pool[wl_idx]). Uses a SEED-DOMAIN-isolated PCG stream constructed by
  // BuildWlStream (pcg_shared.h — see the kWlStreamNonce block for design
  // rationale). The historical "slot 20" isolation was root-cause of the
  // laplacian near-pole green-tint bug (task-gpu-wl-stream-decouple-green-tint)
  // because near-pole GenericReject can consume >20 slots per ray, colliding
  // the wl draw with an orientation draw. Seed-domain XOR is immune to slot
  // consumption regardless of how many slots the orientation loop burns.
  PcgStream wl_stream = BuildWlStream(mixed_seed, global_idx);
  uint wl_idx = (uint)(pcg_uniform(wl_stream) * float(gp.wl_pool_size));
  if (wl_idx >= gp.wl_pool_size) {
    wl_idx = gp.wl_pool_size - 1u;  // guard against pcg_uniform → 1.0f rounding
  }
  root_wl_idx[tid] = wl_idx;

  // 1. Sample crystal orientation (lon, lat, roll) → 3×3 rotation.
  float lon, lat, roll;
  // scrum-328.2 Step 1: attempt-count observability — writes the per-ray
  // rejection-loop iteration count when the sibling buffer is armed (always 1
  // since 330.3; every path is rejection-free).
  // (attempts_ctrl.x!=0). ctrl.y is the multi-ci write-offset. Production:
  // ctrl.x==0 → &attempts_local skipped, branch predicted off.
  thread int attempts_local = 1;
  // 330.2 S3: LUT arrays are nullptr until the buffers are bound (S3b) — safe because the
  // kLatPathLutInverseCdf branch is dormant until SelectLatPath is flipped (S5).
  sample_lat_lon_roll(stream, gp, lat_lut_theta, lat_lut_cdf, lat_lut_flip, lon, lat, roll,
                      attempts_ctrl.x != 0u ? &attempts_local : nullptr);
  if (attempts_ctrl.x != 0u) {
    lat_attempts[tid + attempts_ctrl.y] = attempts_local;
  }
  float mat9[9];
  build_crystal_rotation_9(lon, lat, roll, mat9);

  // 2. Sample incident direction in WORLD space, then rotate into crystal-local.
  float d_world[3];
  sample_sph_cap(stream, gp.sun_lon, gp.sun_lat, gp.sun_half_angle, d_world);
  float d_crystal[3];
  apply_inverse_mat9(mat9, d_world, d_crystal);

  // 2.5. K-shape pool: pick this ray's shape via a SEED-DOMAIN-isolated PCG
  // draw (BuildGeomShapeStream / kGeomShapeStreamNonce). Independent seed
  // domain matters — sharing seed with orientation/wl would silently collide
  // (same failure mode that shifted metal_lap green-excess before wl was
  // isolated). P_ci == 1 (knob unset / deterministic params / host-gen
  // fallback) collapses this to a no-op: shape_slot = 0, floor(u * 1) = 0,
  // pool_shape_table[0] = (0, poly_cnt_full, 0, tri_cnt_full) — the historical
  // single-shape layout falls out bit-for-bit. The draw is UNCONDITIONAL so
  // there is no branch divergence per warp (AC2 bit-identity argument is
  // structural, not gated on a runtime if).
  PcgStream shape_stream = BuildGeomShapeStream(mixed_seed, global_idx);
  float u_shape = pcg_uniform(shape_stream);
  uint pool_slot = min(uint(u_shape * float(gp.pool_shape_count)),
                       gp.pool_shape_count - 1u);
  uint4 shape_info = pool_shape_table[pool_slot];
  uint poly_off = shape_info.x;
  uint tri_off  = shape_info.z;
  uint tri_cnt  = shape_info.w;
  // Publish this ray's chosen polygon slice for trace_layer_kernel to read
  // (buffer(16), r_pool_shape). poly_cnt is what the trace kernel's
  // ray-polygon loop needs; poly_off gets it into the right slice of poly_n /
  // poly_d / centroid.
  root_pool_shape[tid] = uint2(poly_off, shape_info.y);

  // 3. Triangle area×facing weighted pick → uniform point on the chosen tri.
  //    Restricted to THIS shape's [tri_off, tri_off + tri_cnt) window. Legacy
  //    InitRay_p_fid (simulator.cpp:106-124) is single-shape; we walk it
  //    unchanged inside the shape's window (same proj_prob = max(-d·n * area,
  //    0), same categorical_sample, same sample_triangle, same tri_to_poly
  //    lookup — the only difference is that tri_id / tri_to_poly indices are
  //    ABSOLUTE positions in the flattened pool buffer, which UploadCrystalPool
  //    already baked into tri_to_poly's values).
  float proj_prob[kMaxTriPerKernel];
  uint n_tri = min(tri_cnt, kMaxTriPerKernel);
  for (uint t = 0u; t < n_tri; t++) {
    uint g = tri_off + t;
    float dot = d_crystal[0] * tri_norm[g * 3 + 0]
              + d_crystal[1] * tri_norm[g * 3 + 1]
              + d_crystal[2] * tri_norm[g * 3 + 2];
    proj_prob[t] = max(-dot * tri_area[g], 0.0f);
  }
  float u_cat = pcg_uniform(stream);
  uint local_tri = categorical_sample(proj_prob, n_tri, u_cat);
  uint tri_id = tri_off + local_tri;
  float p[3];
  sample_triangle(stream, tri_vtx + tri_id * 9u, p);
  ushort to_face = tri_to_poly[tri_id];
  float weight = wl_pool[wl_idx].spd_weight;  // scrum-268.8 per-ray spd weight
  if (to_face == kInvalidId) {
    // Mirrors InitRay_p_fid fallback (simulator.cpp:92-94): zero weight when
    // a triangle has no polygon backing so downstream HitSurface can drop it.
    weight = 0.0f;
  }

  // 4. Emit.
  root_d[tid * 3 + 0] = d_crystal[0];
  root_d[tid * 3 + 1] = d_crystal[1];
  root_d[tid * 3 + 2] = d_crystal[2];
  root_p[tid * 3 + 0] = p[0];
  root_p[tid * 3 + 1] = p[1];
  root_p[tid * 3 + 2] = p[2];
  root_w[tid] = weight;
  root_tf[tid] = to_face;
  for (uint k = 0u; k < 9u; k++) {
    root_rot[tid * 9u + k] = mat9[k];
  }
}

// scrum-267 task-device-resident-continuation Step 1: device frame-transit
// kernel. Reads world-space continuation rays (cont_d_in / cont_w_in) produced
// by the prior layer's emit gate and emits root_*_buf for the next layer's
// trace dispatch. Mirrors the legacy InitRayOtherMs (simulator.cpp:199-210)
// three responsibilities now lifted onto the GPU:
//   1) sample a fresh per-ray crystal orientation (InitRay_rot equivalent),
//   2) rotate world dir into the NEW crystal-local frame (ApplyInverse), and
//   3) sample entry point + hit face on the NEW crystal (InitRay_p_fid).
// gp.gen_seed must be derived from transit_seed (independent from root-gen and
// gate PCG streams); gp.gen_ray_base MUST advance across SimBatches via the
// host-side transit_ray_count_ counter (same monotone contract as gen_root's
// root_ray_count) so per-batch transit dispatches on the same (layer,ci) key
// consume DISJOINT PCG ranges — otherwise tid=k of every batch collapses onto
// the same orientation, severely under-sampling crystal orientations across
// batches (scrum-267 bugfix). The sun-* / ray_weight fields in gp are unused
// (world dir comes from cont_d_in instead of sample_sph_cap; weight is
// carried through from cont_w_in).
kernel void transit_root_kernel(
    device const float*  cont_d_in   [[buffer(0)]],
    device const float*  cont_w_in   [[buffer(1)]],
    device float*        root_d      [[buffer(2)]],
    device float*        root_p      [[buffer(3)]],
    device float*        root_w      [[buffer(4)]],
    device ushort*       root_tf     [[buffer(5)]],
    device float*        root_rot    [[buffer(6)]],
    device const float*  tri_vtx     [[buffer(7)]],
    device const float*  tri_norm    [[buffer(8)]],
    device const float*  tri_area    [[buffer(9)]],
    device const ushort* tri_to_poly [[buffer(10)]],
    constant GenRootKernelParams& gp [[buffer(11)]],
    // scrum-268.8 (DR-3): per-ray wavelength carrier through the layer hop.
    // The emit gate wrote each continuation ray's wl_idx into cont_wl_idx_in;
    // transit pass-through copies it to root_wl_idx_out so the next layer's
    // trace kernel reads the same lifetime tag.
    device const uint*   cont_wl_idx_in   [[buffer(12)]],
    device uint*         root_wl_idx_out  [[buffer(13)]],
    // 330.2 S3b: unified latitude LUT (read only when lat_path==LutInverseCdf).
    device const float*  lat_lut_theta    [[buffer(14)]],
    device const float*  lat_lut_cdf      [[buffer(15)]],
    device const float*  lat_lut_flip     [[buffer(16)]],
    // task-331.5: per-ray component mask pass-through (sibling of wl_idx).
    // Rebased onto scrum-332: LUT took 14/15/16, so component moved 14/15→17/18.
    device const ulong*  cont_component_in  [[buffer(17)]],
    device ulong*        root_component_out [[buffer(18)]],
    // K-shape geometry pool (same bindings as gen_root_kernel). Every layer
    // hop resolves a fresh pool per (layer, ci), so transit picks its shape
    // independently — no cross-layer carry.
    device const uint4*  pool_shape_table [[buffer(19)]],
    device uint2*        root_pool_shape  [[buffer(20)]],
    uint tid [[thread_position_in_grid]])
{
  if (tid >= gp.num_rays || gp.tri_count == 0u) {
    return;
  }
  // 1. Orientation sample (shares sample_lat_lon_roll with gen_root_kernel;
  //    gp.gen_seed carries transit_seed, gp.gen_ray_base + gp.gen_ray_base_hi
  //    together carry the 64-bit host-side transit_ray_count_ so the mixed
  //    (seed, global_idx, slot) tuple is unique per (layer, ci, batch, tid)
  //    across the whole Run() PCG range even past 2^32 rays. See
  //    pcg_advance_hi / pcg_seed_with_high in pcg_shared.h — hi==0 is a
  //    no-op so in-range sessions stay bit-exact with pre-fix streams.
  uint hi_epoch = pcg_advance_hi(gp.gen_ray_base, gp.gen_ray_base_hi, tid);
  uint mixed_seed = pcg_seed_with_high(gp.gen_seed, hi_epoch);
  uint global_idx = gp.gen_ray_base + tid;
  PcgStream stream;
  stream.seed = mixed_seed;
  stream.global_idx = global_idx;
  stream.slot = 0u;
  float lon, lat, roll;
  // scrum-328.2 Step 1: transit surface does not carry attempt-count today
  // (near-pole acceptance-rate observation is anchored to the gen kernel).
  sample_lat_lon_roll(stream, gp, lat_lut_theta, lat_lut_cdf, lat_lut_flip, lon, lat, roll, nullptr);  // 330.2 S3b
  float mat9[9];
  build_crystal_rotation_9(lon, lat, roll, mat9);

  // 2. Read world-space continuation direction, rotate into crystal-local.
  float d_world[3] = { cont_d_in[tid * 3u + 0u],
                       cont_d_in[tid * 3u + 1u],
                       cont_d_in[tid * 3u + 2u] };
  float d_crystal[3];
  apply_inverse_mat9(mat9, d_world, d_crystal);

  // 2.5. K-shape pool: pick this ray's shape. Same seed-domain-isolated PCG
  // draw pattern as gen_root_kernel — mixed_seed here already carries the
  // transit_seed XOR (kTransitNonce ^ per-(layer, ci) nonce), so XOR-ing
  // kGeomShapeStreamNonce on top keeps this draw independent from the
  // orientation stream both across layers and within the same tid. P_ci == 1
  // collapses to the historical single-shape behavior bit-for-bit (unchanged
  // for parity when the K knob is unset).
  PcgStream shape_stream = BuildGeomShapeStream(mixed_seed, global_idx);
  float u_shape = pcg_uniform(shape_stream);
  uint pool_slot = min(uint(u_shape * float(gp.pool_shape_count)),
                       gp.pool_shape_count - 1u);
  uint4 shape_info = pool_shape_table[pool_slot];
  uint poly_off = shape_info.x;
  uint tri_off  = shape_info.z;
  uint tri_cnt  = shape_info.w;
  root_pool_shape[tid] = uint2(poly_off, shape_info.y);

  // 3. Triangle area×facing weighted pick → uniform point on the chosen tri.
  //    Restricted to the picked shape's window in the flattened pool buffer,
  //    identical to gen_root_kernel §3.
  float proj_prob[kMaxTriPerKernel];
  uint n_tri = min(tri_cnt, kMaxTriPerKernel);
  for (uint t = 0u; t < n_tri; t++) {
    uint g = tri_off + t;
    float dot = d_crystal[0] * tri_norm[g * 3u + 0u]
              + d_crystal[1] * tri_norm[g * 3u + 1u]
              + d_crystal[2] * tri_norm[g * 3u + 2u];
    proj_prob[t] = max(-dot * tri_area[g], 0.0f);
  }
  float u_cat = pcg_uniform(stream);
  uint local_tri = categorical_sample(proj_prob, n_tri, u_cat);
  uint tri_id = tri_off + local_tri;
  float p[3];
  sample_triangle(stream, tri_vtx + tri_id * 9u, p);
  ushort to_face = tri_to_poly[tri_id];

  // 4. Carry continuation weight; mirror InitRay_p_fid fallback (zero weight
  //    when a triangle has no polygon backing).
  float w = cont_w_in[tid];
  if (to_face == kInvalidId) {
    w = 0.0f;
  }

  // 5. Emit.
  root_d[tid * 3u + 0u] = d_crystal[0];
  root_d[tid * 3u + 1u] = d_crystal[1];
  root_d[tid * 3u + 2u] = d_crystal[2];
  root_p[tid * 3u + 0u] = p[0];
  root_p[tid * 3u + 1u] = p[1];
  root_p[tid * 3u + 2u] = p[2];
  root_w[tid] = w;
  root_tf[tid] = to_face;
  for (uint k = 0u; k < 9u; k++) {
    root_rot[tid * 9u + k] = mat9[k];
  }
  // scrum-268.8 (DR-3): pass-through wavelength index (photon lifetime tag).
  root_wl_idx_out[tid] = cont_wl_idx_in[tid];
  // task-331.5: pass-through the OR-accumulated component mask.
  root_component_out[tid] = cont_component_in[tid];
}

// --- shuffle_cont_kernel ---------------------------------------------------
//
// Continuation-pool decorrelation shuffle (task-gpu-backend-recombine-shuffle).
// Gather-form Feistel permutation: each thread tid in [0, n) computes
// `src = feistel_bijection(tid, n, seed)` (shared lm_pcg, identical to the CUDA
// shuffle_cont_kernel) and copies (d, w, wl_idx) from in[src] to out[tid]. The
// dest must NOT alias the source, so the host passes cont[other_slot] as the
// dest and swaps the slot handles afterwards (see MetalTraceBackend::Recombine).
//
// Why: when a multi-CI layer's per-CI trace dispatches run, cont[written_slot]
// ends up grouped by parent CI. The next layer's per-CI slicing would then hand
// "parent-correlated" subsets to each child CI, biasing the ray→crystal pairing.
// Legacy host Fisher-Yates breaks this (simulator.cpp:946-950); explore-300
// confirmed both GPU backends were missing the same decorrelation step.
kernel void shuffle_cont_kernel(
    device const float*  in_d   [[buffer(0)]],   // 3 × n
    device const float*  in_w   [[buffer(1)]],   // n
    device const uint*   in_wl  [[buffer(2)]],   // n
    device float*        out_d  [[buffer(3)]],   // 3 × n
    device float*        out_w  [[buffer(4)]],   // n
    device uint*         out_wl [[buffer(5)]],   // n
    constant uint&       n_rays [[buffer(6)]],
    constant uint&       seed   [[buffer(7)]],
    // task-331.5: per-ray component mask must gather in lockstep with (d,w,wl).
    // Omitting it is the exact CPU LANDMINE (SwapRay) reproduced on device —
    // the mask would decorrelate from its ray across the layer boundary.
    device const ulong*  in_component  [[buffer(8)]],   // n
    device ulong*        out_component [[buffer(9)]],   // n
    uint tid [[thread_position_in_grid]])
{
  if (tid >= n_rays) {
    return;
  }
  uint src = feistel_bijection(tid, n_rays, seed);
  out_d[tid * 3u + 0u] = in_d[src * 3u + 0u];
  out_d[tid * 3u + 1u] = in_d[src * 3u + 1u];
  out_d[tid * 3u + 2u] = in_d[src * 3u + 2u];
  out_w[tid]  = in_w[src];
  out_wl[tid] = in_wl[src];
  out_component[tid] = in_component[src];
}
