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
    device atomic_uint*    exit_cnt [[buffer(15)]],
    device atomic_float*   exit_wsum [[buffer(16)]],
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
    uint tid [[thread_position_in_grid]]) {
  if (tid >= prm.num_rays) { return; }

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
              gate_filter_desc[gate_slot], gate_sub_desc_buf,
              path_local, gate_len,
              gate_getfn_bytes, gate_getfn_offsets,
              gate_slot, ray_dir_w, prm.crystal_config_id);
          if (filter_pass) {
            // Independent PCG stream for the prob draw — gate_seed is derived
            // from gen_seed_ XOR (ms_layer_idx, crystal_id) on the host so
            // two dispatches with the same global_idx draw different prob
            // values. tid as global_idx gives statistical (not bit-exact)
            // parity with the legacy host mt19937 stream (scrum-267 §3.6).
            PcgStream gate_stream;
            gate_stream.seed       = prm.gate_seed;
            gate_stream.global_idx = tid;
            gate_stream.slot       = 0u;
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
              }
              // scrum-267 task-fused-emit-gate: post-gate semantics — exit_cnt
              // / exit_wsum now tally "filter_pass polygon-exits" (gate dropped
              // filter_fail rays above; legacy meaning was "all polygon-exits").
              // Diagnostic-only counters; not consumed by parity tests.
              atomic_fetch_add_explicit(exit_cnt, 1u, memory_order_relaxed);
              atomic_fetch_add_explicit(exit_wsum, cw, memory_order_relaxed);
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
                  AccumXyzToPixel(image, uint(py_m) * prm.img_w + uint(px_m),
                                  cmf_x, cmf_y, cmf_z, cw);
                  if (pr_m.hits[hi].bump_landed) {
                    atomic_fetch_add_explicit(landed_weight, cw, memory_order_relaxed);
                  }
                }
              }
              // Diagnostic counters (not consumed by parity tests).
              atomic_fetch_add_explicit(exit_cnt, 1u, memory_order_relaxed);
              atomic_fetch_add_explicit(exit_wsum, cw, memory_order_relaxed);
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
              gate_filter_desc[gate_slot_f], gate_sub_desc_buf,
              path_local_f, gate_len_f,
              gate_getfn_bytes, gate_getfn_offsets,
              gate_slot_f, ray_dir_w_f, prm.crystal_config_id);
          // Final-layer prob draw — independent PCG stream per dispatch (same
          // construction as the ms_mode==1 emit gate). rng<ms_prob → drop.
          PcgStream gate_stream_f;
          gate_stream_f.seed       = prm.gate_seed;
          gate_stream_f.global_idx = tid;
          gate_stream_f.slot       = 0u;
          bool prob_drop_f = (pcg_uniform(gate_stream_f) < prm.ms_prob);
          if (filter_pass_f && !prob_drop_f) {
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
                uint pix = (uint(py_f) * prm.img_w + uint(px_f)) * 3u;
                atomic_fetch_add_explicit(&image[pix + 0u], cmf_x * cw, memory_order_relaxed);
                atomic_fetch_add_explicit(&image[pix + 1u], cmf_y * cw, memory_order_relaxed);
                atomic_fetch_add_explicit(&image[pix + 2u], cmf_z * cw, memory_order_relaxed);
                if (pr_f.hits[hi].bump_landed) {
                  atomic_fetch_add_explicit(landed_weight, cw, memory_order_relaxed);
                }
              }
            }
            atomic_fetch_add_explicit(exit_cnt, 1u, memory_order_relaxed);
            atomic_fetch_add_explicit(exit_wsum, cw, memory_order_relaxed);
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
  // kLatPathGenericReject iteration count when the sibling buffer is armed
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

  // 3. Triangle area×facing weighted pick → uniform point on the chosen tri.
  float proj_prob[kMaxTriPerKernel];
  uint n_tri = min(gp.tri_count, kMaxTriPerKernel);
  for (uint t = 0u; t < n_tri; t++) {
    float dot = d_crystal[0] * tri_norm[t * 3 + 0]
              + d_crystal[1] * tri_norm[t * 3 + 1]
              + d_crystal[2] * tri_norm[t * 3 + 2];
    proj_prob[t] = max(-dot * tri_area[t], 0.0f);
  }
  float u_cat = pcg_uniform(stream);
  uint tri_id = categorical_sample(proj_prob, n_tri, u_cat);
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

  // 3. Triangle area×facing weighted pick → uniform point on the chosen tri.
  float proj_prob[kMaxTriPerKernel];
  uint n_tri = min(gp.tri_count, kMaxTriPerKernel);
  for (uint t = 0u; t < n_tri; t++) {
    float dot = d_crystal[0] * tri_norm[t * 3u + 0u]
              + d_crystal[1] * tri_norm[t * 3u + 1u]
              + d_crystal[2] * tri_norm[t * 3u + 2u];
    proj_prob[t] = max(-dot * tri_area[t], 0.0f);
  }
  float u_cat = pcg_uniform(stream);
  uint tri_id = categorical_sample(proj_prob, n_tri, u_cat);
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
}
