// Metal backend for TraceBackend (Apple platforms). Pimpl over Objective-C
// Metal types so the public header stays pure C++. The kernel source of truth
// is `src/core/metal/lumice_trace.metal`, which CMake precompiles to a
// .metallib at build time. The embedded bytes are loaded via
// newLibraryWithData (see LoadMetalLibrary), bypassing the macOS 26.5 broken
// MSL source frontend. The source string is also embedded as a fallback (and
// is suppressed by LUMICE_DISABLE_METAL_SOURCE_COMPILE for the AC2 regression
// test). See task-#283 (metal-build-time-metallib).

#import <Foundation/Foundation.h>
#import <Metal/Metal.h>

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include "config/color_class_table.hpp"  // task-358.1 Step 4 device Y-lane accumulation
#include "config/color_gate_table.hpp"  // task-358.1 Design-2 color-gate migration
#include "config/component_table.hpp"  // task-331.5 raypath-color foundation
#include "config/proj_config.hpp"
#include "config/raypath_color_config.hpp"  // task-358.1
#include "config/render_config.hpp"
#include "config/sim_data.hpp"
#include "core/color_util.hpp"
#include "core/crystal.hpp"
#include "core/device_filter_desc.hpp"
#include "core/exit_seam.hpp"
#include "core/filter_spec.hpp"
#include "core/geo3d.hpp"
#include "core/lat_lut.hpp"          // BuildLatLut / LatLut (330.2 unified LUT sampler)
#include "core/lens_proj_build.hpp"  // BuildProjParams / ProjParams (315.3 unified render projection)
#include "core/math.hpp"
#include "core/metal_filter_match_src.hpp"
#include "core/backend/metal_trace_backend.hpp"
#include "core/backend/metal_trace_backend_test_hooks.hpp"  // scrum-328.2 Step 3
#include "core/backend/wl_pool.hpp"  // WlEntry / ComputeWlPool / ResolveWlPoolSize (296.6 single-sourced)
#include "core/shared/lat_path_selection.hpp"  // SelectLatPath (single source)
#include "core/shared/pcg_shared.h"             // lm_pcg::kLatPath* (device sink values)
// task-#283 (metal-build-time-metallib): build-generated headers — wrap their
// content in `namespace lumice { ... }`, so they are #included at file scope
// (NOT inside an open `namespace lumice` or anonymous namespace) to keep the
// embedded symbols at `lumice::kLumice...` rather than nested deeper.
#include "lumice_trace_metallib_embed.h"   // lumice::kLumiceMetallibBytes / Size
#include "lumice_trace_src_embed.h"         // lumice::kLumiceCombinedKernelSrc
#include "core/projection.hpp"
#include "core/raypath.hpp"
#include "core/scatter_accum.hpp"
#include "core/simulator.hpp"  // PartitionCrystalRayNum
#include "core/trace_ops.hpp"
#include "util/color_data.hpp"
#include "util/env_knobs.hpp"
#include "util/illuminant.hpp"
#include "util/logger.hpp"

namespace lumice {

namespace {

// scrum-267 task-fused-emit-gate Step 1: kRecCap (trace kernel hop budget) and
// kDevRecCap (filter-match helper path-byte budget) BOTH live in MSL constant
// strings — they must agree numerically, or `path_local[kDevRecCap]` in the
// emit gate cannot accept the trace kernel's full `path[kRecCap]`. Mirror the
// trace-kernel value here so this static_assert traps a future drift the
// moment one constant is bumped.
constexpr int kTraceKernelRecCap = 64;
static_assert(kDevFilterMatchRecCap == kTraceKernelRecCap,
              "kDevRecCap (filter-match helper) and kRecCap (trace kernel) must match");

// task-358.1 (metal-color-parity, Design-2 migration): per-slot max number of
// color-descriptor symmetry groups (each group = one synthesized
// ComplexFilterParam wrapping predicates that share the same P/B/D bitmask).
// The 3-bit P/B/D bitmask spans 8 values in principle, but production
// raypath_color configs cluster on {kSymNone, kSymP, kSymB, kSymPB} — 4 is a
// tight soft cap; assert fires on overflow (mirrors kDeviceFilterMaxOrClauses).
// Raising it: bump this constant AND the MSL sibling (kColorMaxGroupsPerSlot in
// lumice_trace.metal) together, and audit color_desc_offset / color_bits_offset
// arithmetic downstream. See plan §4 Step 1 + assumption B.
constexpr size_t kColorMaxGroupsPerSlot = 4;

// task-358.1 Step 4 (AC3 device-side Y-lane accumulation): host-side upper
// bound on the number of runtime color classes shipped into the trace kernel.
// KernelParams carries `color_class_count`, `color_class_bits[kMaxColorClassesDevice]`
// (uint64) and `color_class_combine[kMaxColorClassesDevice]` (uint8) — sizing
// these arrays statically keeps the parameter block simple and avoids the
// extra buffer binding a variable-length descriptor would need. 16 comfortably
// covers current + planned raypath_color palettes (typical GUI configs use 2-8
// classes; scrum-342 documented palettes up to 12). Assert fires on overflow;
// raising the cap = bump this constant AND the MSL sibling
// (`kMaxColorClassesDeviceMsl`) together.
constexpr size_t kMaxColorClassesDevice = 16;

// Continuation-pool shuffle PCG nonce (task-gpu-backend-recombine-shuffle).
// MUST equal the CUDA-side kCudaShuffleNonce so the two backends derive the
// same per-layer Feistel seed (spec.seed ^ nonce ^ ms_idx) — the shuffle only
// needs statistical uniformity, not bit-match with legacy host Fisher-Yates,
// but keeping the nonces identical keeps the two GPU backends in lock-step.
constexpr uint32_t kMetalShuffleNonce = 0xB17CA3D9u;

// Mirror of the Metal-side KernelParams (host layout MUST match the .metal
// struct field-for-field — all 4-byte scalars, natural alignment).
// NOTE: field order MUST match MSL KernelParams in src/core/metal/lumice_trace.metal — static_assert
// guards size only; reviewer-facing field-by-field check is the maintainer's
// responsibility when adding/reordering members.
struct KernelParams {
  // scrum-268.8 (DR-3): per-batch n_idx + cie_x/y/z removed. trace_layer
  // kernel reads per-ray optics from the wl_pool[wl_idx] buffer instead.
  uint32_t max_hits;
  uint32_t poly_cnt;
  uint32_t num_rays;
  uint32_t img_w;
  uint32_t img_h;
  uint32_t ms_mode;
  uint32_t out_cap;
  uint32_t exit_cap;  // exit seam (scrum-258.1) — buffer-egress capacity (rays)
  // Exit metadata (scrum-258.2). Field order MUST match the MSL struct above
  // (crystal_id, face_seq_cap, ms_layer_idx) — silent reorder would only
  // change semantics, not sizeof, so the static_assert below is necessary
  // but insufficient. Reviewer-side cross-check between host + MSL is
  // mandatory until layout_test (plan §7 Risk 2) lands.
  uint32_t crystal_id;
  uint32_t face_seq_cap;
  uint32_t ms_layer_idx;
  // Emit-gate (scrum-267 task-fused-emit-gate Step 2). Field order MUST match
  // the MSL struct above (ms_prob → gate_seed → filter_desc_max_ci →
  // crystal_config_id); reviewer-facing field-by-field check is required.
  float    ms_prob;
  uint32_t gate_seed;
  uint32_t filter_desc_max_ci;
  uint32_t crystal_config_id;
  // Unified render projection (315.3) — mirrors the MSL KernelParams::proj.
  // Filled by BeginSession via BuildProjParams; consumed by
  // lm_proj::ProjectExitToPixel in the kernel exit tail. Replaced the former
  // loose az0 / proj_type / r_scale / max_abs_dz fields.
  lm_proj::ProjParams proj;
  // task-358.3 (renamed from capture_component after Fork-C retirement): 0 in
  // production (emit gate skips capture-ring append), 1 in the CPU-parity test
  // (append the ray's (this_mask, weight) — now purely Design-2 colour bits —
  // to the capture ring). Mirrors MSL KernelParams::capture_ray_mask.
  uint32_t capture_ray_mask;
  // task-358.1 (metal-color-parity, Design-2 ColorGateTable migration):
  //   has_color_groups        : 1 iff ColorGateTable non-empty AND at least one
  //                             placement has ≥1 color group in this session; 0
  //                             skips the color pass entirely (zero-cost path).
  //   color_desc_offset       : index (in DeviceFilterDesc units) into
  //                             filter_desc_buf_ where the color-descriptor
  //                             region starts (= physical n_slot). Kernel reads
  //                             gate_filter_desc[color_desc_offset +
  //                                              gate_slot * kColorMaxGroupsPerSlot + g]
  //                             for group g of the current slot. See plan §4
  //                             Step 1 addressing公式 + assumption B.
  //   color_bits_offset       : index (in uint8_t units) into
  //                             gate_component_bits_buf_ where the color-region
  //                             begins (= physical n_slot * stride).
  //   color_max_groups_per_slot: stride for the color-descriptor region and the
  //                              color-component-bit region (kColorMaxGroupsPerSlot,
  //                              currently 4). Passed as a param to avoid
  //                              hard-coding in MSL.
  uint32_t has_color_groups;
  uint32_t color_desc_offset;
  uint32_t color_bits_offset;
  uint32_t color_max_groups_per_slot;
  // task-358.1 Step 4 (AC3 device-side Y-lane accumulation): per-class predicate
  // over each ray's OR-accumulated component mask, evaluated in the emit gate.
  //   color_class_count       : number of active classes (0 → skip lane accum
  //                             entirely; class_lane_buf_ stays a 4-byte dummy).
  //   color_class_bits        : per-class OR-union of member component bits
  //                             (matches ColorClass.member_bits_).
  //   color_class_combine     : per-class predicate combinator; 0 = kAny (mask &
  //                             bits) != 0, 1 = kAll (mask & bits) == bits.
  //
  // Layout: color_class_count sits at offset 156 (the 4 preceding step-1+2
  // fields bring the running offset to 156, itself 4-aligned but not 8-aligned).
  // A single uint32_t at 156 lands color_class_bits at offset 160 which is
  // exactly 8-aligned (uint64_t's requirement) — no explicit padding needed.
  uint32_t color_class_count;
  uint64_t color_class_bits[kMaxColorClassesDevice];
  uint8_t  color_class_combine[kMaxColorClassesDevice];
  // task-device-flat-and-terms: byte offset (in bytes) inside the shared
  // complex_sub_desc_buf_ where the flat `and_term_counts` region begins. The
  // kernel derives `gate_and_term_counts = base + offset` via reinterpret_cast
  // (both regions share buffer 27 to stay within Metal's 30-buffer per-stage
  // cap). Zero when no Complex filter is present. Must match the MSL field of
  // the same name in `KernelParams`.
  uint32_t and_term_counts_base_offset;
};
// sizeof(ProjParams) == 76 (6 ints + 4 floats + float[9]); the 15 leading
// 4-byte KernelParams scalars add 60, + proj 76 + capture_ray_mask 4 → 140
// pre-358.1. task-358.1 Step 1+2 adds 4 more 4-byte fields (color has/offset/
// bits/stride) → 156. Step 4 adds color_class_count (4, at offset 156) +
// color_class_bits[16] (128, at 160, naturally 8-aligned) +
// color_class_combine[16] (16, at 288) → 304. 304 % 8 = 0 → no trailing pad.
// task-358.3 renamed capture_component → capture_ray_mask (same 4B slot, no
// layout change; the 304 assert below is unaffected).
static_assert(sizeof(lm_proj::ProjParams) == 76u, "ProjParams layout drift — check projection_shared.h");
// task-device-flat-and-terms: appends a single uint32_t field
// `and_term_counts_base_offset` (offset 304 → 308 bytes) but the struct's
// alignment is 8 (uint64_t member `color_class_bits`), so the tail is rounded
// up to 312.
static_assert(sizeof(KernelParams) == 312u,
              "KernelParams size mismatch — update host struct to match Metal-side layout");

// Device root-gen latitude path tags. Numeric wire encoding is single-sourced
// in lm_pcg::kLatPath* (src/core/shared/pcg_shared.h); the pairwise
// static_assert below guards against LatPathKind (host taxonomy) vs
// lm_pcg::kLatPath* (device sink) drift.
static_assert(lat_path::ToWireValue(lat_path::LatPathKind::kFullSphere) == lm_pcg::kLatPathFullSphere,
              "LatPathKind::kFullSphere must match lm_pcg::kLatPathFullSphere");
static_assert(lat_path::ToWireValue(lat_path::LatPathKind::kNoRandom) == lm_pcg::kLatPathNoRandom,
              "LatPathKind::kNoRandom must match lm_pcg::kLatPathNoRandom");
static_assert(lat_path::ToWireValue(lat_path::LatPathKind::kGaussLegacy) == lm_pcg::kLatPathGaussLegacy,
              "LatPathKind::kGaussLegacy must match lm_pcg::kLatPathGaussLegacy");
static_assert(lat_path::ToWireValue(lat_path::LatPathKind::kLutInverseCdf) == lm_pcg::kLatPathLutInverseCdf,
              "LatPathKind::kLutInverseCdf must match lm_pcg::kLatPathLutInverseCdf");

// Mirror of the Metal-side GenRootKernelParams (host layout MUST match the
// MSL struct field-for-field — all 4-byte scalars, natural alignment).
// Field order MUST match the MSL struct in src/core/metal/lumice_trace.metal — static_assert guards
// size only; reviewer-facing field-by-field check is required when adding
// or reordering members (same convention as KernelParams above).
// NOTE: consumed by gen_root_kernel and transit_root_kernel; see
// BuildTransitRootParams for the transit-side field subset (orientation +
// geometry only; sun_* / ray_weight are populated but unused by transit).
struct GenRootKernelParams {
  uint32_t gen_seed;
  // task-gpu-rng-ray-index-uint64: `gen_ray_base` is now only the LOW 32 bits
  // of the 64-bit host ray-index; `gen_ray_base_hi` carries the HIGH 32 bits
  // (split via lumice::SplitPcgRayBase). Device mixes hi into per-ray PCG
  // stream seed. Field order MUST match the MSL struct in
  // src/core/shared/pcg_shared.h (which lumice_trace.metal pulls in via
  // `using namespace lm_pcg;`); the static_assert below only guards size.
  uint32_t gen_ray_base;
  uint32_t gen_ray_base_hi;
  uint32_t num_rays;
  uint32_t tri_count;
  float    sun_lon;
  float    sun_lat;
  float    sun_half_angle;
  // scrum-268.8 (DR-3): host mirror of MSL `wl_pool_size`. Replaces the
  // per-batch `ray_weight` (now per-ray from wl_pool[wl_idx].spd_weight).
  uint32_t wl_pool_size;
  uint32_t lat_path;
  uint32_t lat_dist_type;   // dead post-330.3 (retired GenericReject path); wire layout frozen
  float    lat_mean_rad;
  float    lat_std_rad;
  float    lat_rejection_m; // dead post-330.3 (surviving paths rejection-free); wire layout frozen
  uint32_t lat_lut_n;       // node count for kLatPathLutInverseCdf (330.2); LUT arrays bound separately
  uint32_t az_type;
  float    az_mean_rad;
  float    az_std_rad;
  float    az_pad;
  uint32_t roll_type;
  float    roll_mean_rad;
  float    roll_std_rad;
  float    roll_pad;
};
static_assert(sizeof(GenRootKernelParams) == 92u,
              "GenRootKernelParams size mismatch — update host struct to match Metal-side layout");

size_t ComputeOutCap(size_t n, size_t max_hits) {
  // explore mh16 ~9.8x fan-out; (max_hits*2+4) covers the observed upper
  // bound with margin; cap at 64M to bound device-memory usage.
  size_t cap = n * (max_hits * 2 + 4);
  return std::min<size_t>(cap, 64u * 1024u * 1024u);
}

Logger& EffectiveLogger(Logger* logger) {
  return logger ? *logger : GetGlobalLogger();
}

// task-#283 (metal-build-time-metallib): build-time metallib loader.
//
// Loads the build-time precompiled metallib bytes embedded in the binary via
// newLibraryWithData. This path does NOT invoke the runtime MSL source frontend
// (which is broken on macOS 26.5 — see issue.md): the metallib contains AIR
// (Apple Intermediate Representation), and newLibraryWithData hands it
// directly to the driver-level AIR→ISA codegen, which is unaffected.
//
// The dispatch_data_t wraps a static `constexpr` byte array — the destructor
// MUST be a no-op block (`^{}`), NOT DISPATCH_DATA_DESTRUCTOR_DEFAULT
// (which calls free() on the buffer — UB on a static-storage array).
id<MTLLibrary> LoadMetallibFromEmbeddedBytes(id<MTLDevice> device,
                                             NSError** out_err) {
  dispatch_data_t data = dispatch_data_create(
      kLumiceMetallibBytes,
      kLumiceMetallibSize,
      nullptr,                       // run callback on the default queue
      ^{}                            // no-op: static array, never freed
  );
  NSError* err = nil;
  id<MTLLibrary> lib = [device newLibraryWithData:data error:&err];
  // Contract: *out_err is written iff the load fails. Scoping the write to
  // lib == nil makes this hold for ANY caller, independent of caller behavior —
  // newLibraryWithData leaves err nil on success, so an unconditional write
  // would clobber a standalone caller's pre-set *out_err on the success path.
  // (The current sole caller, LoadMetalLibrary, also clears *out_err on its own
  // success paths, so the old unconditional write was benign in practice — but
  // the contract must not depend on that.)
  if (lib == nil && out_err) { *out_err = err; }
  return lib;
}

// Returns nil if neither the embedded metallib nor (optional) source fallback
// produces a usable library. Caller is responsible for converting nil into a
// BackendUnavailableError (EnsurePso) or a gate-off (MetalPipelineAvailable).
//
// Loading priority:
//   1. Primary: newLibraryWithData(embedded .metallib) — bypasses source frontend.
//   2. Fallback: newLibraryWithSource(embedded .metal text) — suppressed when
//      LUMICE_DISABLE_METAL_SOURCE_COMPILE is set (this is the AC1/AC2 switch
//      that simulates macOS 26.5's broken source frontend on dev machines so
//      the metallib path is verified to be self-sufficient).
//
// On success: *out_err is cleared to nil so a stale error from a previous
// failed attempt cannot mislead the caller (matches LoadMissingFunction's
// expectation that err reflects the FINAL outcome).
id<MTLLibrary> LoadMetalLibrary(id<MTLDevice> device,
                                Logger* logger,
                                NSError** out_err) {
  // Primary: embedded pre-compiled metallib (bypasses the macOS 26.5 broken
  // MSL source frontend).
  NSError* primary_err = nil;
  id<MTLLibrary> lib = LoadMetallibFromEmbeddedBytes(device, &primary_err);
  if (lib != nil) {
    // POSITIVELY surface "we took the embedded-metallib path" so the AC2
    // regression test can distinguish this from the source fallback (a
    // returncode==0 alone could not tell which load route ran).
    // INFO (not VERBOSE) so the AC2 regression sentinel can detect this
    // marker without juggling log levels. The event is one-shot per backend
    // instance (BeginSession → EnsurePso) — not per dispatch — so the noise
    // cost is bounded.
    ILOG_INFO(EffectiveLogger(logger),
              "MetalTraceBackend: loaded embedded metallib ({} functions)",
              (unsigned)lib.functionNames.count);
    if (out_err) { *out_err = nil; }
    return lib;
  }
  // Metallib load failed. On any macOS ≥13 with an unmodified binary this
  // should NEVER happen (AIR is forward-compatible). Log and consider the
  // source fallback.
  const char* primary_err_cstr =
      primary_err.localizedDescription.UTF8String;
  ILOG_WARN(EffectiveLogger(logger),
            "MetalTraceBackend: newLibraryWithData failed ({}); evaluating source fallback",
            primary_err_cstr ? primary_err_cstr : "(none)");

  // AC1/AC2 switch: when set, the runtime source compile path is suppressed
  // entirely. The dev machine then behaves like macOS 26.5 in the sense that
  // the source frontend is unreachable — exercising the metallib-only path.
  bool disable_source = env::DisableMetalSourceCompile(EffectiveLogger(logger));
  if (disable_source) {
    ILOG_WARN(EffectiveLogger(logger),
              "MetalTraceBackend: LUMICE_DISABLE_METAL_SOURCE_COMPILE=1 — "
              "skipping source fallback; reporting metallib failure");
    if (out_err) { *out_err = primary_err; }
    return nil;
  }

  // Fallback: runtime source compile (the legacy pre-task-#283 path).
  NSString* src = @(kLumiceCombinedKernelSrc);
  MTLCompileOptions* opts = [MTLCompileOptions new];
  // Pin MSL 3.0 + safe math so the source fallback produces a numerically
  // equivalent kernel to the offline metallib (see Step 2 compile flags).
  opts.languageVersion = MTLLanguageVersion3_0;
  if (@available(macOS 15.0, *)) {
    opts.mathMode = MTLMathModeSafe;
  } else {
    opts.fastMathEnabled = NO;
  }
  NSError* src_err = nil;
  id<MTLLibrary> src_lib = [device newLibraryWithSource:src
                                                options:opts
                                                  error:&src_err];
  if (src_lib != nil) {
    ILOG_INFO(EffectiveLogger(logger),
              "MetalTraceBackend: loaded source-compiled library ({} functions)",
              (unsigned)src_lib.functionNames.count);
    if (out_err) { *out_err = nil; }
    return src_lib;
  }
  // Both routes failed: surface the source-fallback err (the more informative
  // one, since metallib failure on a healthy install is silent / opaque).
  if (out_err) { *out_err = src_err; }
  return nil;
}

// task-metal-gui-commit-backpressure O2: process-level cache of Metal
// device/queue/PSOs. Rationale: `MetalTraceBackend` instances are per-Run
// (simulator.cpp CreateBackend) so on macOS the CommitConfig→Stop→Start cycle
// used to rebuild all 4 PSOs from the metallib on every commit — measured at
// ~100-150ms cost, larger than the 70ms GUI commit interval, which starved
// slider-drag previews. The cache stores only the immutable, device-tied
// objects; per-Run RNG state (rng/seeded/seeded_seed/gen_seed_/root_ray_count/
// transit_ray_count_) is NOT touched — see plan §3 design point 2.
//
// Note (logger param): only the FIRST successful call's logger is used for the
// one-time BuildMetalDeviceCache logs; subsequent callers' loggers are silently
// ignored by C++ function-static init semantics. Downstream per-instance logs
// still go through Impl::logger_.
struct MetalDeviceCache {
  id<MTLDevice>               device = nil;
  id<MTLCommandQueue>         queue = nil;
  id<MTLComputePipelineState> pso = nil;               // trace_layer_kernel
  id<MTLComputePipelineState> gen_root_pso = nil;      // gen_root_kernel
  id<MTLComputePipelineState> transit_root_pso = nil;  // transit_root_kernel
  id<MTLComputePipelineState> shuffle_pso = nil;       // shuffle_cont_kernel
};

MetalDeviceCache BuildMetalDeviceCache(Logger* logger) {
  MetalDeviceCache c;
  c.device = MTLCreateSystemDefaultDevice();
  if (c.device == nil) {
    NSArray<id<MTLDevice>>* all = MTLCopyAllDevices();
    if (all.count > 0) {
      c.device = all[0];
    }
  }
  if (c.device == nil) {
    ILOG_ERROR(EffectiveLogger(logger),
               "MetalTraceBackend: no Metal device available (cache build)");
    throw BackendUnavailableError("MetalTraceBackend: no Metal device available");
  }
  c.queue = [c.device newCommandQueue];
  if (c.queue == nil) {
    ILOG_ERROR(EffectiveLogger(logger),
               "MetalTraceBackend: newCommandQueue returned nil (cache build)");
    throw BackendUnavailableError("MetalTraceBackend: newCommandQueue failed");
  }
  NSError* err = nil;
  id<MTLLibrary> lib = LoadMetalLibrary(c.device, logger, &err);
  if (lib == nil) {
    const char* desc = err.localizedDescription.UTF8String;
    ILOG_ERROR(EffectiveLogger(logger),
               "MetalTraceBackend: kernel library load failed: {}",
               desc ? desc : "(no error)");
    throw BackendUnavailableError("MetalTraceBackend: kernel library load failed");
  }
  // Diagnostic helper matches EnsurePso's original message shape so downstream
  // repros / tests observe unchanged log text on failure.
  auto LogMissingFunction = [&](const char* name) {
    NSArray<NSString*>* names = lib.functionNames;
    NSString* joined = [names componentsJoinedByString:@", "];
    const char* names_cstr = joined.UTF8String;
    ILOG_ERROR(EffectiveLogger(logger),
               "MetalTraceBackend: kernel entry point '{}' missing — library functionNames=[{}]",
               name,
               names_cstr ? names_cstr : "");
  };
  auto BuildPso = [&](const char* name) -> id<MTLComputePipelineState> {
    id<MTLFunction> fn = [lib newFunctionWithName:[NSString stringWithUTF8String:name]];
    if (fn == nil) {
      LogMissingFunction(name);
      throw BackendUnavailableError("MetalTraceBackend: kernel entry point missing");
    }
    NSError* pso_err = nil;
    id<MTLComputePipelineState> pso = [c.device newComputePipelineStateWithFunction:fn
                                                                              error:&pso_err];
    if (pso == nil) {
      const char* desc = pso_err.localizedDescription.UTF8String;
      ILOG_ERROR(EffectiveLogger(logger),
                 "MetalTraceBackend: pipeline state creation failed for '{}': {}",
                 name,
                 desc ? desc : "(no error)");
      throw BackendUnavailableError("MetalTraceBackend: pipeline state creation failed");
    }
    return pso;
  };
  c.pso              = BuildPso("trace_layer_kernel");
  c.gen_root_pso     = BuildPso("gen_root_kernel");
  c.transit_root_pso = BuildPso("transit_root_kernel");
  c.shuffle_pso      = BuildPso("shuffle_cont_kernel");
  return c;
}

// C++11 guarantees function-local static initialization is thread-safe and
// retried on constructor exception ([stmt.dcl]p4) — a natural fit for the
// BackendUnavailableError throw path in BuildMetalDeviceCache. No manual
// once_flag needed.
const MetalDeviceCache& GetOrBuildMetalDeviceCache(Logger* logger) {
  static MetalDeviceCache cache = BuildMetalDeviceCache(logger);
  return cache;
}

}  // namespace

bool MetalDeviceAvailable() {
  static std::once_flag flag;
  static bool available = false;
  std::call_once(flag, []() {
    @autoreleasepool {
      // Mirror EnsureDevice's two-step probe: MTLCreateSystemDefaultDevice can
      // return nil on otherwise Metal-capable Macs in certain environments
      // (e.g. some headless / login-class contexts on M-series chips), while
      // MTLCopyAllDevices still enumerates the integrated GPU. Treating only
      // the system-default probe as authoritative would gate the GUI checkbox
      // off in those environments even though the Metal backend itself runs
      // fine via the same fallback in EnsureDevice.
      id<MTLDevice> probe = MTLCreateSystemDefaultDevice();
      if (probe == nil) {
        NSArray<id<MTLDevice>>* all = MTLCopyAllDevices();
        available = (all.count > 0);
      } else {
        available = true;
      }
    }
  });
  return available;
}

bool MetalPipelineAvailable() {
  // task-#283 (metal-build-time-metallib): the gate now goes through the SAME
  // LoadMetalLibrary helper EnsurePso uses, so there is no asymmetry between
  // "gate says Metal is available" and "BeginSession actually succeeds" — the
  // pre-task-#283 risk that the gate compiled source-from-string while
  // EnsurePso took a different MTLCompileOptions path is structurally gone.
  // The LUMICE_DISABLE_METAL_SOURCE_COMPILE env var (AC1/AC2 switch) is
  // honored by LoadMetalLibrary, so the gate also fails closed on dev machines
  // simulating the macOS 26.5 broken-source-frontend condition.
  //
  // task-282: cache is once_flag write-once-then-read-only — safe for any
  // thread to call after the first probe completes. We deliberately do NOT
  // build a full MTLComputePipelineState — entry-point lookup is sufficient
  // for the observed failure mode, and BeginSession's BackendUnavailableError
  // + simulator fallback is the safety net for any future runtime-only PSO
  // failure.
  static std::once_flag flag;
  static bool available = false;
  std::call_once(flag, []() {
    @autoreleasepool {
      id<MTLDevice> device = MTLCreateSystemDefaultDevice();
      if (device == nil) {
        NSArray<id<MTLDevice>>* all = MTLCopyAllDevices();
        if (all.count > 0) {
          device = all[0];
        }
      }
      if (device == nil) {
        ILOG_WARN(EffectiveLogger(nullptr),
                  "MetalPipelineAvailable: no Metal device — gating Metal backend off");
        return;  // available stays false
      }
      NSError* err = nil;
      id<MTLLibrary> lib = LoadMetalLibrary(device, nullptr, &err);
      if (lib == nil) {
        const char* err_cstr = err.localizedDescription.UTF8String;
        ILOG_WARN(EffectiveLogger(nullptr),
                  "MetalPipelineAvailable: library load failed — gating Metal backend off; err={}",
                  err_cstr ? err_cstr : "(none)");
        return;
      }
      // Check ALL three entry points EnsurePso resolves. Even when the load
      // returned a non-nil library, a degenerate library (functionNames=[]) —
      // the macOS 26.5 source-fallback signature, see task-282 — must still
      // gate Metal off. Checking only trace_layer_kernel would let the gate
      // return true while EnsurePso still throws on a later lookup; the cost
      // of three lookups on the same library object is trivial.
      for (NSString* name in @[ @"trace_layer_kernel", @"gen_root_kernel", @"transit_root_kernel" ]) {
        id<MTLFunction> fn = [lib newFunctionWithName:name];
        if (fn == nil) {
          NSArray<NSString*>* names = lib.functionNames;
          const char* names_cstr = [names componentsJoinedByString:@", "].UTF8String;
          ILOG_WARN(EffectiveLogger(nullptr),
                    "MetalPipelineAvailable: library loaded but kernel entry point '{}' missing — "
                    "gating Metal backend off; library functionNames=[{}]",
                    name.UTF8String,
                    names_cstr ? names_cstr : "");
          return;
        }
      }
      available = true;
    }
  });
  return available;
}

struct MetalTraceBackend::Impl {
  Impl() {
    // task-260.5 Step 4 (code-review-01 Minor): cache the device-gen escape
    // hatch once per backend instance. LUMICE_DISABLE_DEVICE_GEN is a
    // process-level config (single-worker server, no in-process re-config),
    // so reading it per-dispatch (the prior behavior) was needless overhead
    // and risked observing different values across BeginSession cycles if a
    // future test toggled it mid-instance. The ctor capture pairs naturally
    // with the test helpers ForceHostGenForByteIdentity / EnableDeviceGen…,
    // which already setenv BEFORE constructing a fresh MetalTraceBackend.
    // logger_ may still be null here (set by MetalTraceBackend's ctor after the
    // Impl is built); EffectiveLogger falls back to the global logger.
    disable_device_gen_ = env::DisableDeviceGen(EffectiveLogger(logger_));
  }

  Logger* logger_ = nullptr;
  // Captured at construction from LUMICE_DISABLE_DEVICE_GEN. See ctor above.
  bool disable_device_gen_ = false;
  // gen+trace fusion stash (task-264). When device-gen runs for a ci,
  // GenerateFirstLayerRootsForCi parks the GenRootKernelParams here instead of
  // dispatching its own command buffer; DispatchLayer then prepends an
  // EncodeGenRoot encode pass into its own cb so gen and trace share a single
  // Metal queue round-trip. Reset by DispatchLayer after consumption and by
  // Reset() as a defensive session-boundary cleanup.
  std::optional<GenRootKernelParams> pending_gen_params_{};

  // task-268.7 architectural-layer move: DispatchLayer now commits without
  // waiting; the last committed command buffer parks here so TraceLayer's
  // ci-loop can wait + read back at a single explicit sync point. This is a
  // STRUCTURAL precondition for the transit+trace CB merge below (the merged
  // CB still needs exactly one commit and one wait, just relocated). The
  // direct latency win comes from Step 3 (one CB per non-first MS ci instead
  // of two) and Step 1 (single-engine, no GPU contention); pending_cb_ alone
  // does not introduce in-flight overlap in the single-ci common case.
  id<MTLCommandBuffer> pending_cb_ = nil;
  int pending_out_slot_ = 0;  // out_slot for the dispatch parked in pending_cb_

  id<MTLDevice>               device = nil;
  id<MTLCommandQueue>         queue  = nil;
  id<MTLComputePipelineState> pso    = nil;
  // Device root-gen PSO (task-260.2). Compiled by EnsurePso alongside the
  // trace_layer kernel; nil until first BeginSession.
  id<MTLComputePipelineState> gen_root_pso_ = nil;
  // Device frame-transit PSO (scrum-267 task-device-resident-continuation).
  // Same compiled library as gen_root_pso_; nil until first BeginSession.
  id<MTLComputePipelineState> transit_root_pso_ = nil;
  // Continuation-pool shuffle PSO (task-gpu-backend-recombine-shuffle).
  // Same compiled library as the others; nil until first BeginSession.
  id<MTLComputePipelineState> shuffle_pso_ = nil;
  // Captured from spec.seed by BeginSession. 0 → device gen disabled (host
  // fallback path). Non-zero implies single-worker determinism contract.
  uint32_t gen_seed_ = 0u;

  SessionSpec spec{};
  bool   in_session = false;
  size_t ms_idx = 0;
  // First-layer root index running counter — globally monotone PCG index
  // across all SimBatches of a single Simulator::Run().
  //   * Reset to 0 ONLY at first seeding (BeginSession + !seeded gate), in
  //     lock-step with rng.SetSeed; Reset()/EndSession do NOT touch it.
  //   * Each GenerateFirstLayerRootsForCi device-gen dispatch advances it by
  //     the rays it emitted, so (gen_seed_, gen_ray_base + tid) is a unique
  //     PCG stream per ray within one Run(). Host-gen also advances it so a
  //     later device-gen-eligible call in the same session stays monotone.
  //   * Architectural pre-condition: backend instances are per-Run() (created
  //     in simulator.cpp:596 / CreateBackend), so `seeded` starts false on
  //     each new Run() and the counter restarts at 0. If a future refactor
  //     pools backend instances across Run()s, this contract still holds for
  //     PCG-stream determinism within a Run() but NOT across pooled Run()s —
  //     revisit cross-Run() seeding semantics at that point.
  // Value-initialized to 0 by Impl's = {} member initializer below; the
  // explicit `= 0` here also serves as the !seeded-gate's pre-seed contract.
  size_t root_ray_count = 0;
  // scrum-267 task-device-resident-continuation bugfix: per-Run() running
  // counter for transit_root_kernel dispatches, mirroring root_ray_count.
  //   * Key invariant: each cont ray's PCG stream key is
  //     (transit_seed(ms_layer,ci), transit_ray_count_ + tid). Without a
  //     monotone counter, every SimBatch's transit dispatch on (layer,ci) keys
  //     by (transit_seed, 0..ci_n-1) which COLLAPSES the per-batch ray
  //     orientations onto the same ~ci_n discrete draws, severely under-
  //     sampling crystal orientations across batches. See
  //     scratchpad/debug-metal-continuation-correctness/findings.md.
  //   * Reset semantics match root_ray_count exactly: zeroed only at first
  //     seeding (BeginSession + !seeded gate); persists across Reset()/
  //     EndSession so successive SimBatches consume disjoint PCG ranges.
  size_t transit_ray_count_ = 0;
  int    width = 0;
  int    height = 0;
  // scrum-268.8 (DR-3): per-batch cie_x/y/z removed (KernelParams fields
  // dropped; trace kernel reads CMF from wl_pool[wl_idx]).
  // Unified render projection (315.3) — populated by BeginSession via
  // BuildProjParams(render, camera_rot, short_pix). Copied into
  // KernelParams::proj by DispatchLayer; consumed by ProjectExitToPixel.
  lm_proj::ProjParams proj_params_{};

  RandomNumberGenerator rng{ 0 };
  // Seed-once flag — see CpuTraceBackend::seeded_ rationale (per-SimBatch
  // BeginSession would otherwise reset rng every 128-ray batch and collapse
  // axis-sample diversity). progress.md DONE 2026-06-10 15:35.
  bool seeded = false;
  uint32_t seeded_seed = 0;  // seed value used when seeded was set

  // Persistent crystal cache for the *current* TraceLayer (rebuilt per layer).
  Crystal current_crystal{};
  float   current_n_idx = 0.0f;
  bool    have_crystal = false;

  // XYZ accumulator (W*H*3 floats).
  id<MTLBuffer> xyz_image = nil;
  size_t        xyz_pix_capacity = 0;
  // scrum-312: dims xyz_image was actually allocated for (persist across sessions,
  // unlike width/height which Reset clears). Used by the between-session
  // third-clock drain to release-safe-verify the caller's dims.
  int           alloc_xyz_w_ = 0;
  int           alloc_xyz_h_ = 0;

  // Polygon geometry (uploaded per-layer; capacity-resized lazily).
  id<MTLBuffer> poly_n_buf  = nil;
  id<MTLBuffer> poly_d_buf  = nil;
  id<MTLBuffer> centroid_buf = nil;
  size_t        poly_capacity = 0;

  // First-layer root rays (uploaded from host).
  id<MTLBuffer> root_d_buf  = nil;
  id<MTLBuffer> root_p_buf  = nil;
  id<MTLBuffer> root_w_buf  = nil;
  id<MTLBuffer> root_tf_buf = nil;
  // Per-ray crystal->world rotation (9 floats/ray, row-major, == Rotation::mat_).
  // The kernel applies mat*exit_dir before projection to return exit rays from
  // crystal-local to world space (invariant 6 / DESIGN D2). Sized in lock-step
  // with root_d_buf by EnsureRootBuffers.
  id<MTLBuffer> root_rot_buf = nil;
  size_t        root_capacity = 0;

  // [TEST-ONLY] scrum-328.2 Step 1 attempt-count sibling buffer. ALWAYS bound
  // to gen_root_kernel at buffer(12) (Metal validation forbids nil bindings);
  // the paired `attempts_ctrl.x` scalar tells the kernel whether to actually
  // write. Grown lazily by EnableGenAttemptCountForTest; a dummy 4-byte
  // allocation is created on first bind so production dispatches see a valid
  // buffer without the test path being armed. `attempts_ci_start_` is the
  // per-ci write offset (mirrors the RNG probe's ci_start).
  id<MTLBuffer> lat_attempts_buf_ = nil;
  size_t        lat_attempts_cap_ = 0;
  size_t        lat_attempts_ci_start_ = 0;
  bool          attempts_enabled_ = false;
  // code-review round 1 Major#2: the EFFECTIVE per-ci write offset for the
  // NEXT EncodeGenRoot call (= lat_attempts_ci_start_ + running per-ci offset
  // accumulated by TraceLayer's ci-loop). Without this, every ci in a multi-
  // crystal-instance layer would write to the same [lat_attempts_ci_start_,
  // lat_attempts_ci_start_ + ci_n) window and silently overwrite the
  // previous ci's data. Set by GenerateFirstLayerRootsForCi, consumed by
  // EncodeGenRoot (mirrors the pending_gen_params_ stash/consume pattern).
  size_t        pending_attempts_ci_start_ = 0;

  // Triangle-level geometry for device-resident root-gen (task-260.2). Uploaded
  // by UploadCrystal alongside polygon-level data; sized lazily by
  // EnsureTriBuffers. The gen kernel reads these to perform area×facing
  // weighted triangle sampling + entry-point sampling and to map the chosen
  // triangle back to its polygon face (mirrors PolygonFaceOfTri in
  // simulator.cpp). face_seq_cap_-style stride is N/A (these are per-triangle).
  id<MTLBuffer> tri_vtx_buf_     = nil;  // N_tri × 9 float (3 vtx × 3 coords)
  id<MTLBuffer> tri_norm_buf_    = nil;  // N_tri × 3 float
  id<MTLBuffer> tri_area_buf_    = nil;  // N_tri × 1 float
  id<MTLBuffer> tri_to_poly_buf_ = nil;  // N_tri × 1 uint16 (kInvalidId on miss)
  size_t        tri_buf_capacity_ = 0;

  // Unified area-measure inverse-CDF latitude LUT (330.2). Three fixed-size
  // (LatLut::kNodes float) shared buffers rebuilt per-ci by UploadLatLut when the
  // orientation distribution routes to kLatPathLutInverseCdf. Metal forbids nil
  // bindings, so the buffers are always allocated (EnsureLatLutBuffers) and bound
  // at indices 14/15/16 in gen_root/transit_root; their CONTENTS are only read by
  // the shader when gp.lat_path == kLatPathLutInverseCdf (gp.lat_lut_n > 0).
  id<MTLBuffer> lat_lut_theta_buf_ = nil;  // LatLut::kNodes float (colatitude nodes)
  id<MTLBuffer> lat_lut_cdf_buf_   = nil;  // LatLut::kNodes float (CDF)
  id<MTLBuffer> lat_lut_flip_buf_  = nil;  // LatLut::kNodes float (flip prob)

  // Continuation ping-pong (indexed by ms_idx & 1).
  // scrum-267 task-fused-emit-gate Step 4b: the parallel cont_crystal_id /
  // cont_face_seq_* buffers (formerly slots 24-26) are removed — the emit gate
  // now applies filter+prob on-device, so the host hop reads only direction +
  // weight + crystal-rot.
  // scrum-268.8 (DR-3): cont_p / cont_tf retired. trace kernel slots 10 / 12
  // now bind the wavelength pool / per-ray root wavelength index; the next
  // layer's transit_root_kernel resamples entry point + face from device
  // geometry instead of reading the dead cont_p / cont_tf writes.
  id<MTLBuffer> cont_d[2]  = { nil, nil };
  id<MTLBuffer> cont_w[2]  = { nil, nil };
  size_t        cont_capacity[2] = { 0, 0 };
  size_t        cont_counts[2]   = { 0, 0 };
  size_t        out_cap = 0;

  id<MTLBuffer> counter_buf  = nil;
  id<MTLBuffer> rec_sink_buf = nil;
  size_t        rec_sink_capacity = 0;
  size_t        max_produced = 0;

  // Exit-ray statistics buffers (parity harness). Both 4-byte atomic scalars
  // populated by the kernel in both ms_mode branches and read back after each
  // DispatchLayer; cached in `last_stats` for the producing TraceLayer to
  // hand off to MetalLayerHandle.
  id<MTLBuffer> exit_count_buf = nil;
  id<MTLBuffer> exit_w_sum_buf = nil;
  LayerStats    last_stats{};

  // S1 device-fused: per-session landed-weight scalar buffer (1 × float).
  // The trace kernel atomically adds in-bounds filter-pass ray weights here
  // (slot 18). ReadbackXyzAccum reads it alongside the W*H*3 XYZ image.
  // Allocated on first use; cleared at each BeginSession.
  id<MTLBuffer> landed_weight_buf_ = nil;

  // Exit-record buffers — retained as nil members for compile compatibility
  // after S1 device-fused removed their allocations and kernel bindings.
  // EnsureExitBuffers is a no-op; the kernel no longer writes these slots.
  id<MTLBuffer> exit_ray_d_buf = nil;
  id<MTLBuffer> exit_ray_w_buf = nil;
  id<MTLBuffer> exit_slot_buf  = nil;
  size_t        exit_ray_capacity = 0;
  id<MTLBuffer> exit_crystal_id_buf    = nil;
  id<MTLBuffer> exit_face_seq_len_buf  = nil;
  id<MTLBuffer> exit_face_seq_data_buf = nil;
  id<MTLBuffer> exit_ms_layer_buf      = nil;
  uint32_t      face_seq_cap_ = 0;

  // scrum-268.8 (DR-3): per-ray wavelength pool + per-ray wl_idx side-cars.
  //   * wl_pool_buf_: M × WlEntry (20B/entry), allocated once at pool-size resolve
  //     and re-populated per ci dispatch (crystal n_idx depends on ci).
  //   * root_wl_idx_buf_: max_rays × uint32, single (root buffers are NOT
  //     ping-pong; gen_root + transit_root both target the same root_*_buf set).
  //   * cont_wl_idx_buf_[2]: out_cap × uint32, ping-pong with cont_d/cont_w.
  // (exit_wl_idx_buf_ removed in S1 device-fused: exit records no longer materialised)
  // The buffers grow with the same capacity tracking as their host siblings.
  id<MTLBuffer> wl_pool_buf_        = nil;
  id<MTLBuffer> root_wl_idx_buf_    = nil;
  id<MTLBuffer> cont_wl_idx_buf_[2] = { nil, nil };
  uint32_t      wl_pool_size_       = 0u;

  // task-331.5 (raypath-color foundation): per-ray uint64 component mask —
  // structural sibling of the wl_idx side-cars above.
  //   * root_component_buf_: max_rays × uint64, single (root buffers not
  //     ping-pong). Layer 0 memset to 0; layer≥1 written by transit_root_kernel.
  //   * cont_component_buf_[2]: out_cap × uint64, ping-pong with cont_d/cont_w/
  //     cont_wl_idx — MUST move in lockstep through the shuffle gather + the
  //     Recombine handle swap (LANDMINE, mirror CPU RayBuffer::SwapRay).
  //   * gate_component_bits_buf_: Design-2 color-region summand → color-bit map
  //     (task-358.3 removed the leading Fork-C physical-bits region — the buffer
  //     now holds ONLY the color-region bytes, starting at offset 0). Built in
  //     EnsureFilterBuffers alongside filter_desc_buf_'s color region.
  //   * exit_comp_* : per-batch capture ring for emitted rays (mid-exit +
  //     final). Drained per layer into captured_masks_/captured_ws_ when
  //     capture_ray_mask_ is on (test-only; production leaves capture off).
  //     Buffer names retain the *_comp_* / *_component_* legacy for historical
  //     reasons (task-358.3 renamed only the flag/API layer to avoid touching
  //     the buffer-allocation and MSL binding-slot code); semantically these
  //     hold the ray's uint64 `this_mask` (== capture_ray_mask), which is now
  //     purely Design-2 colour bits.
  id<MTLBuffer> root_component_buf_    = nil;
  id<MTLBuffer> cont_component_buf_[2] = { nil, nil };
  id<MTLBuffer> gate_component_bits_buf_ = nil;
  id<MTLBuffer> exit_comp_mask_buf_ = nil;
  id<MTLBuffer> exit_comp_w_buf_    = nil;
  id<MTLBuffer> exit_comp_cnt_buf_  = nil;
  size_t        exit_comp_capacity_ = 0;
  bool          capture_ray_mask_   = false;
  std::vector<uint64_t> captured_masks_;
  std::vector<float>    captured_ws_;

  // task-358.1 (metal-color-parity): Design-2 ColorGateTable state built once
  // per BeginSession from spec.raypath_color. Empty when no raypath_color config
  // is active — has_color_groups_ then stays false and the emit gate skips the
  // color pass entirely (zero-cost hot-path, AC4).
  //   color_gate_table_       : per-(layer, crystal_id) predicate → color-bit map
  //                             (config/color_gate_table.hpp). Owned by Impl for
  //                             the session's lifetime; EnsureFilterBuffers reads
  //                             it while synthesizing per-slot symmetry groups.
  //   has_color_groups_       : true iff at least one (layer, ci) placement in
  //                             this session produced ≥1 color group. When
  //                             false, KernelParams.has_color_groups = 0 and the
  //                             MSL color pass is compile-out-equivalent.
  //   color_desc_offset_      : n_slot (offset in filter_desc_buf_ where color
  //                             descriptors start; kernel reads at
  //                             color_desc_offset_ + gate_slot * K + g). The
  //                             filter-desc buffer still carries the physical
  //                             filter descriptors up-front (used by filter
  //                             matching itself); the color region is appended
  //                             after them (see EnsureFilterBuffers).
  //   color_bits_offset_      : 0 (task-358.3 removed the leading Fork-C
  //                             physical-bits region from gate_component_bits_
  //                             buf_; the color region now begins at buffer
  //                             offset 0). Kept as a KernelParams field for
  //                             symmetry with color_desc_offset and to keep the
  //                             MSL addressing formula unchanged.
  ColorGateTable color_gate_table_{};
  bool           has_color_groups_  = false;
  uint32_t       color_desc_offset_ = 0u;
  uint32_t       color_bits_offset_ = 0u;

  // task-358.1 Step 4 (AC3 device-side Y-lane accumulation).
  //   class_table_          : runtime color-class table (parity with server-side
  //                           RenderConsumer::class_table_). Built in
  //                           BeginSession from spec.raypath_color; empty when
  //                           no color config → class_lane_buf_ stays a dummy
  //                           4-byte allocation and KernelParams.color_class_
  //                           count == 0 (MSL branch skipped end-to-end).
  //   class_count_          : cached class_table_.classes_.size() for hot-path
  //                           reads (KernelParams populate + readback sizing).
  //                           Bounded by kMaxColorClassesDevice (assert).
  //   class_lane_buf_       : atomic_float[class_count * W * H] on device.
  //                           Layout matches MSL indexing: class_lane_buf_
  //                           [c * W * H + pix]. Zeroed at
  //                           BeginSession (mirror landed_weight_buf_) and
  //                           re-zeroed by ReadbackClassLanes after each drain.
  //   class_lane_pix_capacity_ : the (class_count * pix) allocation actually
  //                           held by class_lane_buf_. Regrown lazily by
  //                           EnsureClassLaneBuf on shape / class-count change.
  ColorClassTable class_table_{};
  size_t          class_count_ = 0;
  id<MTLBuffer>   class_lane_buf_ = nil;
  size_t          class_lane_pix_capacity_ = 0;
  // BeginSession captures the spectrum mode + per-batch sentinel so
  // ResolveLayerCrystalForCi can rebuild the WlEntry pool against the active
  // crystal's refractive index each ci dispatch. illuminant_mode_=true uses
  // an M-entry uniform sampling over [380, 780]; otherwise the pool degenerates
  // to a 1-entry replication of (spec_wl, spec_weight) so the discrete-list
  // path stays per-batch (matches the legacy CPU behaviour).
  bool             illuminant_mode_  = false;
  IlluminantType   illuminant_       = IlluminantType::kD65;
  float            per_batch_wl_     = 0.0f;
  float            per_batch_weight_ = 1.0f;
  std::vector<WlEntry> wl_pool_host_;

  // Per-layer hop state (scrum-258.3 + scrum-267 task-fused-emit-gate +
  // scrum-267 task-device-resident-continuation Task 3).
  // hop_ms_prob_ feeds the device emit gate's KernelParams.ms_prob.
  // last_layer_* is still consumed by ReadbackExitRays for the final-layer
  // host filter+prob path. The sibling per-ci hop vectors and the mid-exit
  // host drain were removed in Task 3 once mid-exit emission + frame-transit
  // both ran on-device.
  float                           hop_ms_prob_[2]      = { 0.0f, 0.0f };
  std::vector<Crystal>            last_layer_crystals_;
  std::vector<AxisDistribution>   last_layer_axis_dists_;
  std::vector<FilterConfig>       last_layer_filter_configs_;
  float                           last_ms_prob_      = 0.0f;
  uint8_t                         last_ms_layer_idx_ = 0u;

  // scrum-267 task-msl-filter-match-port (Step 4): device filter MATCH state.
  // Filled once per session by EnsureFilterBuffers. Layout contract — both this
  // upload path AND the parity test build the buffers with these semantics:
  //   * filter_desc_buf_: array of DeviceFilterDesc, one entry per
  //     (ms_layer, ci) flattened as `ms_layer * max_ci + ci` (see
  //     filter_desc_strides_). Same crystal/filter pair across layers does
  //     NOT dedup — keeps lookup O(1).
  //   * getfn_offsets_buf_: uint32[n_slot + 1] prefix sum of poly_face_cnt per
  //     (ms_layer, ci); last entry = total bytes.
  //   * getfn_bytes_buf_: flat uchar stream, slot i lives in
  //     [offsets[i], offsets[i+1]); content = crystal.GetFn(poly_idx) per
  //     poly_idx (D1 layout).
  id<MTLBuffer> filter_desc_buf_    = nil;
  id<MTLBuffer> getfn_offsets_buf_  = nil;
  id<MTLBuffer> getfn_bytes_buf_    = nil;
  // Complex filter sub-spec flat buffer (267.1b). For each top-level Complex
  // desc, `sub_desc_start` indexes here and `or_clause_count` +
  // `and_term_counts[]` describe the OR/AND layout. Sub-descs are always
  // Simple (None/Raypath/EntryExit/Direction/Crystal); Complex never nests
  // Complex (host semantics: `ComplexSpec` uses `SimpleSpecCreator`).
  // Allocated even when there are no Complex filters (1-byte dummy) so the
  // kernel buffer(11) binding is never nil.
  id<MTLBuffer> complex_sub_desc_buf_ = nil;
  size_t filter_desc_count_    = 0;  // total descs uploaded (flattened slots)
  size_t filter_desc_max_ci_   = 0;  // per-layer ci stride; flattened slot
                                     // (mi, ci) → mi * max_ci + ci
  // task-device-flat-and-terms: byte offset inside `complex_sub_desc_buf_`
  // where the flat AND-term counts region begins (packed AFTER the
  // DeviceFilterDesc[] region in the SAME MTLBuffer allocation to stay under
  // Metal's 30-buffer per-stage cap). Populated by EnsureFilterBuffers along
  // with the desc/count layout; propagated to the kernel via
  // `KernelParams::and_term_counts_base_offset`.
  uint32_t and_term_counts_base_offset_ = 0u;
  // Layer dispatch helpers.
  void EnsureDevice();
  void EnsurePso();
  void EnsureImage(int w, int h);
  // task-358.1 Step 4 (AC3 device-side Y-lane accumulation): allocate/resize
  // class_lane_buf_ to class_count_ * w * h atomic_floats (or a 4-byte dummy
  // when class_count_==0 to keep the buffer(30) binding non-nil). Zeroes the
  // freshly allocated buffer. Idempotent: no-op when the requested capacity
  // matches the current allocation and shape.
  void EnsureClassLaneBuf(int w, int h);
  void EnsurePolyBuffers(size_t poly_cnt);
  void EnsureRootBuffers(size_t n);
  void EnsureTriBuffers(size_t tri_cnt);
  void EnsureContBuffer(int slot);
  void EnsureComponentCaptureBuffers(size_t cap);  // task-331.5 (test-only ring)
  void EnsureRecSink(size_t n);
  // EnsureExitBuffers removed in S1 device-fused (exit records eliminated)
  void EnsureFilterBuffers(const SessionSpec& session_spec);  // scrum-267.1
  void EnsureWlPoolBuffer();  // scrum-268.8 (DR-3) per-ray wavelength pool
  void UploadCrystal(const Crystal& crystal);
  // 330.2 S3b: lazily allocate the three fixed-size (LatLut::kNodes float) shared
  // LUT buffers (always bound, so they must always exist).
  void EnsureLatLutBuffers();
  // 330.2 S3b: rebuild + upload the latitude LUT for the given axis distribution
  // when it routes to kLatPathLutInverseCdf (per-ci cadence; covers gen + transit).
  void UploadLatLut(const AxisDistribution& axis);
  void ResolveLayerCrystalForCi(const ScatteringSetting& setting, bool use_host,
                                const HostRayBatch& host_batch);
  size_t GenerateFirstLayerRootsForCi(const ScatteringSetting& setting,
                                      size_t ci, size_t crystal_ray_num,
                                      bool can_use_device_gen,
                                      size_t attempts_ci_off);
  // task-267.4 (continuation-validation) golden-ray hook: test-only host-ray
  // injection path. Activated when HostRayBatch::d / p / w / tf are non-null
  // at first MS, ci=0 (TraceLayer guards the branch). Bypasses RNG-based
  // root generation; root_*_buf are filled directly from the caller-supplied
  // arrays so a GPU-side trace pass runs over deterministic, analytically
  // chosen rays. crystal_rot_ is set to identity (rays are interpreted as
  // crystal-local + world-aligned). Returns the number of injected rays.
  // Production callers (Simulator) leave host.d == nullptr — this path stays
  // dormant.
  size_t InjectHostRoots(const HostRayBatch& host);
  GenRootKernelParams BuildGenRootParams(const ScatteringSetting& setting,
                                          size_t crystal_ray_num) const;
  void EncodeGenRoot(id<MTLCommandBuffer> cb, const GenRootKernelParams& gp);
  // scrum-267 task-device-resident-continuation: build the transit_root_kernel
  // params (re-uses BuildGenRootParams' orientation+geometry fields and only
  // overrides PCG seed / base; transit_seed nonce isolates the stream from
  // gen_root and the emit gate). Used by TraceLayer's non-first_ms branch.
  // task-gpu-rng-ray-index-uint64: `ray_base` widened to size_t so the full
  // 64-bit host counter reaches the device via SplitPcgRayBase (populating
  // gp.gen_ray_base + gp.gen_ray_base_hi in a single site).
  GenRootKernelParams BuildTransitRootParams(const ScatteringSetting& setting,
                                              size_t ci_n,
                                              uint32_t ms_layer_idx,
                                              uint32_t ci,
                                              size_t ray_base) const;
  // Encodes a transit_root_kernel compute pass that reads the cont_d/cont_w
  // slice [ci_start, ci_start + gp.num_rays) of in_slot and writes the full
  // root_*_buf in lock-step with the trace kernel's input layout.
  void EncodeTransitRoot(id<MTLCommandBuffer> cb, const GenRootKernelParams& gp,
                         int in_slot, size_t ci_start);
  // Encodes a shuffle_cont_kernel pass: gather-reads the full cont[in_slot]
  // slice (n entries) and writes the Feistel permutation into cont[out_slot].
  // The caller (Recombine) swaps the slot handles afterwards so cont[in_slot]
  // ends up holding the shuffled data the next layer consumes.
  void EncodeShuffleCont(id<MTLCommandBuffer> cb, int in_slot, int out_slot,
                         uint32_t n, uint32_t seed);
  // DispatchLayer encodes the trace pass (and any pending gen_root) into either
  // a freshly-created command buffer or `existing_cb` (used by the non-first
  // MS path to share the CB with a preceding EncodeTransitRoot), commits the
  // CB without waiting, and parks it in `pending_cb_` along with `out_slot`.
  // The caller MUST invoke WaitAndReadbackLayer() before reading any host-
  // visible output (cont_counts, last_stats, exit_slot_buf).
  void DispatchLayer(size_t num_rays,
                     id<MTLBuffer> r_d, id<MTLBuffer> r_p,
                     id<MTLBuffer> r_w, id<MTLBuffer> r_tf,
                     uint32_t ms_mode, int out_slot,
                     uint32_t counter_init,
                     uint32_t crystal_id, uint32_t ms_layer_idx,
                     id<MTLCommandBuffer> existing_cb = nil);
  // Waits on `pending_cb_` (if any), validates status, reads back the
  // continuation counter into cont_counts[pending_out_slot_], accumulates
  // last_stats from exit_count_buf/exit_w_sum_buf, and clears pending_cb_.
  // No-op when pending_cb_ is nil (safe to call at ci-loop tail).
  void WaitAndReadbackLayer();
  void Reset();
};

void MetalTraceBackend::Impl::EnsureDevice() {
  if (device != nil) {
    return;
  }
  // task-metal-gui-commit-backpressure O2: pull device/queue from the process
  // cache. Objective-C strong-reference assignment retains the cached objects;
  // ~Impl only releases this instance's references, cache retains its own.
  const MetalDeviceCache& c = GetOrBuildMetalDeviceCache(logger_);
  device = c.device;
  queue  = c.queue;
}

void MetalTraceBackend::Impl::EnsurePso() {
  if (pso != nil) {
    return;
  }
  // task-metal-gui-commit-backpressure O2: PSOs come from the process cache
  // (same MetalDeviceCache built once per process). Cache build reproduces the
  // original EnsurePso logic (LoadMetalLibrary + 4 entry-point lookups +
  // newComputePipelineStateWithFunction) so failure modes / log text / exception
  // type are unchanged; only the "when" is different (first BeginSession in the
  // process, not every BeginSession).
  const MetalDeviceCache& c = GetOrBuildMetalDeviceCache(logger_);
  pso               = c.pso;
  gen_root_pso_     = c.gen_root_pso;
  transit_root_pso_ = c.transit_root_pso;
  shuffle_pso_      = c.shuffle_pso;
  // Per-Impl observability marker (honest wording, task-364 code review): this
  // Impl obtained its PSOs from the process-level MetalDeviceCache — it did NOT
  // load anything, so the text says "using cached" (the actual load happens once
  // per process inside LoadMetalLibrary, which keeps emitting "loaded embedded
  // metallib"). The AC2 sentinel
  // (test/regression-sentinel/test_metallib_no_source_compile.py) accepts BOTH
  // phrasings: either one positively confirms the embedded-metallib route is in
  // effect (vs the source-compile fallback), which is the invariant it guards.
  // Emitted once per Impl because this branch is gated by `pso == nil`; needed so
  // the marker appears in every session's captured log window even when an earlier
  // session in the same process already built the cache (so LoadMetalLibrary's
  // one-time "loaded" line fell outside this window). Count is hardcoded to 4 to
  // mirror _EXPECTED_FUNCTION_COUNT (trace_layer / gen_root / transit_root /
  // shuffle_cont) — bumping requires updating both this literal and the sentinel
  // test in lock-step.
  ILOG_INFO(EffectiveLogger(logger_),
            "MetalTraceBackend: using cached embedded metallib ({} functions)", 4);
}

void MetalTraceBackend::Impl::EnsureImage(int w, int h) {
  size_t pix = static_cast<size_t>(w) * static_cast<size_t>(h);
  // Grow/shrink the byte allocation only when the pixel COUNT changes.
  if (pix != xyz_pix_capacity) {
    xyz_image = [device newBufferWithLength:pix * 3 * sizeof(float)
                                    options:MTLResourceStorageModeShared];
    assert(xyz_image != nil);
    xyz_pix_capacity = pix;
  }
  // scrum-312 third clock: reset on any SHAPE change (w or h), not just pixel
  // count — a same-area shape swap (e.g. 512x1024 -> 1024x512) reuses the buffer
  // but is a fresh accumulation region. Gate on (w,h) so alloc_xyz_w_/h_ track the
  // current shape (else the between-session drain's dim check would false-throw,
  // review-Major-1) and BOTH twin accumulators reset together (else landed_weight
  // would mix old/new-shape rays while xyz_image reset, review-Major-2). A shape
  // change always rides a generation change, whose flush already drained the prior
  // window, so re-zeroing here is correct. Steady state: BeginSession does NOT
  // clear these — they persist across batches; the drain's post-read reset is the
  // per-window reset.
  if (w != alloc_xyz_w_ || h != alloc_xyz_h_) {
    // Reset BOTH twin accumulators together. landed_weight_buf_ MUST already be
    // allocated (BeginSession allocates it before calling EnsureImage) — assert
    // the invariant loudly rather than silently best-effort, so a future caller
    // that forgets the ordering can't silently reintroduce the round-1 Major-2
    // (xyz reset while landed keeps stale rays).
    assert(landed_weight_buf_ != nil && "EnsureImage: landed_weight_buf_ must be allocated before reset");
    std::memset([xyz_image contents], 0, pix * 3 * sizeof(float));
    *static_cast<float*>([landed_weight_buf_ contents]) = 0.0f;
    alloc_xyz_w_ = w;
    alloc_xyz_h_ = h;
  }
}

void MetalTraceBackend::Impl::EnsureClassLaneBuf(int w, int h) {
  // class_lane_pix_capacity_ is the total float capacity currently held
  // (class_count * W * H, or 1 for the dummy branch). Regrown only when the
  // requested layout would need more floats than the current allocation.
  const size_t pix = static_cast<size_t>(w) * static_cast<size_t>(h);
  const size_t needed_elems = (class_count_ == 0) ? 1u : (class_count_ * pix);
  // explore-359 FIX: zero ONLY on (re)allocation — NOT every BeginSession.
  // class_lane_buf_ is a scrum-312 third-clock PERSISTENT accumulator (twin of
  // xyz_image): it accumulates across batches within a drain window and is
  // reset per-window by ReadbackClassLanes' post-read memset (mm:3122).
  // The old code memset'd every BeginSession — but BeginSession runs PER BATCH,
  // so with ray_num > LUMICE_DISPATCH_RAY_NUM (multiple batches per drain) every
  // batch wiped the prior batches' lane accumulation, leaving only the LAST
  // batch. xyz_image did NOT have this bug (EnsureImage only zeroes on a shape
  // change, which rides a generation flush/drain). Net effect: device Y-lanes
  // delivered ~1/N_batches of the energy → GPU-color composites 30-50× sparser
  // than CPU while full-spectrum matched. Mirror EnsureImage: zero on alloc,
  // let the drain own the per-window reset.
  if (class_lane_buf_ == nil || needed_elems > class_lane_pix_capacity_) {
    class_lane_buf_ = [device newBufferWithLength:needed_elems * sizeof(float)
                                          options:MTLResourceStorageModeShared];
    assert(class_lane_buf_ != nil);
    class_lane_pix_capacity_ = needed_elems;
    std::memset([class_lane_buf_ contents], 0, needed_elems * sizeof(float));
  }
}

void MetalTraceBackend::Impl::EnsurePolyBuffers(size_t poly_cnt) {
  if (poly_cnt <= poly_capacity) {
    return;
  }
  poly_capacity = poly_cnt;
  poly_n_buf  = [device newBufferWithLength:poly_cnt * 3 * sizeof(float)
                                    options:MTLResourceStorageModeShared];
  assert(poly_n_buf != nil);
  poly_d_buf  = [device newBufferWithLength:poly_cnt * sizeof(float)
                                    options:MTLResourceStorageModeShared];
  assert(poly_d_buf != nil);
  centroid_buf = [device newBufferWithLength:poly_cnt * 3 * sizeof(float)
                                     options:MTLResourceStorageModeShared];
  assert(centroid_buf != nil);
}

void MetalTraceBackend::Impl::EnsureRootBuffers(size_t n) {
  if (n <= root_capacity) {
    return;
  }
  root_capacity = n;
  root_d_buf  = [device newBufferWithLength:n * 3 * sizeof(float)
                                    options:MTLResourceStorageModeShared];
  assert(root_d_buf != nil);
  root_p_buf  = [device newBufferWithLength:n * 3 * sizeof(float)
                                    options:MTLResourceStorageModeShared];
  assert(root_p_buf != nil);
  root_w_buf  = [device newBufferWithLength:n * sizeof(float)
                                    options:MTLResourceStorageModeShared];
  assert(root_w_buf != nil);
  root_tf_buf = [device newBufferWithLength:n * sizeof(uint16_t)
                                    options:MTLResourceStorageModeShared];
  assert(root_tf_buf != nil);
  root_rot_buf = [device newBufferWithLength:n * 9 * sizeof(float)
                                    options:MTLResourceStorageModeShared];
  assert(root_rot_buf != nil);
  // scrum-268.8 (DR-3): per-ray wavelength index. Lock-stepped with root_d
  // so transit_root_kernel (root-buf writer for non-first MS) and the trace
  // kernel (root-buf reader) see a side-car of the same length.
  root_wl_idx_buf_ = [device newBufferWithLength:n * sizeof(uint32_t)
                                        options:MTLResourceStorageModeShared];
  assert(root_wl_idx_buf_ != nil);
  // task-331.5: per-ray component mask carried into the trace kernel. Single
  // (not ping-pong), lock-stepped with root_d. Layer 0 is memset to 0 by
  // TraceLayer; layer≥1 is overwritten by transit_root_kernel.
  root_component_buf_ = [device newBufferWithLength:n * sizeof(uint64_t)
                                           options:MTLResourceStorageModeShared];
  assert(root_component_buf_ != nil);
}

void MetalTraceBackend::Impl::EnsureTriBuffers(size_t tri_cnt) {
  // task-260.5 Step 3: device-gen requires at least one triangle for the
  // area×facing categorical sampler. A zero-count crystal would underflow
  // categorical_sample in gen_root_kernel (n=0 → returns 0xffffffff → OOB
  // index). Production configs always have tri_cnt > 0; this assert catches
  // a future scene/config bug at the layer-prep stage rather than as a GPU
  // hang.
  assert(tri_cnt > 0u && "EnsureTriBuffers: tri_cnt == 0 (gen_root_kernel cannot sample)");
  if (tri_cnt <= tri_buf_capacity_) {
    return;
  }
  tri_buf_capacity_ = tri_cnt;
  tri_vtx_buf_ = [device newBufferWithLength:tri_cnt * 9 * sizeof(float)
                                    options:MTLResourceStorageModeShared];
  assert(tri_vtx_buf_ != nil);
  tri_norm_buf_ = [device newBufferWithLength:tri_cnt * 3 * sizeof(float)
                                     options:MTLResourceStorageModeShared];
  assert(tri_norm_buf_ != nil);
  tri_area_buf_ = [device newBufferWithLength:tri_cnt * sizeof(float)
                                     options:MTLResourceStorageModeShared];
  assert(tri_area_buf_ != nil);
  tri_to_poly_buf_ = [device newBufferWithLength:tri_cnt * sizeof(uint16_t)
                                        options:MTLResourceStorageModeShared];
  assert(tri_to_poly_buf_ != nil);
}

void MetalTraceBackend::Impl::EnsureContBuffer(int slot) {
  if (out_cap <= cont_capacity[slot]) {
    return;
  }
  cont_capacity[slot] = out_cap;
  cont_d[slot]  = [device newBufferWithLength:out_cap * 3 * sizeof(float)
                                      options:MTLResourceStorageModeShared];
  assert(cont_d[slot] != nil);
  cont_w[slot]  = [device newBufferWithLength:out_cap * sizeof(float)
                                      options:MTLResourceStorageModeShared];
  assert(cont_w[slot] != nil);
  // scrum-268.8 (DR-3): per-ray wavelength index for continuation rays.
  // Lock-stepped with cont_d/cont_w so the emit gate's cont_wl_idx write and
  // transit_root_kernel's cont_wl_idx read agree on capacity.
  cont_wl_idx_buf_[slot] = [device newBufferWithLength:out_cap * sizeof(uint32_t)
                                              options:MTLResourceStorageModeShared];
  assert(cont_wl_idx_buf_[slot] != nil);
  // task-331.5: per-ray component mask for continuation rays. Lock-stepped with
  // cont_d/cont_w/cont_wl_idx so the emit gate write, shuffle gather, and
  // Recombine handle swap all agree on capacity.
  cont_component_buf_[slot] = [device newBufferWithLength:out_cap * sizeof(uint64_t)
                                                 options:MTLResourceStorageModeShared];
  assert(cont_component_buf_[slot] != nil);
}

void MetalTraceBackend::Impl::EnsureComponentCaptureBuffers(size_t cap) {
  // task-331.5: grow-only per-batch capture ring for emitted (mid-exit + final)
  // rays. Only allocated when capture_ray_mask_ is on; production never calls
  // this (the kernel's capture branch is gated by KernelParams.capture_ray_mask).
  // The atomic counter buffer is allocated once (1 × uint32).
  if (exit_comp_cnt_buf_ == nil) {
    exit_comp_cnt_buf_ = [device newBufferWithLength:sizeof(uint32_t)
                                            options:MTLResourceStorageModeShared];
    assert(exit_comp_cnt_buf_ != nil);
  }
  if (cap <= exit_comp_capacity_ && exit_comp_mask_buf_ != nil) {
    return;
  }
  exit_comp_capacity_ = cap;
  exit_comp_mask_buf_ = [device newBufferWithLength:cap * sizeof(uint64_t)
                                           options:MTLResourceStorageModeShared];
  assert(exit_comp_mask_buf_ != nil);
  exit_comp_w_buf_ = [device newBufferWithLength:cap * sizeof(float)
                                        options:MTLResourceStorageModeShared];
  assert(exit_comp_w_buf_ != nil);
}

void MetalTraceBackend::Impl::EnsureRecSink(size_t n) {
  if (n <= rec_sink_capacity) {
    return;
  }
  rec_sink_capacity = n;
  rec_sink_buf = [device newBufferWithLength:n * sizeof(float)
                                     options:MTLResourceStorageModeShared];
  assert(rec_sink_buf != nil);
}

// EnsureExitBuffers was removed in S1 device-fused: exit records are no longer
// materialised. The function declaration is preserved in comments for history.
// (formerly: Exit seam scrum-258.1 grow-only buffer-egress storage + atomic slot)

// scrum-268.8 (DR-3): allocate the host-uploaded wavelength pool exactly once
// per backend (pool size is invariant across sessions and ci dispatches; only
// the pool contents change per ci). Idempotent — safe to call from every
// BeginSession.
void MetalTraceBackend::Impl::EnsureWlPoolBuffer() {
  if (wl_pool_size_ == 0u) {
    wl_pool_size_ = ResolveWlPoolSize(EffectiveLogger(logger_));
    assert(wl_pool_size_ > 0u && wl_pool_size_ <= kWlPoolSizeMax &&
           "ResolveWlPoolSize returned out-of-range");
  }
  if (wl_pool_buf_ != nil) {
    return;
  }
  wl_pool_buf_ = [device newBufferWithLength:wl_pool_size_ * sizeof(WlEntry)
                                    options:MTLResourceStorageModeShared];
  assert(wl_pool_buf_ != nil);
}

// scrum-267 task-msl-filter-match-port (Step 4): upload per-session filter
// descriptors + GetFn tables for the future fused emit gate (sub-task 2).
// Layout: one DeviceFilterDesc per (ms_layer, ci) slot, flat-indexed as
// `ms_layer * max_ci + ci` (max_ci = max setting_.size() across layers); one
// GetFn byte stripe per slot with a uint prefix-sum offsets array. All
// hexagonal crystals share `poly_face_cnt_ == 8`, so a prototype crystal made
// with a fixed seed is enough to derive GetFn — current Metal backend supports
// hex prism only (BeginSession asserts already enforce that elsewhere). Per
// plan §3 D1 + Step 4 + Round-2 Minor-3 refinement.
void MetalTraceBackend::Impl::EnsureFilterBuffers(const SessionSpec& session_spec) {
  filter_desc_count_ = 0;
  filter_desc_max_ci_ = 0;
  filter_desc_buf_ = nil;
  getfn_offsets_buf_ = nil;
  getfn_bytes_buf_ = nil;
  complex_sub_desc_buf_ = nil;
  and_term_counts_base_offset_ = 0u;
  // task-358.1: reset color state up-front; the descriptor append below turns
  // it back on when at least one placement produces groups.
  has_color_groups_  = false;
  color_desc_offset_ = 0u;
  color_bits_offset_ = 0u;
  // scrum-267 task-fused-emit-gate Step 3a (R5 fix): the trace kernel's emit
  // gate unconditionally binds filter_desc_buf_ / getfn_offsets_buf_ /
  // getfn_bytes_buf_ / complex_sub_desc_buf_ at buffer slots 24-27 (plan §3
  // originally reserved 27-30 but Metal's per-stage buffer-index ceiling is
  // 30 — see DECISION in progress.md). Metal disallows nil-buffer bindings,
  // so each early-return path must still allocate a 1-byte dummy. The kernel
  // never reads through them in the
  // no-filter case (DeviceFilterCheck on a None desc returns true and the gate
  // proceeds, but on the ms_mode==0 path the gate is never entered; the
  // ms_mode==1 path only fires when there are MS layers, which co-occurs with
  // EnsureFilterBuffers having a real desc array).
  auto alloc_filter_dummies = [&]() {
    filter_desc_buf_ = [device newBufferWithLength:1
                                          options:MTLResourceStorageModeShared];
    assert(filter_desc_buf_ != nil);
    getfn_offsets_buf_ = [device newBufferWithLength:1
                                            options:MTLResourceStorageModeShared];
    assert(getfn_offsets_buf_ != nil);
    getfn_bytes_buf_ = [device newBufferWithLength:1
                                          options:MTLResourceStorageModeShared];
    assert(getfn_bytes_buf_ != nil);
    complex_sub_desc_buf_ = [device newBufferWithLength:1
                                               options:MTLResourceStorageModeShared];
    assert(complex_sub_desc_buf_ != nil);
    // task-331.5: kernel binds gate_component_bits unconditionally (slot 21);
    // 1-byte dummy for the no-filter path (never read when capture is off).
    gate_component_bits_buf_ = [device newBufferWithLength:1
                                                  options:MTLResourceStorageModeShared];
    assert(gate_component_bits_buf_ != nil);
  };
  gate_component_bits_buf_ = nil;
  if (session_spec.scene == nullptr) {
    alloc_filter_dummies();
    return;
  }
  size_t n_layers = session_spec.scene->ms_.size();
  if (n_layers == 0) {
    alloc_filter_dummies();
    return;
  }
  size_t max_ci = 0;
  for (const auto& ms : session_spec.scene->ms_) {
    if (ms.setting_.size() > max_ci) { max_ci = ms.setting_.size(); }
  }
  if (max_ci == 0) {
    alloc_filter_dummies();
    return;
  }
  filter_desc_max_ci_ = max_ci;
  size_t n_slot = n_layers * max_ci;

  // Build per-slot prototype crystals via a private RNG so the session's main
  // rng stream (used for trace) is not advanced. Hex-prism GetFn(poly_idx) is
  // shape-invariant for hex faces; any sampled instance suffices for the
  // canonical_bytes + GetFn table contents (Round-2 Minor-3 acknowledgement:
  // canonical computation is fn_period=6-invariant for hex crystals).
  RandomNumberGenerator proto_rng(0xC0FEFEEDu);
  std::vector<DeviceFilterDesc> descs(n_slot);
  std::vector<std::vector<uint8_t>> per_slot_bytes(n_slot);
  // Flat sub-desc buffer for Complex filters (267.1b). Inlined collection so a
  // Complex slot's `sub_desc_start` is recorded BEFORE pushing its sub-descs —
  // avoids needing a separate (slot, ComplexFilterParam) map for a second pass.
  std::vector<DeviceFilterDesc> all_sub_descs;
  // task-device-flat-and-terms: flat AND-term counts buffer, one uint8 per
  // OR-clause across all Complex filters, packed in OR-clause order under each
  // parent desc. `and_terms_start` on the parent indexes here; two-region
  // packing into `complex_sub_desc_buf_` keeps Metal's 30-buffer per-stage cap
  // intact (single binding at atIndex:27; kernel reinterprets the tail).
  std::vector<uint8_t> and_term_counts_flat;
  for (size_t mi = 0; mi < n_layers; ++mi) {
    const auto& ms = session_spec.scene->ms_[mi];
    for (size_t ci = 0; ci < ms.setting_.size(); ++ci) {
      const auto& setting = ms.setting_[ci];
      Crystal proto = MakeCrystal(proto_rng, setting.crystal_.param_);
      size_t slot = mi * max_ci + ci;
      descs[slot] = detail::BuildDeviceFilterDesc(setting.filter_, proto, setting.crystal_.axis_);
      per_slot_bytes[slot] = detail::BuildDeviceGetFnBytes(proto);
      // Inline Complex sub-desc collection — see plan §3 D5 / Step 3 rationale.
      if (descs[slot].type == kDeviceFilterTypeComplex) {
        const auto* complex_p = std::get_if<ComplexFilterParam>(&setting.filter_.param_);
        // FillComplexDescTop is only invoked for ComplexFilterParam, so this
        // get_if must succeed (defensive against future variant additions).
        assert(complex_p != nullptr && "Complex desc type without ComplexFilterParam variant");
        descs[slot].sub_desc_start = static_cast<uint32_t>(all_sub_descs.size());
        descs[slot].and_terms_start = static_cast<uint32_t>(and_term_counts_flat.size());
        detail::BuildComplexSubDescs(*complex_p, proto, descs[slot].symmetry,
                                     descs[slot].sigma_a, descs[slot].d_applicable != 0u,
                                     all_sub_descs, and_term_counts_flat);
      }
    }
    // Empty trailing slots (ms.setting_.size() < max_ci) keep zero-init
    // DeviceFilterDesc{type=kDeviceFilterTypeNone} + empty GetFn stripe; the
    // device path treats type=None as pass-through true, so an invalid index
    // surfaces as a benign true rather than a memory fault.
  }

  // task-358.1 (metal-color-parity): APPEND color-group descriptors after the
  // physical filter descriptors, into the SAME filter_desc_buf_ + complex_sub_
  // desc_buf_ (buffer-slot budget assumption B; new bindings would exceed
  // Metal's per-stage 30-buffer ceiling). Addressing公式 for the kernel:
  //   color_slot = color_desc_offset_ + gate_slot * kColorMaxGroupsPerSlot + g
  // where color_desc_offset_ = n_slot (physical-region size). The formula is
  // TIGHTLY COUPLED to the physical-descriptor region size — if a future
  // maintainer changes the physical layout above (e.g. adds trailing padding
  // for a new descriptor field), color_desc_offset_ MUST track that change.
  // See plan §4 Step 1 Minor 1 + assumption B for the slot-budget derivation.
  //
  // Per placement (mi, ci), we build ColorSpecGroups via BuildColorSpecGroups
  // (same code path as CPU cpu_trace_backend.cpp:367-373 → SimulateOneWavelength
  // → CollectData). Each group carries:
  //   - one ComplexFilterParam (OR-of-singleton-AND-clauses over predicates
  //     sharing a P/B/D symmetry)
  //   - a parallel `bits[]` array mapping local summand idx → global color bit
  // We synthesize a `FilterConfig{ symmetry_=group.sym, action_=kFilterIn }`,
  // hand it to BuildDeviceFilterDesc / BuildComplexSubDescs (same as physical),
  // and append. Empty group slots stay zero-initialised (type=None), which the
  // MSL DeviceFilterSummandMask treats as "no summand matches" (mask=0) —
  // guaranteed benign no-op, matching the physical-region convention.
  color_desc_offset_ = static_cast<uint32_t>(n_slot);
  // task-358.3: Fork-C physical-bits region deleted; color region starts at
  // buffer offset 0 (see gate_component_bits_buf_ comment).
  color_bits_offset_ = 0u;
  const size_t color_desc_region = n_slot * kColorMaxGroupsPerSlot;
  std::vector<DeviceFilterDesc> color_descs(color_desc_region);  // zero-init
  const size_t color_bits_region = color_desc_region * kDeviceFilterMaxOrClauses;
  std::vector<uint8_t> color_bit_map(color_bits_region, ComponentTable::kNoBit);
  bool any_color_group = false;

  // Per-slot color-region build. Group formation delegated to the shared
  // authority (GroupPlacementBySymmetry, config/color_gate_table.hpp) so this
  // path and CPU's BuildColorSpecGroups (filter_spec.cpp) can never silently
  // disagree on group order (task-358.1 code-review round 2: this used to be
  // an inline copy of the CPU algorithm, kept in sync only by comment
  // discipline).
  for (size_t mi = 0; mi < n_layers; ++mi) {
    const auto& ms = session_spec.scene->ms_[mi];
    for (size_t ci = 0; ci < ms.setting_.size(); ++ci) {
      const auto& setting = ms.setting_[ci];
      ColorGatePlacement placement = ColorGatePlacementFor(
          color_gate_table_, static_cast<IdType>(mi), setting.crystal_.id_);
      if (placement.predicates_.empty()) { continue; }
      RandomNumberGenerator color_proto_rng(0xC0FEFEEDu ^
                                            static_cast<uint32_t>(mi * 65537u + ci));
      Crystal proto = MakeCrystal(color_proto_rng, setting.crystal_.param_);
      ColorPlacementGrouping grouping = GroupPlacementBySymmetry(placement);
      const std::vector<uint8_t>& group_symmetry = grouping.group_symmetry_;
      const std::vector<size_t>& group_of = grouping.group_of_;
      const size_t group_count_raw = group_symmetry.size();
      // Runtime clamp (not just the assert below, which -DNDEBUG release
      // builds compile out): color_descs/color_bit_map are sized for exactly
      // kColorMaxGroupsPerSlot groups per gate slot. A placement producing
      // more distinct symmetry groups than that budget must drop the excess
      // groups rather than write past color_descs/color_bit_map's allocation
      // (code-review round 2 Major: this was assert-only, i.e. a
      // heap-buffer-overflow write in release for any placement exceeding the
      // as-yet-unaudited-in-production kColorMaxGroupsPerSlot=4 cap).
      if (group_count_raw > kColorMaxGroupsPerSlot) {
        ILOG_ERROR(EffectiveLogger(logger_),
                   "MetalTraceBackend::EnsureFilterBuffers: color placement (layer={}, crystal_id={}) produced {} "
                   "symmetry groups, exceeding kColorMaxGroupsPerSlot={} — dropping the excess {} group(s) rather "
                   "than overflowing color_descs/color_bit_map. Raise kColorMaxGroupsPerSlot (+ MSL sibling) if this "
                   "config is intentional.",
                   mi, setting.crystal_.id_, group_count_raw, kColorMaxGroupsPerSlot,
                   group_count_raw - kColorMaxGroupsPerSlot);
      }
      const size_t group_count = std::min(group_count_raw, kColorMaxGroupsPerSlot);
      assert(group_count <= kColorMaxGroupsPerSlot &&
             "Color placement produced more symmetry groups than "
             "kColorMaxGroupsPerSlot; raise the constant + MSL sibling together");
      any_color_group = true;
      size_t gate_slot = mi * max_ci + ci;
      for (size_t gi = 0; gi < group_count; ++gi) {
        uint8_t sym = group_symmetry[gi];
        // Rebuild the ComplexFilterParam (singleton AND-clauses in placement
        // order, filtered to this group's symmetry).
        ComplexFilterParam cfp;
        std::vector<uint8_t> local_bits;
        for (size_t k = 0; k < placement.predicates_.size(); ++k) {
          if (group_of[k] != gi) continue;
          // Runtime clamp (same failure class as the group-count guard
          // above): color_bit_map reserves exactly kDeviceFilterMaxOrClauses
          // summand slots per color_slot. Stop accepting summands once full
          // instead of writing past this slot's span into the next one (or
          // past the vector end for the last slot).
          if (local_bits.size() >= kDeviceFilterMaxOrClauses) {
            ILOG_ERROR(EffectiveLogger(logger_),
                       "MetalTraceBackend::EnsureFilterBuffers: color group (layer={}, crystal_id={}, symmetry={}) "
                       "has more than kDeviceFilterMaxOrClauses={} OR-summands — dropping the remainder rather than "
                       "overflowing color_bit_map.",
                       mi, setting.crystal_.id_, static_cast<int>(sym), kDeviceFilterMaxOrClauses);
            break;
          }
          cfp.filters_.push_back({ { kInvalidId, placement.predicates_[k] } });
          local_bits.push_back(placement.bits_[k]);
        }
        // Build the top-level Complex DeviceFilterDesc with the group's
        // symmetry (routed into ReduceRaypath canonical bytes) + kFilterIn
        // action, mirroring BuildColorSpecGroups's synthesized FilterConfig.
        FilterConfig fc{};
        fc.id_ = kInvalidId;
        fc.symmetry_ = sym;
        fc.action_ = FilterConfig::kFilterIn;
        fc.param_ = FilterParam{ cfp };
        DeviceFilterDesc top = detail::BuildDeviceFilterDesc(fc, proto, setting.crystal_.axis_);
        // sub_desc_start / and_terms_start assigned BEFORE BuildComplexSubDescs
        // appends (mirrors the physical-slot loop above).
        top.sub_desc_start = static_cast<uint32_t>(all_sub_descs.size());
        top.and_terms_start = static_cast<uint32_t>(and_term_counts_flat.size());
        detail::BuildComplexSubDescs(cfp, proto, top.symmetry, top.sigma_a,
                                     top.d_applicable != 0u, all_sub_descs, and_term_counts_flat);
        // Place the descriptor at color_slot = gate_slot * K + gi (region-local
        // index; absolute filter_desc_buf_ index adds n_slot when we upload).
        size_t color_slot = gate_slot * kColorMaxGroupsPerSlot + gi;
        assert(color_slot < color_desc_region);
        color_descs[color_slot] = top;
        // Populate the color bit map at the SAME region-local stride the MSL
        // will index with: color_slot * kDeviceFilterMaxOrClauses + summand.
        assert(local_bits.size() <= kDeviceFilterMaxOrClauses &&
               "Color group has more OR-summands than kDeviceFilterMaxOrClauses");
        size_t bit_base = color_slot * kDeviceFilterMaxOrClauses;
        for (size_t s = 0; s < local_bits.size(); ++s) {
          color_bit_map[bit_base + s] = local_bits[s];
        }
      }
    }
  }
  has_color_groups_ = any_color_group;

  // Upload physical + color descs together.
  filter_desc_count_ = n_slot;
  size_t total_desc_count = n_slot + color_desc_region;
  size_t descs_bytes = total_desc_count * sizeof(DeviceFilterDesc);
  filter_desc_buf_ = [device newBufferWithLength:descs_bytes
                                          options:MTLResourceStorageModeShared];
  assert(filter_desc_buf_ != nil);
  {
    DeviceFilterDesc* dst = static_cast<DeviceFilterDesc*>([filter_desc_buf_ contents]);
    std::memcpy(dst, descs.data(), n_slot * sizeof(DeviceFilterDesc));
    std::memcpy(dst + n_slot, color_descs.data(),
                color_desc_region * sizeof(DeviceFilterDesc));
  }

  // Upload GetFn prefix-sum offsets + flat byte stream.
  std::vector<uint32_t> offsets(n_slot + 1, 0u);
  for (size_t i = 0; i < n_slot; ++i) {
    offsets[i + 1] = offsets[i] + static_cast<uint32_t>(per_slot_bytes[i].size());
  }
  size_t offsets_bytes = offsets.size() * sizeof(uint32_t);
  getfn_offsets_buf_ = [device newBufferWithLength:offsets_bytes
                                            options:MTLResourceStorageModeShared];
  assert(getfn_offsets_buf_ != nil);
  std::memcpy([getfn_offsets_buf_ contents], offsets.data(), offsets_bytes);

  size_t total_bytes = offsets.back();
  // newBufferWithLength:0 returns nil on some drivers — guard with a 1-byte
  // minimum so the buffer is always bindable (kernel reads gated by offsets).
  size_t alloc_bytes = std::max<size_t>(total_bytes, 1);
  getfn_bytes_buf_ = [device newBufferWithLength:alloc_bytes
                                          options:MTLResourceStorageModeShared];
  assert(getfn_bytes_buf_ != nil);
  if (total_bytes > 0) {
    uint8_t* dst = static_cast<uint8_t*>([getfn_bytes_buf_ contents]);
    for (size_t i = 0; i < n_slot; ++i) {
      if (!per_slot_bytes[i].empty()) {
        std::memcpy(dst + offsets[i], per_slot_bytes[i].data(), per_slot_bytes[i].size());
      }
    }
  }

  // Upload Complex sub-desc flat buffer (267.1b) + flat AND-term counts
  // (task-device-flat-and-terms) packed after it. Allocate a 1-byte dummy when
  // both are empty so the kernel's buffer(27) binding is always valid (Metal
  // disallows nil buffer binds). Kernel derives the counts pointer via
  // `reinterpret_cast<device const uchar*>(sub_desc_buf) + and_term_counts_base_offset`.
  //
  // Layout audit: `sub_desc_bytes` is finalized BEFORE `and_term_counts_flat`
  // is packed onto the tail (both vectors were populated in the same pass over
  // Complex descriptors above), so `and_term_counts_base_offset_` cannot drift
  // during the packing.
  //
  // Future extension: if a third distinct region ever needs to piggy-back on
  // this buffer, replace the hand-tracked offsets with a small region table
  // (offset + length per region) instead of adding another hand-computed
  // constant — the "handful of offsets in KernelParams" pattern does not
  // scale. See doc/gpu-porting-checklist.md for how to record shared-buffer
  // regions when propagating to CUDA / future GPU backends.
  size_t sub_desc_bytes = all_sub_descs.size() * sizeof(DeviceFilterDesc);
  size_t and_term_bytes = and_term_counts_flat.size();
  size_t sub_alloc_bytes = std::max<size_t>(sub_desc_bytes + and_term_bytes, 1);
  complex_sub_desc_buf_ = [device newBufferWithLength:sub_alloc_bytes
                                              options:MTLResourceStorageModeShared];
  assert(complex_sub_desc_buf_ != nil);
  if (sub_desc_bytes > 0) {
    std::memcpy([complex_sub_desc_buf_ contents], all_sub_descs.data(), sub_desc_bytes);
  }
  if (and_term_bytes > 0) {
    auto* base = static_cast<uint8_t*>([complex_sub_desc_buf_ contents]);
    std::memcpy(base + sub_desc_bytes, and_term_counts_flat.data(), and_term_bytes);
  }
  and_term_counts_base_offset_ = static_cast<uint32_t>(sub_desc_bytes);

  // task-358.3: upload the Design-2 color-region bit map as the sole content of
  // gate_component_bits_buf_ (the leading Fork-C physical-bits region is gone;
  // color_bits_offset_ is now 0). Kernel indexes color with:
  //   color_bit = gate_component_bits[color_bits_offset_
  //                                   + color_slot * kDeviceFilterMaxOrClauses + summand]
  // where color_slot = gate_slot * kColorMaxGroupsPerSlot + g.
  const size_t color_bits_count = color_bits_region;
  gate_component_bits_buf_ = [device newBufferWithLength:color_bits_count * sizeof(uint8_t)
                                                options:MTLResourceStorageModeShared];
  assert(gate_component_bits_buf_ != nil);
  std::memcpy([gate_component_bits_buf_ contents], color_bit_map.data(), color_bits_count);
}

void MetalTraceBackend::Impl::UploadCrystal(const Crystal& crystal) {
  size_t poly_cnt = crystal.PolygonFaceCount();
  EnsurePolyBuffers(poly_cnt);

  std::memcpy([poly_n_buf contents], crystal.GetPolygonFaceNormal(),
              poly_cnt * 3 * sizeof(float));
  std::memcpy([poly_d_buf contents], crystal.GetPolygonFaceDist(),
              poly_cnt * sizeof(float));

  // Centroid per polygon face (mean of the polygon-triangle vertices) — same
  // formula as the explore spike (metal_full.mm / metal_ms.mm).
  const int* poly_tri = crystal.GetPolygonFaceTriId();
  const float* tvtx   = crystal.GetTriangleVtx();
  auto* centroid_ptr  = static_cast<float*>([centroid_buf contents]);
  for (size_t f = 0; f < poly_cnt; f++) {
    const float* v = tvtx + static_cast<size_t>(poly_tri[f]) * 9;
    for (int k = 0; k < 3; k++) {
      centroid_ptr[f * 3 + k] = (v[0 * 3 + k] + v[1 * 3 + k] + v[2 * 3 + k]) / 3.0f;
    }
  }

  // Triangle-level geometry (task-260.2). Uploaded so the device root-gen
  // kernel can replicate InitRay_p_fid: area×facing-weighted triangle pick +
  // uniform sample inside the triangle, plus tri→polygon mapping.
  // tri_to_poly mirrors simulator.cpp::detail::PolygonFaceOfTri: argmax over
  // polygon-face normals with a sanity floor (kFaceCoplanarFloor=1e-2). A
  // first-match `dot > 1-1e-3` cannot distinguish adjacent upper-pyramid faces
  // on extreme-wedge (~≥87.4°) crystals (dot ≈ 0.9994).
  // NOTE: kFaceCoplanarFloor must match the value in crystal.cpp::BuildPolygonFaceData
  // and simulator.cpp::detail::PolygonFaceOfTri.
  size_t tri_cnt = crystal.TotalTriangles();
  EnsureTriBuffers(tri_cnt);
  std::memcpy([tri_vtx_buf_ contents], crystal.GetTriangleVtx(),
              tri_cnt * 9 * sizeof(float));
  std::memcpy([tri_norm_buf_ contents], crystal.GetTriangleNormal(),
              tri_cnt * 3 * sizeof(float));
  std::memcpy([tri_area_buf_ contents], crystal.GetTirangleArea(),
              tri_cnt * sizeof(float));
  const float* tri_norms_src = crystal.GetTriangleNormal();
  const float* poly_norms_src = crystal.GetPolygonFaceNormal();
  auto* tri_to_poly_ptr = static_cast<uint16_t*>([tri_to_poly_buf_ contents]);
  constexpr uint16_t kInvalidIdU16 = 0xffffu;
  constexpr float kFaceCoplanarFloor = 1e-2f;
  for (size_t t = 0; t < tri_cnt; t++) {
    const float* tn = tri_norms_src + t * 3;
    int best_p = -1;
    float best_dot = -1.0f;
    for (size_t p = 0; p < poly_cnt; p++) {
      const float* pn = poly_norms_src + p * 3;
      float dot = tn[0] * pn[0] + tn[1] * pn[1] + tn[2] * pn[2];
      if (dot > best_dot) {
        best_dot = dot;
        best_p = static_cast<int>(p);
      }
    }
    tri_to_poly_ptr[t] = (best_p >= 0 && best_dot >= 1.0f - kFaceCoplanarFloor)
                             ? static_cast<uint16_t>(best_p)
                             : kInvalidIdU16;
  }
}

// 330.2 S3b: allocate the three fixed-size shared LUT buffers if not yet present.
// Metal validation forbids nil bindings, so gen_root/transit_root always bind
// these at 14/15/16 even for non-LUT paths; their contents are read only when
// gp.lat_path == kLatPathLutInverseCdf. The size never varies (LatLut::kNodes).
void MetalTraceBackend::Impl::EnsureLatLutBuffers() {
  constexpr NSUInteger kLen = static_cast<NSUInteger>(LatLut::kNodes) * sizeof(float);
  if (lat_lut_theta_buf_ == nil) {
    lat_lut_theta_buf_ = [device newBufferWithLength:kLen options:MTLResourceStorageModeShared];
    assert(lat_lut_theta_buf_ != nil);
  }
  if (lat_lut_cdf_buf_ == nil) {
    lat_lut_cdf_buf_ = [device newBufferWithLength:kLen options:MTLResourceStorageModeShared];
    assert(lat_lut_cdf_buf_ != nil);
  }
  if (lat_lut_flip_buf_ == nil) {
    lat_lut_flip_buf_ = [device newBufferWithLength:kLen options:MTLResourceStorageModeShared];
    assert(lat_lut_flip_buf_ != nil);
  }
}

// 330.2 S3b: rebuild the latitude LUT for this ci's orientation distribution and
// memcpy the three arrays into the shared buffers. Only the LUT-routed families
// consume the contents; other paths leave stale buffer data (never read). Called
// at per-ci cadence from ResolveLayerCrystalForCi (covers gen AND transit).
void MetalTraceBackend::Impl::UploadLatLut(const AxisDistribution& axis) {
  EnsureLatLutBuffers();
  if (lat_path::SelectLatPath(axis).kind != lat_path::LatPathKind::kLutInverseCdf) {
    return;
  }
  // task-335: shared build-once cache (was per-ci BuildLatLut = a full 65536-sample
  // quadrature rebuilt on every crystal switch). Only the 3 KB memcpy stays per ci.
  const LatLut* lut = GetSharedLatLut(axis.latitude_dist);
  constexpr size_t kLen = static_cast<size_t>(LatLut::kNodes) * sizeof(float);
  std::memcpy([lat_lut_theta_buf_ contents], lut->theta.data(), kLen);
  std::memcpy([lat_lut_cdf_buf_ contents], lut->cdf.data(), kLen);
  std::memcpy([lat_lut_flip_buf_ contents], lut->flip_prob.data(), kLen);
}

void MetalTraceBackend::Impl::ResolveLayerCrystalForCi(const ScatteringSetting& setting,
                                                        bool use_host,
                                                        const HostRayBatch& host_batch) {
  if (use_host && host_batch.crystal != nullptr) {
    current_crystal = *host_batch.crystal;
    current_n_idx = host_batch.refractive_index;
  } else {
    current_crystal = MakeCrystal(rng, setting.crystal_.param_);
    // scrum-268.8 (DR-3): current_n_idx is dead state now — the trace kernel
    // reads per-ray n from wl_pool[wl_idx], not from this field (KernelParams
    // .n_idx was removed). After Step 9 the illuminant path passes wl=0, so
    // guard the Sellmeier call against the zero sentinel to avoid evaluating
    // GetRefractiveIndex out of its valid range for a value nothing consumes.
    current_n_idx =
        (spec.wl.wl_ > 1.0f) ? current_crystal.GetRefractiveIndex(spec.wl.wl_) : 0.0f;
  }
  have_crystal = true;
  UploadCrystal(current_crystal);
  // 330.2 S3b: rebuild the latitude LUT at the same per-ci cadence — it depends
  // only on the axis distribution and is shared by the gen and transit passes of
  // this ci. EnsureLatLutBuffers (inside) keeps the buffers non-nil for binding.
  UploadLatLut(setting.crystal_.axis_);
  // scrum-268.8 (DR-3): pool upload moved to BeginSession — Crystal::
  // GetRefractiveIndex delegates to a global IceRefractiveIndex::Get so the
  // refractive index per wavelength is identical across every crystal shape
  // in a session. Recomputing per ci was a measured ~21× throughput hit on
  // heavy multi-MS configs (M=64 lookups × N_ci × N_batch CPU work that the
  // GPU could not hide). The session-level cache is invalidated implicitly
  // by EndSession's Reset().
}

size_t MetalTraceBackend::Impl::GenerateFirstLayerRootsForCi(const ScatteringSetting& setting,
                                                              size_t ci, size_t crystal_ray_num,
                                                              bool can_use_device_gen,
                                                              size_t attempts_ci_off) {
  if (can_use_device_gen) {
    // code-review round 1 Major#2: stash the EFFECTIVE per-ci attempts offset
    // (base + running per-ci accumulation) for EncodeGenRoot to consume,
    // mirroring pending_gen_params_'s stash/consume within the same ci
    // iteration.
    pending_attempts_ci_start_ = lat_attempts_ci_start_ + attempts_ci_off;
    // Device root-gen path (task-260.2). Replicates InitRayFirstMs on the GPU
    // using a counter-based PCG stream keyed by (gen_seed_, gen_ray_base+tid).
    // root_ray_count accumulates across dispatches to keep the global index
    // monotone across batches of the same session.
    GenRootKernelParams gp = BuildGenRootParams(setting, crystal_ray_num);
    gp.gen_seed     = gen_seed_;
    // task-gpu-rng-ray-index-uint64: SplitPcgRayBase carries the full 64-bit
    // running ray count into the device via lo/hi halves; the kernel mixes hi
    // into each ray's PCG seed so sessions beyond 2^32 rays no longer collapse
    // on wrap (replaces the pre-fix NarrowPcgRayBase 4.29e9 cap).
    auto split = SplitPcgRayBase(root_ray_count);
    gp.gen_ray_base    = split.lo;
    gp.gen_ray_base_hi = split.hi;
    gp.num_rays     = static_cast<uint32_t>(crystal_ray_num);
    // task-264 gen+trace fusion: stash params for DispatchLayer to encode into
    // the same command buffer as the trace pass. DispatchLayer is guaranteed
    // to be called for every ci that takes this branch (the ci_n == 0 early
    // continue in TraceLayer happens before GenerateFirstLayerRootsForCi, and
    // the device-gen path always returns crystal_ray_num > 0), so the stash
    // is always consumed within the same ci iteration.
    pending_gen_params_ = gp;
    root_ray_count += crystal_ray_num;
    return crystal_ray_num;
  }

  const AxisDistribution& crystal_axis = setting.crystal_.axis_;

  RayBuffer workspace[2]{};
  workspace[0].Reset(crystal_ray_num);
  workspace[1].Reset(crystal_ray_num);
  RayBuffer all_data = AllocateAllData(*spec.scene, crystal_ray_num);

  InitRayFirstMs(rng, spec.scene->light_source_.param_, spec.wl, crystal_ray_num,
                 current_crystal, /*crystal_id=*/ci, crystal_axis,
                 workspace, all_data);

  // EnsureRootBuffers is called by TraceLayer at the top of each layer with
  // total_ray_num so per-ci root buffers are guaranteed >= crystal_ray_num.
  size_t n = workspace[0].size_;
  auto* d_ptr   = static_cast<float*>([root_d_buf contents]);
  auto* p_ptr   = static_cast<float*>([root_p_buf contents]);
  auto* w_ptr   = static_cast<float*>([root_w_buf contents]);
  auto* tf_ptr  = static_cast<uint16_t*>([root_tf_buf contents]);
  auto* rot_ptr = static_cast<float*>([root_rot_buf contents]);

  for (size_t i = 0; i < n; i++) {
    const RaySeg& r = workspace[0][i];
    d_ptr[i * 3 + 0] = r.d_[0];
    d_ptr[i * 3 + 1] = r.d_[1];
    d_ptr[i * 3 + 2] = r.d_[2];
    p_ptr[i * 3 + 0] = r.p_[0];
    p_ptr[i * 3 + 1] = r.p_[1];
    p_ptr[i * 3 + 2] = r.p_[2];
    w_ptr[i] = r.w_;
    tf_ptr[i] = static_cast<uint16_t>(r.to_face_);
    // Per-ray crystal->world rotation (row-major mat_, same layout the kernel
    // applies as mat*v). InitRayFirstMs sampled this orientation and applied
    // its inverse to bring d_ into crystal-local space for tracing; the kernel
    // re-applies the forward rotation before projection (invariant 6).
    std::memcpy(rot_ptr + i * 9, r.crystal_rot_.GetMat(), 9 * sizeof(float));
  }
  // scrum-268.8 (DR-3): the host-gen fallback is per-ray too, mirroring the
  // device gen_root path. Step 9 hands this backend a zero-wl WlParam, so
  // InitRayFirstMs left root_w at 0 (no SPD weight) — the pool supplies both the
  // per-ray wavelength index AND the SPD weight (root_w = pool[wl_idx].spd_weight,
  // identical to the device path at gen_root_kernel). wl_idx is drawn from the
  // session rng: statistically equivalent to the device PCG stream (the
  // device-gen-vs-host-gen test asserts ds-corr, not bit parity). Without this,
  // host-gen rays carry weight 0 → a black image once curr_wl_ is the Step 9
  // sentinel.
  auto* wl_idx_ptr = static_cast<uint32_t*>([root_wl_idx_buf_ contents]);
  assert(wl_pool_size_ > 0u && !wl_pool_host_.empty() &&
         "wl_pool must be uploaded before host root-gen");
  for (size_t i = 0; i < n; i++) {
    uint32_t wl_idx = static_cast<uint32_t>(rng.GetUniform() * static_cast<float>(wl_pool_size_));
    if (wl_idx >= wl_pool_size_) {
      wl_idx = wl_pool_size_ - 1u;
    }
    wl_idx_ptr[i] = wl_idx;
    w_ptr[i] = wl_pool_host_[wl_idx].spd_weight;
  }
  // Accumulate the host-gen count too so a future device-gen-eligible call
  // within the same session keeps gen_ray_base globally monotone.
  root_ray_count += n;
  return n;
}

// task-267.4 (continuation-validation) golden-ray hook.
//
// Test-only injection path that bypasses InitRayFirstMs / device gen_root and
// copies caller-supplied initial rays straight into the device root buffers.
// Used by GoldenRay* tests in test_metal_trace_parity.cpp to drive the real
// Metal trace kernel with analytically constructed rays (normal incidence,
// Snell 30°, etc.) and assert exit direction/weight against closed-form
// expectations.
//
// Lifetime: host.d / p / w / tf are read once here and copied into MTLBuffers
// (MTLResourceStorageModeShared — Apple unified memory; the `[buf contents]`
// pointer is the GPU-visible address). Caller can free / reuse the buffers
// after this call returns.
//
// crystal_rot_ is filled with the 3x3 identity matrix per ray: the test rays
// are constructed in the same frame the kernel uses for both tracing and
// projection, so no rotation is needed. (Production gen_root samples a per-ray
// orientation; injection sidesteps that entirely.)
//
// root_ray_count is intentionally NOT advanced: this path consumes no PCG
// stream, so leaving the counter alone keeps any subsequent dispatches (test
// path normally has none; production never takes this branch) on the same
// monotone schedule they'd have otherwise.
size_t MetalTraceBackend::Impl::InjectHostRoots(const HostRayBatch& host) {
  assert(host.d != nullptr && host.p != nullptr && host.w != nullptr && host.tf != nullptr &&
         "InjectHostRoots requires non-null d/p/w/tf");
  assert(host.count > 0 && "InjectHostRoots requires host.count > 0");
  assert(host.count <= root_capacity &&
         "InjectHostRoots: host.count exceeds EnsureRootBuffers allocation");
  size_t n = host.count;
  auto* d_ptr   = static_cast<float*>([root_d_buf contents]);
  auto* p_ptr   = static_cast<float*>([root_p_buf contents]);
  auto* w_ptr   = static_cast<float*>([root_w_buf contents]);
  auto* tf_ptr  = static_cast<uint16_t*>([root_tf_buf contents]);
  auto* rot_ptr = static_cast<float*>([root_rot_buf contents]);
  std::memcpy(d_ptr,  host.d,  n * 3 * sizeof(float));
  std::memcpy(p_ptr,  host.p,  n * 3 * sizeof(float));
  std::memcpy(w_ptr,  host.w,  n * sizeof(float));
  // IdType is uint16_t (def.hpp); root_tf_buf element width matches.
  std::memcpy(tf_ptr, host.tf, n * sizeof(uint16_t));
  for (size_t i = 0; i < n; i++) {
    float* m = rot_ptr + i * 9;
    m[0] = 1.0f; m[1] = 0.0f; m[2] = 0.0f;
    m[3] = 0.0f; m[4] = 1.0f; m[5] = 0.0f;
    m[6] = 0.0f; m[7] = 0.0f; m[8] = 1.0f;
  }
  // scrum-268.8 (DR-3): golden-ray injection is single-wl by construction
  // (tests set spec.wl.wl_ to a fixed value). Map per_batch_wl_ to a pool
  // index so the trace kernel reads matching (n_idx, cmf_*) for the
  // analytically derived rays.
  auto* wl_idx_ptr = static_cast<uint32_t*>([root_wl_idx_buf_ contents]);
  uint32_t wl_idx_batch = 0u;
  if (wl_pool_size_ > 0u) {
    float t = (per_batch_wl_ - 380.0f) * static_cast<float>(wl_pool_size_) / 400.0f - 0.5f;
    if (t < 0.0f) {
      wl_idx_batch = 0u;
    } else {
      uint32_t v = static_cast<uint32_t>(t + 0.5f);
      wl_idx_batch = (v >= wl_pool_size_) ? (wl_pool_size_ - 1u) : v;
    }
  }
  for (size_t i = 0; i < n; i++) {
    wl_idx_ptr[i] = wl_idx_batch;
  }
  return n;
}

// Maps host axis_dist + sun + wavelength into the device GenRootKernelParams.
// All angular fields are pre-converted to radians on the host so the kernel
// itself does zero degree↔radian conversions.
GenRootKernelParams MetalTraceBackend::Impl::BuildGenRootParams(
    const ScatteringSetting& setting, size_t crystal_ray_num) const {
  GenRootKernelParams gp{};
  gp.tri_count = static_cast<uint32_t>(current_crystal.TotalTriangles());
  assert(gp.tri_count <= 64u &&
         "BuildGenRootParams: tri_count > kMaxTriPerKernel — caller must fall back to host gen");
  gp.num_rays = static_cast<uint32_t>(crystal_ray_num);
  // gen_seed / gen_ray_base are filled by the caller (depend on session state).

  const SunParam& sun = spec.scene->light_source_.param_;
  gp.sun_lon        = (sun.azimuth_ + 180.0f) * math::kDegreeToRad;
  gp.sun_lat        = -sun.altitude_ * math::kDegreeToRad;
  gp.sun_half_angle = (sun.diameter_ * 0.5f) * math::kDegreeToRad;
  // scrum-268.8 (DR-3): per-ray weight = wl_pool[wl_idx].spd_weight. Pass
  // the pool modulo bound so the kernel hash maps a global ray index into
  // [0, wl_pool_size). EnsureWlPoolBuffer guarantees wl_pool_size_ > 0
  // before BuildGenRootParams runs.
  assert(wl_pool_size_ > 0u && "wl_pool_size_ must be resolved before BuildGenRootParams");
  gp.wl_pool_size   = wl_pool_size_;

  const AxisDistribution& axis_dist = setting.crystal_.axis_;
  // Latitude path / proposal type single-sourced with math.cpp + CUDA via
  // lat_path::SelectLatPath (scrum-328.2 Step 4).
  auto decision = lat_path::SelectLatPath(axis_dist);
  auto lat_type = axis_dist.latitude_dist.type;
  float lat_mean_rad = axis_dist.latitude_dist.mean * math::kDegreeToRad;
  float lat_std_rad  = axis_dist.latitude_dist.std  * math::kDegreeToRad;

  gp.lat_path        = lat_path::ToWireValue(decision.kind);
  gp.lat_dist_type   = static_cast<uint32_t>(lat_type);
  gp.lat_mean_rad    = lat_mean_rad;
  gp.lat_std_rad     = lat_std_rad;
  gp.lat_rejection_m = decision.rejection_m;
  // 330.2 S3b: node count is the shader-side signal that the LUT buffers hold
  // valid data (0 for every non-LUT path). The arrays themselves bind separately.
  gp.lat_lut_n = (decision.kind == lat_path::LatPathKind::kLutInverseCdf)
                     ? lumice::LatLut::kNodes
                     : 0u;

  gp.az_type     = static_cast<uint32_t>(axis_dist.azimuth_dist.type);
  gp.az_mean_rad = axis_dist.azimuth_dist.mean * math::kDegreeToRad;
  gp.az_std_rad  = axis_dist.azimuth_dist.std  * math::kDegreeToRad;
  gp.az_pad      = 0.0f;

  gp.roll_type     = static_cast<uint32_t>(axis_dist.roll_dist.type);
  gp.roll_mean_rad = axis_dist.roll_dist.mean * math::kDegreeToRad;
  gp.roll_std_rad  = axis_dist.roll_dist.std  * math::kDegreeToRad;
  gp.roll_pad      = 0.0f;
  return gp;
}

// Encodes a root-gen compute pass into the caller-provided command buffer
// without committing or waiting. Used by DispatchLayer (task-264 gen+trace
// fusion) to share a single Metal queue round-trip with the subsequent trace
// pass. Metal forbids overlapping active encoders on one cb, so callers MUST
// have no other live encoder when invoking this; this function calls
// endEncoding before returning.
void MetalTraceBackend::Impl::EncodeGenRoot(id<MTLCommandBuffer> cb,
                                            const GenRootKernelParams& gp) {
  @autoreleasepool {
    // 330.2 S3b: guarantee the LUT buffers exist before binding (Metal forbids
    // nil bindings even on the non-LUT paths that never read their contents).
    EnsureLatLutBuffers();
    id<MTLComputeCommandEncoder> enc = [cb computeCommandEncoder];
    [enc setComputePipelineState:gen_root_pso_];
    [enc setBuffer:root_d_buf     offset:0 atIndex:0];
    [enc setBuffer:root_p_buf     offset:0 atIndex:1];
    [enc setBuffer:root_w_buf     offset:0 atIndex:2];
    [enc setBuffer:root_tf_buf    offset:0 atIndex:3];
    [enc setBuffer:root_rot_buf   offset:0 atIndex:4];
    [enc setBuffer:tri_vtx_buf_   offset:0 atIndex:5];
    [enc setBuffer:tri_norm_buf_  offset:0 atIndex:6];
    [enc setBuffer:tri_area_buf_  offset:0 atIndex:7];
    [enc setBuffer:tri_to_poly_buf_ offset:0 atIndex:8];
    [enc setBytes:&gp length:sizeof(GenRootKernelParams) atIndex:9];
    // scrum-268.8 (DR-3): wavelength pool + per-ray wl_idx output buffer.
    [enc setBuffer:wl_pool_buf_     offset:0 atIndex:10];
    [enc setBuffer:root_wl_idx_buf_ offset:0 atIndex:11];
    // scrum-328.2 Step 1: attempt-count observability sibling. Lazily create
    // a dummy 4-byte buffer if this is the first EncodeGenRoot after Init,
    // so Metal validation is satisfied even when the test path is idle.
    if (lat_attempts_buf_ == nil) {
      lat_attempts_buf_ = [device newBufferWithLength:sizeof(int)
                                              options:MTLResourceStorageModeShared];
      assert(lat_attempts_buf_ != nil);
    }
    [enc setBuffer:lat_attempts_buf_ offset:0 atIndex:12];
    // code-review round 1 Major#2: use the per-ci EFFECTIVE offset stashed by
    // GenerateFirstLayerRootsForCi (base lat_attempts_ci_start_ + running
    // per-ci accumulation), not the raw base — see pending_attempts_ci_start_
    // doc.
    uint attempts_ctrl[2] = { attempts_enabled_ ? 1u : 0u,
                              static_cast<uint>(pending_attempts_ci_start_) };
    [enc setBytes:attempts_ctrl length:sizeof(attempts_ctrl) atIndex:13];
    // 330.2 S3b: unified latitude LUT arrays (read only when lat_path==LutInverseCdf).
    [enc setBuffer:lat_lut_theta_buf_ offset:0 atIndex:14];
    [enc setBuffer:lat_lut_cdf_buf_   offset:0 atIndex:15];
    [enc setBuffer:lat_lut_flip_buf_  offset:0 atIndex:16];
    NSUInteger threads = 64;
    NSUInteger groups = (static_cast<NSUInteger>(gp.num_rays) + threads - 1) / threads;
    [enc dispatchThreadgroups:MTLSizeMake(groups, 1, 1)
        threadsPerThreadgroup:MTLSizeMake(threads, 1, 1)];
    [enc endEncoding];
  }
}

// scrum-267 task-device-resident-continuation: derive transit_root_kernel
// params from the gen_root path. The kernel only consumes:
//   * gen_seed / gen_ray_base  (PCG stream)
//   * tri_count                (entry-point sampler bound)
//   * lat_path / lat_dist_type / lat_mean_rad / lat_std_rad / lat_rejection_m
//   * az_type / az_mean_rad / az_std_rad
//   * roll_type / roll_mean_rad / roll_std_rad
//   * num_rays
// and explicitly IGNORES: sun_lon / sun_lat / sun_half_angle / ray_weight
// (world dir comes from cont_d_in instead of sample_sph_cap; weight is carried
// through from cont_w_in). Future changes to GenRootKernelParams or
// BuildGenRootParams must check both consumers.
//
// gen_ray_base IS NOT set here — the caller fills it from transit_ray_count_
// just before encoding so the running counter advances monotonically across
// SimBatches (mirrors gen_root's per-dispatch advance from root_ray_count).
GenRootKernelParams MetalTraceBackend::Impl::BuildTransitRootParams(
    const ScatteringSetting& setting, size_t ci_n,
    uint32_t ms_layer_idx, uint32_t ci, size_t ray_base) const {
  GenRootKernelParams gp = BuildGenRootParams(setting, ci_n);
  // Override seed: transit stream must be statistically independent from
  // root-gen (gen_seed_ + gen_ray_base) and from the emit gate (gate_seed) so
  // multi-layer multi-crystal continuations do not share PCG draws.
  // kTransitNonce separates the transit family; (ms_layer_idx, ci) nonce
  // separates per-dispatch streams within the family. If gen_seed_ is 0
  // (device gen disabled) transit_seed degenerates to pure nonce, which is
  // still statistically independent from any other stream.
  constexpr uint32_t kTransitNonce = 0xA5A5A5A5u;
  gp.gen_seed     = gen_seed_ ^ kTransitNonce ^
                    (ms_layer_idx * 65537u + ci * 2654435761u);
  // gen_ray_base is a REQUIRED parameter (compiler-enforced), not a comment-
  // only contract: it MUST be the running transit_ray_count_ so per-SimBatch
  // dispatches on (layer,ci) consume disjoint PCG ranges. A prior version left
  // it 0-by-default + "caller must overwrite" in a comment; that soft contract
  // silently reused PCG streams across batches → orientation under-sampling
  // (scrum-267 continuation bug). Making it a parameter closes that class.
  //
  // task-gpu-rng-ray-index-uint64: SplitPcgRayBase provides the full 64-bit
  // ray-base as (lo, hi); the kernel mixes hi into the per-ray PCG seed so
  // >2^32 sessions no longer collapse on wrap.
  auto split = SplitPcgRayBase(ray_base);
  gp.gen_ray_base    = split.lo;
  gp.gen_ray_base_hi = split.hi;
  return gp;
}

void MetalTraceBackend::Impl::EncodeTransitRoot(
    id<MTLCommandBuffer> cb, const GenRootKernelParams& gp,
    int in_slot, size_t ci_start) {
  assert(transit_root_pso_ != nil && "transit_root_pso_ nil in EncodeTransitRoot");
  assert(cont_d[in_slot] != nil && cont_w[in_slot] != nil &&
         "EncodeTransitRoot: cont buffers must be allocated by EnsureContBuffer");
  @autoreleasepool {
    // 330.2 S3b: guarantee the LUT buffers exist before binding (see EncodeGenRoot).
    EnsureLatLutBuffers();
    id<MTLComputeCommandEncoder> enc = [cb computeCommandEncoder];
    [enc setComputePipelineState:transit_root_pso_];
    NSUInteger d_off = static_cast<NSUInteger>(ci_start) * 3u * sizeof(float);
    NSUInteger w_off = static_cast<NSUInteger>(ci_start) * sizeof(float);
    [enc setBuffer:cont_d[in_slot]    offset:d_off atIndex:0];
    [enc setBuffer:cont_w[in_slot]    offset:w_off atIndex:1];
    [enc setBuffer:root_d_buf         offset:0     atIndex:2];
    [enc setBuffer:root_p_buf         offset:0     atIndex:3];
    [enc setBuffer:root_w_buf         offset:0     atIndex:4];
    [enc setBuffer:root_tf_buf        offset:0     atIndex:5];
    [enc setBuffer:root_rot_buf       offset:0     atIndex:6];
    [enc setBuffer:tri_vtx_buf_       offset:0     atIndex:7];
    [enc setBuffer:tri_norm_buf_      offset:0     atIndex:8];
    [enc setBuffer:tri_area_buf_      offset:0     atIndex:9];
    [enc setBuffer:tri_to_poly_buf_   offset:0     atIndex:10];
    [enc setBytes:&gp length:sizeof(GenRootKernelParams) atIndex:11];
    // scrum-268.8 (DR-3): per-ray wavelength carrier through transit. The
    // in-slot cont_wl_idx slice mirrors the cont_d / cont_w offset stride.
    NSUInteger wl_off = static_cast<NSUInteger>(ci_start) * sizeof(uint32_t);
    [enc setBuffer:cont_wl_idx_buf_[in_slot] offset:wl_off atIndex:12];
    [enc setBuffer:root_wl_idx_buf_          offset:0      atIndex:13];
    // 330.2 S3b: unified latitude LUT arrays (read only when lat_path==LutInverseCdf).
    [enc setBuffer:lat_lut_theta_buf_        offset:0      atIndex:14];
    [enc setBuffer:lat_lut_cdf_buf_          offset:0      atIndex:15];
    [enc setBuffer:lat_lut_flip_buf_         offset:0      atIndex:16];
    // task-331.5: per-ray component mask carrier through transit (sibling of
    // wl_idx). Same ci_start slice stride, 8B/ray. Rebased onto scrum-332:
    // LUT took 14/15/16, so component moved 14/15→17/18 (matches transit_root_kernel).
    NSUInteger comp_off = static_cast<NSUInteger>(ci_start) * sizeof(uint64_t);
    [enc setBuffer:cont_component_buf_[in_slot] offset:comp_off atIndex:17];
    [enc setBuffer:root_component_buf_          offset:0        atIndex:18];
    NSUInteger threads = 64;
    NSUInteger groups  = (static_cast<NSUInteger>(gp.num_rays) + threads - 1) / threads;
    [enc dispatchThreadgroups:MTLSizeMake(groups, 1, 1)
        threadsPerThreadgroup:MTLSizeMake(threads, 1, 1)];
    [enc endEncoding];
  }
}

void MetalTraceBackend::Impl::EncodeShuffleCont(
    id<MTLCommandBuffer> cb, int in_slot, int out_slot,
    uint32_t n, uint32_t seed) {
  assert(shuffle_pso_ != nil && "shuffle_pso_ nil in EncodeShuffleCont");
  assert(cont_d[in_slot] != nil && cont_w[in_slot] != nil &&
         cont_wl_idx_buf_[in_slot] != nil &&
         "EncodeShuffleCont: source cont buffers must be allocated");
  assert(cont_d[out_slot] != nil && cont_w[out_slot] != nil &&
         cont_wl_idx_buf_[out_slot] != nil &&
         "EncodeShuffleCont: dest cont buffers must be allocated (EnsureContBuffer)");
  @autoreleasepool {
    id<MTLComputeCommandEncoder> enc = [cb computeCommandEncoder];
    [enc setComputePipelineState:shuffle_pso_];
    [enc setBuffer:cont_d[in_slot]          offset:0 atIndex:0];
    [enc setBuffer:cont_w[in_slot]          offset:0 atIndex:1];
    [enc setBuffer:cont_wl_idx_buf_[in_slot] offset:0 atIndex:2];
    [enc setBuffer:cont_d[out_slot]         offset:0 atIndex:3];
    [enc setBuffer:cont_w[out_slot]         offset:0 atIndex:4];
    [enc setBuffer:cont_wl_idx_buf_[out_slot] offset:0 atIndex:5];
    [enc setBytes:&n    length:sizeof(uint32_t) atIndex:6];
    [enc setBytes:&seed length:sizeof(uint32_t) atIndex:7];
    // task-331.5: gather the per-ray component mask in lockstep (LANDMINE —
    // omitting it decorrelates the mask from its ray across the layer boundary,
    // exactly the CPU SwapRay bug reproduced on device).
    [enc setBuffer:cont_component_buf_[in_slot]  offset:0 atIndex:8];
    [enc setBuffer:cont_component_buf_[out_slot] offset:0 atIndex:9];
    // 256 to match the CUDA shuffle blockDim (cross-backend symmetry) and the
    // trace kernel's dispatch, capped to the device ceiling. Non-uniform
    // dispatchThreads handles the n % tg remainder; the kernel guards tid >= n.
    NSUInteger tg = std::min<NSUInteger>(256, shuffle_pso_.maxTotalThreadsPerThreadgroup);
    [enc dispatchThreads:MTLSizeMake(n, 1, 1)
        threadsPerThreadgroup:MTLSizeMake(tg, 1, 1)];
    [enc endEncoding];
  }
}

void MetalTraceBackend::Impl::DispatchLayer(size_t num_rays,
                                            id<MTLBuffer> r_d, id<MTLBuffer> r_p,
                                            id<MTLBuffer> r_w, id<MTLBuffer> r_tf,
                                            uint32_t ms_mode, int out_slot,
                                            uint32_t counter_init,
                                            uint32_t crystal_id, uint32_t ms_layer_idx,
                                            id<MTLCommandBuffer> existing_cb) {
  // task-268.7 invariant: caller drains any prior pending CB before issuing
  // the next DispatchLayer; otherwise we would overwrite pending_cb_ and lose
  // its continuation counter / exit stats. TraceLayer's ci-loop calls
  // WaitAndReadbackLayer() at every iteration tail.
  assert(pending_cb_ == nil && "DispatchLayer: pending_cb_ must be drained before next dispatch");
  KernelParams params{};
  // scrum-268.8 (DR-3): per-batch n_idx + cie_x/y/z dropped (per-ray wl_pool
  // lookup superseded). current_n_idx + cie_* impl state survive for the
  // host-side parity audit only and are no longer wired into KernelParams.
  params.max_hits = static_cast<uint32_t>(spec.scene->max_hits_);
  params.poly_cnt = static_cast<uint32_t>(current_crystal.PolygonFaceCount());
  params.num_rays = static_cast<uint32_t>(num_rays);
  params.img_w    = static_cast<uint32_t>(width);
  params.img_h    = static_cast<uint32_t>(height);
  params.ms_mode  = ms_mode;
  params.out_cap  = static_cast<uint32_t>(out_cap);
  // 315.3: single POD carries all projection routing (proj_type / az0 /
  // r_scale / max_abs_dz / scale / rot / etc.) into the kernel exit tail.
  params.proj     = proj_params_;
  // task-358.3 (renamed from capture_component): gate the test-only capture
  // ring append (0 in prod → append branch skipped).
  params.capture_ray_mask = capture_ray_mask_ ? 1u : 0u;
  // task-358.1 (metal-color-parity): Design-2 color-gate hot-path knobs.
  // has_color_groups = 0 when the session has no raypath_color config (or the
  // ColorGateTable has no matching placements) → MSL color pass is skipped
  // entirely (AC4 zero-cost). When 1, the MSL indexes color descriptors at
  // color_desc_offset + gate_slot * color_max_groups_per_slot + g, and color
  // component-bits at color_bits_offset + color_slot * kDeviceFilterMaxOrClauses
  // + summand. Offsets computed in EnsureFilterBuffers.
  params.has_color_groups = has_color_groups_ ? 1u : 0u;
  params.color_desc_offset = color_desc_offset_;
  params.color_bits_offset = color_bits_offset_;
  params.color_max_groups_per_slot = static_cast<uint32_t>(kColorMaxGroupsPerSlot);
  // task-358.1 Step 4 (AC3 device-side Y-lane accumulation): pack the runtime
  // class table into KernelParams. class_count_==0 → hot-path branch skip in
  // MSL (AC4 zero-cost); non-zero → per-class predicate over this_mask + atomic
  // add of cmf_y * cw into class_lane_buf_[c * W*H + pix].
  params.color_class_count = static_cast<uint32_t>(class_count_);
  std::memset(params.color_class_bits, 0, sizeof(params.color_class_bits));
  std::memset(params.color_class_combine, 0, sizeof(params.color_class_combine));
  for (size_t c = 0; c < class_count_; ++c) {
    const auto& cls = class_table_.classes_[c];
    params.color_class_bits[c] = cls.member_bits_;
    params.color_class_combine[c] = (cls.combine_ == ColorClassCombine::kAll) ? 1u : 0u;
  }
  // S1 device-fused: exit_cap and face_seq_cap are unused (exit buffers gone).
  params.exit_cap     = 0u;
  params.crystal_id   = crystal_id;
  params.face_seq_cap = 0u;
  params.ms_layer_idx = ms_layer_idx;
  // Emit-gate (scrum-267 task-fused-emit-gate Step 6 + scrum-302 S1).
  // For ms_mode==1 dispatches hop_ms_prob_[out_slot] carries the producing-
  // layer prob, populated by TraceLayer's pre-ci-loop setup; the gate's
  // `pcg_uniform() < ms_prob` mirrors legacy simulator.cpp:468 rng < prob_.
  // For ms_mode==0 (final layer) the device-fused gate now runs the SAME prob
  // draw on device (scrum-302): legacy CollectData drops filter-pass rng<prob
  // "would-continue-at-final" rays (there is no next layer) and emits only the
  // (1-prob) remainder. Forcing 0 here (the pre-302 value, valid only when the
  // host ReadbackExitRays path owned the final-layer prob) made the device emit
  // 100% of filter-pass exits → metal/legacy energy = 1/(1-prob) (2× at p=0.5,
  // caught by parity energy-conservation on prob05 / bd_filter / complex_filter).
  // Pass the real final-layer prob so the kernel ms_mode==0 gate can drop them.
  if (ms_mode == 1u) {
    params.ms_prob = hop_ms_prob_[out_slot];
  } else {
    params.ms_prob = last_ms_prob_;
  }
  // gate_seed mixes gen_seed_ with a (layer, crystal) nonce so each dispatch
  // owns an independent PCG stream — without the nonce two dispatches with the
  // same tid would draw the same prob, biasing multi-crystal energy.
  params.gate_seed = gen_seed_ ^
                     (ms_layer_idx * 65537u + crystal_id * 2654435761u);
  params.filter_desc_max_ci = static_cast<uint32_t>(filter_desc_max_ci_);
  // task-device-flat-and-terms: kernel derives `gate_and_term_counts` from the
  // shared `gate_sub_desc_buf` via this byte offset.
  params.and_term_counts_base_offset = and_term_counts_base_offset_;
  // crystal_config_id is consumed only by DeviceFilterMatchCrystal; current
  // canonical configs leave it at kInvalidId (0xffff) so the filter rejects.
  uint16_t cfg_id = current_crystal.config_id_;
  params.crystal_config_id =
      (cfg_id == kInvalidId) ? 0xFFFFu : static_cast<uint32_t>(cfg_id);
  // Defensive sanity bounds — silent host/MSL field reorder would set
  // crystal_id to a different field's value (max_hits ~8, face_seq_cap ~8,
  // ms_layer_idx ~0). The bounds below are narrow enough to fire for any
  // reorder that puts a non-crystal field into the crystal_id slot.
  assert(crystal_id < kMaxCrystalNum && "crystal_id out of range — check KernelParams layout");
  // face_seq_cap_ assertion removed in S1 device-fused (field unused, always 0).

  EnsureRecSink(num_rays);
  EnsureContBuffer(out_slot);
  // task-331.5: capture ring must be bindable (Metal disallows nil buffers).
  // Sized to out_cap when capturing; a 1-element dummy otherwise (kernel's
  // capture branch is gated by KernelParams.capture_ray_mask == 0).
  EnsureComponentCaptureBuffers(capture_ray_mask_ ? out_cap : 1u);
  // Multi-ci append semantics: counter_init carries the running offset of
  // already-written continuation rays from previous ci dispatches; the
  // kernel's atomic_fetch_add resumes from there. ci=0 always passes 0
  // (equivalent to the previous unconditional reset).
  // last_stats is NOT reset here — TraceLayer zeroes it once before the
  // ci loop, and each DispatchLayer accumulates via += below.
  if (counter_buf == nil) {
    counter_buf = [device newBufferWithLength:sizeof(uint32_t)
                                      options:MTLResourceStorageModeShared];
  }
  *static_cast<uint32_t*>([counter_buf contents]) = counter_init;
  // exit_count / exit_w_sum are atomic accumulators populated by the kernel.
  // Reset before each dispatch and add the readback into last_stats so the
  // per-ci contributions sum correctly.
  if (exit_count_buf == nil) {
    exit_count_buf = [device newBufferWithLength:sizeof(uint32_t)
                                         options:MTLResourceStorageModeShared];
    assert(exit_count_buf != nil);
  }
  if (exit_w_sum_buf == nil) {
    exit_w_sum_buf = [device newBufferWithLength:sizeof(float)
                                         options:MTLResourceStorageModeShared];
    assert(exit_w_sum_buf != nil);
  }
  *static_cast<uint32_t*>([exit_count_buf contents]) = 0u;
  *static_cast<float*>([exit_w_sum_buf contents]) = 0.0f;

  // task-268.7: caller may supply `existing_cb` so transit_root + trace share
  // one CB on non-first MS layers (Step 3). Same Apple-Silicon shared-memory
  // RAW-visibility argument as gen+trace fusion (explore-263 / task-264).
  id<MTLCommandBuffer> cb = existing_cb != nil ? existing_cb : [queue commandBuffer];
  // task-264 gen+trace fusion: if a device root-gen was stashed by
  // GenerateFirstLayerRootsForCi for this ci, encode it into the same cb
  // ahead of the trace pass. Two sequential compute encoders on one cb share
  // a single commit/wait round-trip; Apple Silicon + MTLResourceStorageModeShared
  // root_* buffers guarantee read-after-write visibility across encoders
  // without an explicit memoryBarrierWithScope (verified by parity at corr
  // 0.946 in explore-263). Mutually exclusive with `existing_cb != nil`:
  // gen_root only stashes on the FIRST MS layer (transit-root is non-first).
  if (pending_gen_params_.has_value()) {
    assert(existing_cb == nil && "gen_root stash + existing_cb both set");
    EncodeGenRoot(cb, *pending_gen_params_);
    pending_gen_params_.reset();
  }
  id<MTLComputeCommandEncoder> enc = [cb computeCommandEncoder];
  [enc setComputePipelineState:pso];
  [enc setBuffer:r_d            offset:0 atIndex:0];
  [enc setBuffer:r_p            offset:0 atIndex:1];
  [enc setBuffer:r_w            offset:0 atIndex:2];
  [enc setBuffer:r_tf           offset:0 atIndex:3];
  [enc setBuffer:poly_n_buf     offset:0 atIndex:4];
  [enc setBuffer:poly_d_buf     offset:0 atIndex:5];
  [enc setBuffer:centroid_buf   offset:0 atIndex:6];
  [enc setBytes:&params length:sizeof(KernelParams) atIndex:7];
  [enc setBuffer:xyz_image      offset:0 atIndex:8];
  [enc setBuffer:cont_d[out_slot]  offset:0 atIndex:9];
  // scrum-268.8 (DR-3): slots 10/12 retired from out_p / out_tf and reclaimed
  // for the wavelength pool (read) + per-ray root wl_idx (read). cont_w stays
  // at 11; cont_wl_idx (write) moves out to slot 29 / exit_wl_idx (write) to
  // slot 30 — the only two free slots left under Metal's 30-binding ceiling.
  [enc setBuffer:wl_pool_buf_      offset:0 atIndex:10];
  [enc setBuffer:cont_w[out_slot]  offset:0 atIndex:11];
  [enc setBuffer:root_wl_idx_buf_  offset:0 atIndex:12];
  [enc setBuffer:counter_buf    offset:0 atIndex:13];
  [enc setBuffer:rec_sink_buf   offset:0 atIndex:14];
  [enc setBuffer:exit_count_buf offset:0 atIndex:15];
  [enc setBuffer:exit_w_sum_buf offset:0 atIndex:16];
  [enc setBuffer:root_rot_buf   offset:0 atIndex:17];
  // S1 device-fused: slot 18 = landed_weight scalar (previously exit_ray_d).
  // Slots 19-23 (exit_ray_w/slot/crystal_id/face_seq_len/data) and 28
  // (exit_ms_layer) and 30 (exit_wl_idx) are freed; exit records are no
  // longer materialised — the kernel accumulates directly into image + landed_weight.
  [enc setBuffer:landed_weight_buf_ offset:0 atIndex:18];
  // Emit-gate filter state (scrum-267 task-fused-emit-gate Step 4b). Bound for
  // every dispatch (Metal disallows nil buffers). EnsureFilterBuffers guarantees
  // non-nil even in no-filter sessions via the 1-byte dummy fallback (R5 fix).
  // Slots 24-27 carry filter descriptors; slot 28 is freed (was exit_ms_layer).
  [enc setBuffer:filter_desc_buf_      offset:0 atIndex:24];
  [enc setBuffer:getfn_offsets_buf_    offset:0 atIndex:25];
  [enc setBuffer:getfn_bytes_buf_      offset:0 atIndex:26];
  [enc setBuffer:complex_sub_desc_buf_ offset:0 atIndex:27];
  // scrum-268.8 (DR-3): cont_wl_idx propagates the photon's lifetime
  // wavelength tag into the continuation ring (slot 29). Slot 30 freed.
  [enc setBuffer:cont_wl_idx_buf_[out_slot] offset:0 atIndex:29];
  // task-331.5 (raypath-color foundation): per-ray component-mask carry (19/20),
  // summand→bit table (21), and the emit capture ring (22/23/28). All bound
  // unconditionally (Metal disallows nil); the kernel's capture branch is gated
  // by KernelParams.capture_ray_mask so production dispatches skip them.
  [enc setBuffer:root_component_buf_       offset:0 atIndex:19];
  [enc setBuffer:cont_component_buf_[out_slot] offset:0 atIndex:20];
  [enc setBuffer:gate_component_bits_buf_  offset:0 atIndex:21];
  [enc setBuffer:exit_comp_mask_buf_       offset:0 atIndex:22];
  [enc setBuffer:exit_comp_w_buf_          offset:0 atIndex:23];
  [enc setBuffer:exit_comp_cnt_buf_        offset:0 atIndex:28];
  // task-358.1 Step 4 (AC3 device-side Y-lane accumulation): slot 30 = per-
  // class atomic_float accumulator (dummy 4-byte alloc when class_count==0).
  // Slots 0-29 are fully occupied by the trace kernel — 30 is the only free
  // slot under Metal's per-stage 30-buffer ceiling (assumption B).
  [enc setBuffer:class_lane_buf_           offset:0 atIndex:30];

  NSUInteger tg = std::min<NSUInteger>(256, pso.maxTotalThreadsPerThreadgroup);
  [enc dispatchThreads:MTLSizeMake(num_rays, 1, 1)
   threadsPerThreadgroup:MTLSizeMake(tg, 1, 1)];
  [enc endEncoding];
  // task-268.7: commit without waiting; the caller drains via
  // WaitAndReadbackLayer() at the ci-loop tail. Park CB + out_slot so the
  // readback routes to the correct cont_counts slot.
  [cb commit];
  pending_cb_ = cb;
  pending_out_slot_ = out_slot;
}

void MetalTraceBackend::Impl::WaitAndReadbackLayer() {
  if (pending_cb_ == nil) {
    return;
  }
  id<MTLCommandBuffer> cb = pending_cb_;
  int out_slot = pending_out_slot_;
  // Clear up-front so a fail-and-assert path leaves pending_cb_ in a sane
  // state (subsequent DispatchLayer's drain-invariant assert would otherwise
  // double-trip on a CB we already gave up on).
  pending_cb_ = nil;
  [cb waitUntilCompleted];
  if (cb.status != MTLCommandBufferStatusCompleted) {
    ILOG_ERROR(EffectiveLogger(logger_),
               "MetalTraceBackend: GPU dispatch failed: {}",
               cb.error.localizedDescription.UTF8String);
    assert(false && "MetalTraceBackend: GPU dispatch failed");
  }

  uint32_t produced = *static_cast<uint32_t*>([counter_buf contents]);
  if (produced > out_cap) {
    ILOG_ERROR(EffectiveLogger(logger_),
               "MetalTraceBackend: continuation overflow produced={} out_cap={}",
               produced, out_cap);
    assert(false && "MetalTraceBackend: continuation overflow");
    // Release-build clamp: assert is compiled out, so clamp produced to out_cap
    // to prevent the next layer reading cont_* buffers out of bounds on the GPU.
    produced = static_cast<uint32_t>(out_cap);
  }
  if (produced > max_produced) {
    max_produced = produced;
  }
  cont_counts[out_slot] = produced;

  // Readback exit-ray stats for the parity harness. Both atomics live in
  // Shared storage, so a plain host load after waitUntilCompleted is
  // sufficient (no blit encoder required on unified memory). Accumulate
  // into last_stats so the multi-ci loop's contributions sum correctly;
  // TraceLayer is responsible for zeroing last_stats before the ci loop.
  last_stats.exit_count += *static_cast<uint32_t*>([exit_count_buf contents]);
  last_stats.exit_w_sum += *static_cast<float*>([exit_w_sum_buf contents]);
}

void MetalTraceBackend::Impl::Reset() {
  if (max_produced > 0) {
    ILOG_DEBUG(EffectiveLogger(logger_),
               "MetalTraceBackend: session ended — max continuation produced={} (out_cap={})",
               max_produced, out_cap);
  } else {
    ILOG_DEBUG(EffectiveLogger(logger_),
               "MetalTraceBackend: session ended — single-layer MS, no continuation");
  }
  in_session = false;
  ms_idx = 0;
  // task-264 gen+trace fusion: defensive clear at session boundary. In a
  // healthy run the stash is consumed by DispatchLayer within the same ci
  // iteration that set it; clearing here protects against stale state if a
  // session ends mid-ci on an error path.
  pending_gen_params_.reset();
  // task-268.7: in a healthy run TraceLayer drains pending_cb_ at the ci-loop
  // tail (and again before overflow check); clearing here is the defensive
  // session-boundary cleanup mirror of the gen_params reset above.
  if (pending_cb_ != nil) {
    [pending_cb_ waitUntilCompleted];
    pending_cb_ = nil;
  }
  pending_out_slot_ = 0;
  // root_ray_count INTENTIONALLY persists across Reset(): each EndSession()
  // is followed (in the next BeginSession) by another GenerateFirstLayerRootsForCi
  // dispatch that must observe a globally monotone gen_ray_base. The counter
  // is reset only on the first seeding (BeginSession + !seeded gate), in
  // lock-step with rng.SetSeed. Mirror bug of 258.10: if we reset here, the
  // GPU PCG stream collapses to the same 128-ray range every SimBatch and
  // narrow raypath-filter parity (test_parity_multi_ms_prob05_filter) drops
  // to ds_corr=0.33. See task-260.5 SUMMARY / plan §1.
  // gen_seed_ is re-derived from spec.seed every BeginSession; clear so a
  // session that omits spec.seed cannot inherit a previous activation.
  gen_seed_ = 0u;
  width = 0;
  height = 0;
  // scrum-268.8 (DR-3): cie_x/y/z removed from Impl.
  // 315.3: unified projection params reset to POD default.
  proj_params_ = lm_proj::ProjParams{};
  have_crystal = false;
  current_n_idx = 0.0f;
  out_cap = 0;
  max_produced = 0;
  cont_counts[0] = cont_counts[1] = 0;
  last_stats = LayerStats{};
  // Exit seam (scrum-258.1): buffers + capacity persist across sessions
  // (grow-only); only the atomic slot needs reset for the next session,
  // which happens in TraceLayer on the first MS layer.
  // scrum-258.2: face_seq_cap_ is re-derived per BeginSession from
  // scene.max_hits_; clear to make a session-mid leak obvious.
  face_seq_cap_ = 0;
  // scrum-258.3 + scrum-267 Task 3: per-layer filter+prob state.
  // hop_ms_prob_ (device emit gate) + last_layer_* (final-layer host
  // filter+prob in ReadbackExitRays) are populated by TraceLayer's ci loop in
  // the next session; clear here so a stale layer's settings cannot bleed
  // across sessions if BeginSession skips re-assigning a particular slot.
  for (int s = 0; s < 2; s++) {
    hop_ms_prob_[s] = 0.0f;
  }
  last_layer_crystals_.clear();
  last_layer_axis_dists_.clear();
  last_layer_filter_configs_.clear();
  last_ms_prob_ = 0.0f;
  last_ms_layer_idx_ = 0u;
  // scrum-267 task-msl-filter-match-port (Step 4): per-session device filter
  // state. Re-uploaded on the next BeginSession; clear here so a stale config
  // cannot bleed into a re-used backend instance.
  filter_desc_count_ = 0;
  filter_desc_max_ci_ = 0;
  filter_desc_buf_ = nil;
  getfn_offsets_buf_ = nil;
  getfn_bytes_buf_ = nil;
  complex_sub_desc_buf_ = nil;
  and_term_counts_base_offset_ = 0u;
  spec = SessionSpec{};
}


// ============================== MetalTraceBackend ============================

MetalTraceBackend::MetalTraceBackend(Logger* logger)
    : impl_(std::make_unique<Impl>()) {
  impl_->logger_ = logger;
}

MetalTraceBackend::~MetalTraceBackend() = default;

void MetalTraceBackend::BeginSession(const SessionSpec& spec) {
  assert(!impl_->in_session && "BeginSession called on an already-open session");
  assert(spec.scene != nullptr);
  assert(spec.render != nullptr);
  // 315.3: the exit tail now projects via lm_proj::ProjectExitToPixel — the
  // SAME single source as the CPU parity oracle (scatter_accum.hpp) — so every
  // lens type (incl. globe, 315.4) produces byte-identical pixels to legacy CPU
  // at any view/fov.

  impl_->spec = spec;
  impl_->in_session = true;
  impl_->ms_idx = 0;
  // root_ray_count is reset ONLY at first seeding below (the !seeded gate, in
  // lock-step with rng.SetSeed). Resetting here unconditionally would collapse
  // the GPU PCG stream to a single 128-ray range every SimBatch — the mirror
  // bug of 258.10 (RNG re-seed per batch). See task-260.5 fix.
  impl_->width  = spec.render->resolution_[0];
  impl_->height = spec.render->resolution_[1];

  Rotation camera_rot = MakeCameraRotation(*spec.render);
  // scrum-268.8 (DR-3): per-batch ComputeCmf(spec.wl.wl_) deleted — CMF is
  // sourced per-ray from wl_pool[wl_idx] populated below in TraceLayer's ci
  // loop. spec.wl.wl_ remains the simulator-sampled per-batch sentinel until
  // simulator.cpp:663 is retired (M4 / Step 9).
  //
  // Capture the spectrum mode for ResolveLayerCrystalForCi: when the scene's
  // light source is an IlluminantType variant, ComputeWlPool samples M
  // wavelengths uniformly over [380, 780] and weights them by the standard
  // illuminant SPD. For the discrete-WlParam-list path (simulator.cpp:672)
  // the pool degenerates to a single (spec_wl, spec_weight) entry replicated
  // across all M slots so per-ray lookup still works but every ray sees the
  // same wl — matches legacy single-wl semantics.
  const auto& spectrum = spec.scene->light_source_.spectrum_;
  if (const auto* ill = std::get_if<IlluminantType>(&spectrum)) {
    impl_->illuminant_mode_ = true;
    impl_->illuminant_      = *ill;
  } else {
    impl_->illuminant_mode_ = false;
  }
  impl_->per_batch_wl_     = spec.wl.wl_;
  impl_->per_batch_weight_ = spec.wl.weight_;

  // Projection routing (315.3): single-source ProjParams via BuildProjParams —
  // predigests per-type scale, dual-fisheye r_scale/overlap, rectangular az0,
  // and the camera rotation. DispatchLayer copies this into KernelParams::proj;
  // the kernel exit tail calls lm_proj::ProjectExitToPixel(proj, world_exit...),
  // identical to the CPU parity oracle ScatterOutgoingToXyz.
  const float short_pix =
      static_cast<float>(std::min(spec.render->resolution_[0], spec.render->resolution_[1]));
  impl_->proj_params_ = BuildProjParams(*spec.render, camera_rot, short_pix);

  // Seed contract: first call with spec.seed != 0 seeds the RNG; repeated
  // calls with the same seed are no-ops (normal per-SimBatch pattern).
  // A different non-zero seed on the same backend instance is a programming
  // error and triggers the assert. To reseed, destroy and recreate.
  assert(!impl_->seeded || spec.seed == 0 || spec.seed == impl_->seeded_seed);
  if (spec.seed != 0 && !impl_->seeded) {
    impl_->rng.SetSeed(spec.seed);
    RandomNumberGenerator::GetInstance().SetSeed(spec.seed);
    impl_->seeded_seed = spec.seed;
    impl_->seeded = true;
    // task-260.5: zero the device-gen counter ONLY here, alongside SetSeed.
    // Subsequent BeginSession cycles within the same Run() (same instance,
    // same seed) leave root_ray_count untouched so successive SimBatches
    // consume disjoint PCG ranges (gen_ray_base monotonically increases).
    impl_->root_ray_count = 0;
    // scrum-267 task-device-resident-continuation bugfix: transit_ray_count_
    // shares root_ray_count's reset contract — zero ONLY here so per-batch
    // transit dispatches consume disjoint PCG ranges per (layer, ci) stream.
    impl_->transit_ray_count_ = 0;
  }
  // Device root-gen activation (task-260.2). spec.seed != 0 implies the
  // single-worker determinism contract (server.cpp:184), which is the case we
  // accelerate. spec.seed == 0 keeps gen_seed_ at 0, which forces the host
  // path in GenerateFirstLayerRootsForCi via the can_use_device_gen guard in
  // TraceLayer.
  impl_->gen_seed_ = static_cast<uint32_t>(spec.seed);

  // task-282: BeginSession sets in_session=true above (line 2766) BEFORE the
  // Ensure* phase. If EnsurePso (or any downstream Ensure*) throws — e.g. the
  // macOS 26.5 entry-point-missing path — the SimulateOneWavelengthWithBackend
  // EndOnExit RAII guard is not yet constructed (its constructor is reached
  // only on a normal BeginSession return), so EndSession()/Reset() would not
  // run and `in_session` would leak as true. Catch + Reset()+rethrow restores
  // the session-clean invariant so the simulator can drop the backend and
  // continue with the legacy CPU path without tripping a future BeginSession's
  // !in_session assert.
  try {
    impl_->EnsureDevice();
    impl_->EnsurePso();
    // S1 device-fused: landed_weight scalar (1 × float, MTLResourceStorageModeShared).
    // scrum-312 third clock: allocate ONCE (nil-check); it persists as a cross-batch
    // accumulator like xyz_image. Allocate it BEFORE EnsureImage so EnsureImage
    // resets BOTH twin accumulators atomically on a fresh accumulation region /
    // shape change (review-Major-2). No per-call zero here.
    if (impl_->landed_weight_buf_ == nil) {
      impl_->landed_weight_buf_ =
          [impl_->device newBufferWithLength:sizeof(float)
                                     options:MTLResourceStorageModeShared];
      assert(impl_->landed_weight_buf_ != nil);
    }
    impl_->EnsureImage(impl_->width, impl_->height);
    // scrum-268.8 (DR-3): allocate the wavelength pool buffer once per backend
    // (size invariant across sessions) and populate it once per BeginSession.
    // Pool content depends only on (illuminant mode, per_batch_wl_) — both
    // captured above — and on Crystal::GetRefractiveIndex which is the global
    // ice model (identical for every crystal shape in the session), so a single
    // upload covers every ci dispatch.
    impl_->EnsureWlPoolBuffer();
  } catch (...) {
    impl_->Reset();
    throw;
  }
  {
    // Use an empty Crystal proxy — GetRefractiveIndex ignores its argument
    // and consults the global IceRefractiveIndex model directly.
    Crystal proxy{};
    ComputeWlPool(proxy, impl_->illuminant_mode_, impl_->illuminant_,
                  impl_->per_batch_wl_, impl_->per_batch_weight_,
                  impl_->wl_pool_size_, impl_->wl_pool_host_);
    std::memcpy([impl_->wl_pool_buf_ contents], impl_->wl_pool_host_.data(),
                static_cast<size_t>(impl_->wl_pool_size_) * sizeof(WlEntry));
  }

  impl_->cont_counts[0] = 0;
  impl_->cont_counts[1] = 0;
  impl_->out_cap = 0;
  // Exit metadata (scrum-258.2): per-slot stride of buffer(23). task-284 bumped
  // ExitFaceSeq::kCap from 15 to kMaxHits=64; the min() clamp now caps at the
  // configurable kMaxHits ceiling. Set BEFORE EnsureExitBuffers (called from
  // TraceLayer's first-MS branch) so the device allocation reflects the actual
  // stride.
  impl_->face_seq_cap_ = std::min<uint32_t>(
      static_cast<uint32_t>(spec.scene->max_hits_), static_cast<uint32_t>(ExitFaceSeq::kCap));
  // scrum-267 task-fused-emit-gate Step 8: initialize the final-layer index
  // from the scene config so ReadbackExitRays can route mid-exits even when
  // the final TraceLayer call is skipped (e.g. prob=0.0 sinks every ray to
  // mid-exit on layer 0 → layer 1 sees zero inputs → early-returns before
  // last_ms_layer_idx_ would otherwise be written in TraceLayer).
  if (!spec.scene->ms_.empty()) {
    impl_->last_ms_layer_idx_ =
        static_cast<uint8_t>(spec.scene->ms_.size() - 1);
  }

  // task-358.1 (metal-color-parity): build the Design-2 placement-scoped color
  // gate table BEFORE EnsureFilterBuffers reads it. Empty raypath_color →
  // empty ColorGateTable → EnsureFilterBuffers appends only zero-init color
  // descriptors and leaves has_color_groups_ = false (AC4 zero-cost path).
  // Mirrors CPU cpu_trace_backend.cpp:263-268 BuildColorGateTable call.
  {
    RaypathColorConfig empty_color;
    const RaypathColorConfig& color_cfg = spec.raypath_color ? *spec.raypath_color : empty_color;
    impl_->color_gate_table_ = BuildColorGateTable(color_cfg, *spec.scene);
    // task-358.1 Step 4: build the runtime color-class table (server-side
    // parity — same input, same output, matches RenderConsumer::class_table_).
    // Empty raypath_color → empty class_table → class_count_==0 → MSL color
    // lane accumulation branch is skipped (AC4 zero-cost) and class_lane_buf_
    // stays a 4-byte dummy allocation (Metal requires non-nil bindings).
    impl_->class_table_ = BuildColorClassTable(color_cfg, *spec.scene, impl_->color_gate_table_);
    const size_t class_count_raw = impl_->class_table_.classes_.size();
    // Runtime clamp (not just the assert below, which -DNDEBUG release builds
    // compile out): KernelParams.color_class_bits/combine are fixed-size
    // [kMaxColorClassesDevice] arrays. DispatchLayer writes them for
    // c < class_count_ with no further bound, so an unclamped class_count_
    // above kMaxColorClassesDevice is a real out-of-bounds write into
    // KernelParams on every dispatched layer (code-review round 2 Major —
    // triggerable by ordinary user config, no backend/consumer race needed).
    if (class_count_raw > kMaxColorClassesDevice) {
      ILOG_ERROR(EffectiveLogger(impl_->logger_),
                 "MetalTraceBackend::BeginSession: raypath_color configured {} color classes, exceeding "
                 "kMaxColorClassesDevice={} — dropping the excess {} class(es) rather than overflowing "
                 "KernelParams.color_class_bits/combine. Raise kMaxColorClassesDevice (+ MSL sibling) if this "
                 "config is intentional.",
                 class_count_raw, kMaxColorClassesDevice, class_count_raw - kMaxColorClassesDevice);
    }
    impl_->class_count_ = std::min(class_count_raw, kMaxColorClassesDevice);
    assert(impl_->class_count_ <= kMaxColorClassesDevice &&
           "raypath_color class count exceeds kMaxColorClassesDevice; raise both host + MSL "
           "sibling and audit KernelParams padding.");
  }

  // scrum-267 task-msl-filter-match-port (Step 4): upload per-session device
  // filter descriptors + GetFn tables. Production trace kernel does not yet
  // consume these (sub-task 2 will splice the gate); for now BeginSession
  // ensures the descriptors are available to the parity harness and that
  // sizing is exercised against real session configs.
  impl_->EnsureFilterBuffers(spec);

  // task-358.1 Step 4: allocate/resize the device Y-lane accumulator. Layout is
  // atomic_float[class_count * W * H] with class_count==0 collapsing to a
  // 4-byte dummy (Metal binding must be non-nil). Content is zeroed here (per
  // BeginSession), then re-zeroed by ReadbackClassLanes after every drain
  // window — mirrors the landed_weight_buf_ lifecycle.
  impl_->EnsureClassLaneBuf(impl_->width, impl_->height);

  // task-358.3: reset the per-session ray-mask capture accumulators (renamed
  // from component-mask; Fork-C ComponentTable retired).
  impl_->captured_masks_.clear();
  impl_->captured_ws_.clear();
}

LayerHandlePtr MetalTraceBackend::TraceLayer(const RootRaySource& roots) {
  assert(impl_->in_session && "TraceLayer outside session");
  assert(impl_->ms_idx < impl_->spec.scene->ms_.size() &&
         "TraceLayer beyond configured MS layers");
  @autoreleasepool {
    const auto& ms_info = impl_->spec.scene->ms_[impl_->ms_idx];
    assert(!ms_info.setting_.empty() && "MS layer has no scattering settings");
    bool first_ms = !roots.is_device;

    size_t total_ray_num = first_ms ? roots.host.count : roots.device.count;
    if (first_ms) {
      // root_ray_count is now maintained inside GenerateFirstLayerRootsForCi
      // (both host and device paths advance it) so device root-gen sees a
      // globally monotone gen_ray_base across ci slices and successive batches.
      // task-268.4 (commit-batch-decouple-drain): allocate the exit buffer
      // for a SINGLE layer's worst-case fan-out only. The simulator's
      // per-layer drain pattern (TraceLayer -> DrainExits -> Recombine)
      // recycles the same buffer across MS layers, so the prior
      // `per_layer_cap × num_ms` overallocation is no longer needed.
      // Grow-on-overflow inside the ci-loop retry below covers cases where
      // a single layer's actual fan-out exceeds the static ComputeOutCap
      // estimate (e.g. ms3_mixed_pyramid_heavy: ~169-211 exits/root vs.
      // ComputeOutCap's max_hits*2+4 bound). Tests that bypass DrainExits
      // (test_metal_trace_parity.cpp) rely on the same grow-on-overflow to
      // accommodate cumulative accumulation across layers.
      // S1 device-fused: exit buffers removed; no EnsureExitBuffers call needed.
      // landed_weight_buf_ is cleared per BeginSession (not per-layer).
    }
    // Recalculate out_cap per layer so each layer's continuation buffer is
    // sized to the actual fan-out from that layer's input count. This matters
    // for ≥3 MS layers where the fan-out from layer N can exceed the root-count
    // bound used by layer 0.
    impl_->out_cap = ComputeOutCap(total_ray_num, impl_->spec.scene->max_hits_);
    if (total_ray_num == 0) {
      return std::make_unique<MetalLayerHandle>(0u, LayerStats{});
    }

    // scrum-267 task-fused-emit-gate Step 3a: the prior multi-MS final-layer
    // regrow is now subsumed by the first_ms one-time pre-allocation above
    // (num_ms × ComputeOutCap covers final-layer fan-out + every mid-layer
    // emit-gate mid-exit). Keeping a mid-session EnsureExitBuffers here would
    // risk orphaning ms_mode==1 mid-exits already written by earlier layers.

    // Partition the layer's rays across crystal populations. Matches
    // simulator.cpp:572-586 and CpuTraceBackend::TraceLayer.
    size_t crystal_cnt = ms_info.setting_.size();
    std::vector<float> proportions;
    proportions.reserve(crystal_cnt);
    for (size_t ci = 0; ci < crystal_cnt; ci++) {
      proportions.push_back(ms_info.setting_[ci].crystal_proportion_);
    }
    std::vector<double> carry(crystal_cnt, 0.0);
    auto crystal_ray_num = PartitionCrystalRayNum(proportions, total_ray_num, carry);

    // Pre-allocate root_*_buf to the full per-layer total. Each per-ci
    // dispatch only uses crystal_ray_num[ci] of it; sizing for the upper
    // bound once avoids repeated grow-only allocations inside the ci loop
    // and costs nothing on M2 unified memory.
    impl_->EnsureRootBuffers(total_ray_num);

    bool last_layer = (impl_->ms_idx + 1u == impl_->spec.scene->ms_.size());
    uint32_t ms_mode = last_layer ? 0u : 1u;
    int out_slot = static_cast<int>(impl_->ms_idx & 1u);
    int in_slot = static_cast<int>((impl_->ms_idx - 1) & 1u);  // only used when !first_ms

    // scrum-258.3 Step 3 + scrum-267 Task 3 cleanup: per-layer filter+prob
    // state. Non-final layers populate `hop_ms_prob_[out_slot]` (consumed by
    // the device emit gate's `KernelParams.ms_prob` in DispatchLayer); the
    // final layer populates `last_layer_*` (consumed by ReadbackExitRays for
    // host filter+prob on the final-layer exits).
    if (last_layer) {
      impl_->last_layer_crystals_.resize(crystal_cnt);
      impl_->last_layer_axis_dists_.resize(crystal_cnt);
      impl_->last_layer_filter_configs_.resize(crystal_cnt);
      impl_->last_ms_prob_ = ms_info.prob_;
      impl_->last_ms_layer_idx_ = static_cast<uint8_t>(impl_->ms_idx);
    } else {
      impl_->hop_ms_prob_[out_slot] = ms_info.prob_;
    }

    // S1 device-fused: the grow-on-overflow retry loop is dead (no exit buffers).
    // The outer for(;;) { break; } structure is preserved so the ci_start / last_stats
    // / cont_counts reset that follows runs exactly once, matching the original flow.
    size_t ci_start = 0;
    for (;;) {
    ci_start = 0;
    // Reset per-layer counters BEFORE the ci loop: counter_buf is written
    // by each DispatchLayer (counter_init = previous cont_counts), and
    // last_stats accumulates via += inside DispatchLayer. Both must start
    // at zero for the layer.
    impl_->last_stats = LayerStats{};
    impl_->cont_counts[out_slot] = 0;

    // Layer-0 root rays must start with an all-zero carried color-class mask.
    // root_component_buf_ is reused across per-batch BeginSession cycles and the
    // trace kernel reads it on the production color path (transit_root_kernel
    // overwrites it for layer≥1), so this reset MUST be unconditional for the
    // first layer — NOT gated on the test-only capture path below — or batch N's
    // layer-0 rays inherit batch N-1's carried bits and color-class masks merge
    // across batches (observable only with multi-crystal L0 + multi-batch).
    if (first_ms && impl_->root_component_buf_ != nil) {
      std::memset([impl_->root_component_buf_ contents], 0, total_ray_num * sizeof(uint64_t));
    }
    // task-331.5: test-only capture-ring maintenance for the whole block below —
    // (re)allocate the capture buffers AND reset the ring counter so each layer's
    // emits append from 0 (the per-layer drain after the ci loop reads exactly
    // this layer's count). Entirely gated on the test-only capture path; a
    // disjoint buffer set from the production reset above.
    if (impl_->capture_ray_mask_) {
      impl_->EnsureComponentCaptureBuffers(impl_->out_cap);
      *static_cast<uint32_t*>([impl_->exit_comp_cnt_buf_ contents]) = 0u;
    }

    // code-review round 1 Major#2: running per-ci offset for the gen-attempt-
    // count sibling buffer (see GenerateFirstLayerRootsForCi / EncodeGenRoot).
    // Only meaningful on the first_ms/gen branch, but harmless to accumulate
    // unconditionally.
    size_t attempts_win_off = 0;
    for (size_t ci = 0; ci < crystal_cnt; ci++) {
      size_t ci_n = crystal_ray_num[ci];
      if (ci_n == 0) {
        continue;
      }
      const auto& setting = ms_info.setting_[ci];
      bool use_host = first_ms && ci == 0 && roots.host.crystal != nullptr;
      impl_->ResolveLayerCrystalForCi(setting, use_host,
                                       first_ms ? roots.host : HostRayBatch{});

      // scrum-258.3 Step 3 + scrum-267 Task 3 cleanup: final-layer only.
      // Non-final layers no longer need per-ci crystal/axis/filter state —
      // transit_root_kernel pulls geometry from tri_*_buf_ (uploaded by
      // ResolveLayerCrystalForCi → UploadCrystal) and the emit gate consumes
      // the device-side filter buffers (scrum-267 task-msl-filter-match-port).
      // ReadbackExitRays still runs host filter+prob on the final-layer exits,
      // so last_layer_* mirrors stay.
      if (last_layer) {
        impl_->last_layer_crystals_[ci] = impl_->current_crystal;
        impl_->last_layer_axis_dists_[ci] = setting.crystal_.axis_;
        impl_->last_layer_filter_configs_[ci] = setting.filter_;
      }

      size_t in_count = 0;
      if (first_ms) {
        // task-267.4 (continuation-validation) golden-ray hook: if the caller
        // supplied initial-ray buffers via HostRayBatch::d/p/w/tf, bypass both
        // host-mt19937 InitRayFirstMs and device PCG gen_root and feed those
        // analytically constructed rays straight into the trace kernel. Only
        // active at ci=0 (HostRayBatch carries a single crystal) and only
        // when the test path explicitly opted in (production simulator leaves
        // host.d == nullptr; see HostRayBatch design note in trace_backend.hpp).
        const bool inject_host = (ci == 0 && roots.host.d != nullptr);
        if (inject_host) {
          assert(use_host &&
                 "host-ray injection requires roots.host.crystal != nullptr");
          assert(roots.host.count == ci_n &&
                 "host-ray injection: host.count must match the ci=0 partition "
                 "(single-ci, single-crystal contract for golden tests)");
          in_count = impl_->InjectHostRoots(roots.host);
        } else {
          // task-260.2/260.7: device root-gen runs when
          //   (a) per-ci device-gen; each ci resolves a single crystal via
          //       ResolveLayerCrystalForCi before this guard, so crystal_cnt > 1
          //       is safe — per-ci multi-crystal parity verified at ds=0.9998
          //       (explore-260.3 exp2);
          //   (b) host-supplied root rays are NOT pinned via roots.host.crystal
          //       (matches the use_host=true convention below);
          //   (c) spec.seed != 0 (single-worker determinism contract); and
          //   (d) tri_count ≤ kMaxTriPerKernel (kernel stack-array bound); and
          //   (e) LUMICE_DISABLE_DEVICE_GEN env var is unset (escape hatch for
          //       strict-identity parity tests that mirror the host mt19937
          //       stream — these cannot align with the device PCG stream).
          // task-gpu-rng-ray-index-uint64: the pre-fix "root_ray_count fits in
          //   uint32" guard is GONE — the kernel now mixes the high 32 bits
          //   of the 64-bit ray-base into every ray's PCG seed via
          //   pcg_seed_with_high, so >2^32 sessions no longer collapse on
          //   wrap and there is no cap to enforce here.
          // The escape hatch is cached once per backend in Impl's ctor (see
          // task-260.5 Step 4); tests that need to flip it must setenv BEFORE
          // constructing a new MetalTraceBackend.
          bool can_use_device_gen = !use_host &&
                                    impl_->gen_seed_ != 0u &&
                                    impl_->current_crystal.TotalTriangles() <= 64u &&
                                    !impl_->disable_device_gen_;
          in_count = impl_->GenerateFirstLayerRootsForCi(setting, ci, ci_n, can_use_device_gen,
                                                          attempts_win_off);
        }
      } else {
        // scrum-267 task-device-resident-continuation Step 3: device frame-
        // transit. The prior layer's emit gate already filtered + prob-gated
        // every ray in cont_d/cont_w[in_slot] (task-fused-emit-gate), so all
        // ci_n rays propagate through transit and `in_count = ci_n`. Frame-
        // transit (orientation sample + ApplyInverse + entry-point sample)
        // now runs on-device, eliminating the per-ray host loop the prior
        // host frame-transit needed. The TotalTriangles() ≤ 64 guard
        // mirrors `can_use_device_gen` (kMaxTriPerKernel kernel stack-array
        // bound); canonical configs (hex column ~16 tri) never hit it.
        // task-268.7 Step 3: transit_root + trace_layer now share a single
        // command buffer per ci. Apple-Silicon shared-memory RAW visibility
        // across sequential compute encoders on one CB is verified (explore-
        // 263 gen+trace fusion corr 0.946 / task-264). The merge halves the
        // per-ci CPU-GPU sync point count on non-first MS layers (from 2 to 1).
        if (impl_->current_crystal.TotalTriangles() > 64u) {
          ILOG_ERROR(EffectiveLogger(impl_->logger_),
                     "transit_root_kernel: crystal ci={} tri_count={} exceeds kMaxTriPerKernel=64; ci skipped",
                     ci, impl_->current_crystal.TotalTriangles());
          ci_start += ci_n;
          continue;
        }
        // Monotone advance — mirrors gen_root's root_ray_count contract so
        // per-SimBatch transit dispatches on (layer,ci) consume disjoint PCG
        // ranges. task-gpu-rng-ray-index-uint64: the ray_base is passed through
        // to BuildTransitRootParams as the raw size_t; the split into
        // (gen_ray_base_lo, gen_ray_base_hi) happens inside so both host
        // gen/transit call sites share one splitting site.
        auto transit_gp = impl_->BuildTransitRootParams(
            setting, ci_n,
            static_cast<uint32_t>(impl_->ms_idx),
            static_cast<uint32_t>(ci),
            impl_->transit_ray_count_);
        id<MTLCommandBuffer> combined_cb = [impl_->queue commandBuffer];
        impl_->EncodeTransitRoot(combined_cb, transit_gp, in_slot, ci_start);
        // Advance unconditionally now: with the merged CB we no longer probe
        // transit success before issuing trace. A failed combined CB surfaces
        // via WaitAndReadbackLayer's status check (same recovery semantics as
        // gen+trace fusion — assert in debug, log+continue in release).
        impl_->transit_ray_count_ += ci_n;
        ci_start += ci_n;
        in_count = ci_n;  // all cont rays already filter+prob-passed by the
                          // prior layer's device emit gate; transit cannot
                          // drop rays.

        // counter_init = current cumulative write offset; ci=0 starts at 0,
        // each subsequent ci resumes where the previous one's atomic
        // counter left off (already read back into cont_counts[out_slot]
        // by the previous WaitAndReadbackLayer call).
        uint32_t counter_init_nf = static_cast<uint32_t>(impl_->cont_counts[out_slot]);
        impl_->DispatchLayer(in_count,
                             impl_->root_d_buf, impl_->root_p_buf,
                             impl_->root_w_buf, impl_->root_tf_buf,
                             ms_mode, out_slot, counter_init_nf,
                             static_cast<uint32_t>(ci),
                             static_cast<uint32_t>(impl_->ms_idx),
                             combined_cb);
        // task-268.7: drain the combined CB at the ci tail so cont_counts /
        // last_stats are populated before the next ci reads them as
        // counter_init. Single-ci configs hit this once per layer (still one
        // commit+wait pair, same wall-clock).
        impl_->WaitAndReadbackLayer();
        continue;
      }

      // first_ms (or single-MS) branch: DispatchLayer creates its own CB.
      uint32_t counter_init = static_cast<uint32_t>(impl_->cont_counts[out_slot]);
      impl_->DispatchLayer(in_count,
                           impl_->root_d_buf, impl_->root_p_buf,
                           impl_->root_w_buf, impl_->root_tf_buf,
                           ms_mode, out_slot, counter_init,
                           static_cast<uint32_t>(ci),
                           static_cast<uint32_t>(impl_->ms_idx));
      // task-268.7: in-loop drain mirrors the non-first MS branch. Single-ci
      // configs trigger this once per layer; the post-loop nil-check below is
      // a defensive safety net.
      impl_->WaitAndReadbackLayer();
      // ci_start was incremented above inside the !first_ms branch (before
      // the in_count==0 continue), so the next ci reads the correct slice
      // even if this ci's filter+prob dropped everything.
      attempts_win_off += ci_n;
    }  // ci-loop

    // task-268.7 safety net: WaitAndReadbackLayer is invoked at every ci tail,
    // so pending_cb_ is expected to be nil here. The call is a no-op when nil
    // (defensive against a future code path that skips the in-loop drain).
    impl_->WaitAndReadbackLayer();

    // task-331.5: drain this layer's component-mask capture ring into the
    // session accumulators (test-only). All trace CBs are already waited above,
    // so the counter + buffers are coherent. Clamp to capacity (the kernel's
    // `cslot < out_cap` guard drops overflow; assert-free like the cont ring).
    if (impl_->capture_ray_mask_) {
      uint32_t n_cap = *static_cast<uint32_t*>([impl_->exit_comp_cnt_buf_ contents]);
      size_t n = std::min<size_t>(n_cap, impl_->exit_comp_capacity_);
      const uint64_t* mptr = static_cast<const uint64_t*>([impl_->exit_comp_mask_buf_ contents]);
      const float*    wptr = static_cast<const float*>([impl_->exit_comp_w_buf_ contents]);
      impl_->captured_masks_.insert(impl_->captured_masks_.end(), mptr, mptr + n);
      impl_->captured_ws_.insert(impl_->captured_ws_.end(), wptr, wptr + n);
    }

    // S1 device-fused: exit_slot_buf is nil — exit records no longer
    // materialised; no overflow can occur. Break immediately.
    break;
    }  // grow-on-overflow retry (inert after S1 device-fused)

    size_t produced = last_layer ? 0u : impl_->cont_counts[out_slot];
    return std::make_unique<MetalLayerHandle>(produced, impl_->last_stats);
  }
}

RootRaySource MetalTraceBackend::Recombine(LayerHandlePtr handle, const RecombineSpec& spec) {
  assert(impl_->in_session);
  assert(handle != nullptr);
  size_t count = handle->ContinuationCount();

  // Continuation-pool decorrelation shuffle (task-gpu-backend-recombine-shuffle).
  // Mirrors legacy host Fisher-Yates (simulator.cpp:946-950): the per-CI trace
  // dispatches leave cont[written_slot] grouped by parent CI; without this
  // shuffle, the next layer's per-CI slicing hands parent-correlated subsets to
  // each child CI (explore-300 root cause).
  //
  // Slot accounting (mirrors TraceLayer's out_slot = ms_idx & 1 /
  // in_slot = (ms_idx-1) & 1 at metal_trace_backend.mm:2105-2106; F-verified):
  //   - This layer wrote cont[out_slot = ms_idx & 1]; we are BEFORE the ++ so
  //     written_slot below is exactly that out_slot.
  //   - The next layer N+1 reads in_slot = ((N+1)-1) & 1 = N & 1 = written_slot.
  //   - Gather READ cont[written_slot] (source), WRITE cont[other_slot] (dest),
  //     then SWAP the slot handles so cont[written_slot] holds the shuffled
  //     data the next layer consumes; cont[other_slot] post-swap holds the
  //     stale source the next layer overwrites (out_slot = (N+1) & 1) — no alias.
  if (spec.shuffle && count > 1u) {
    const int written_slot = static_cast<int>(impl_->ms_idx & 1u);
    const int other_slot   = 1 - written_slot;
    // Sync precondition: TraceLayer drained every trace CB via
    // WaitAndReadbackLayer ([cb waitUntilCompleted]) at each ci-tail before
    // returning the handle, so no CB is in flight against cont[other_slot] when
    // we realloc it below (CUDA's equivalent guard is the default-stream
    // cudaDeviceSynchronize before its grow path).
    // Grow cont[other_slot] to the current fan-out bound. EnsureContBuffer sizes
    // every slot to out_cap, and out_cap >= count by the fan-out invariant
    // (WaitAndReadbackLayer asserts produced <= out_cap), so this guarantees
    // capacity >= count. Assert makes the implicit invariant explicit, symmetric
    // with CUDA's explicit cont_cap_[other_slot] < cont_n check.
    impl_->EnsureContBuffer(other_slot);
    assert(impl_->cont_capacity[other_slot] >= count &&
           "shuffle: other_slot capacity must cover the continuation count");
    // Per-layer Feistel seed: spec.seed ^ nonce ^ ms_idx. Matches the CUDA side
    // (shuffle_seed_ ^ ms_layer_idx_, with shuffle_seed_ = spec.seed ^ nonce);
    // inlined here rather than cached since Recombine is called rarely.
    const uint32_t shuf_seed = impl_->spec.seed ^ kMetalShuffleNonce ^
                               static_cast<uint32_t>(impl_->ms_idx);
    id<MTLCommandBuffer> shuf_cb = [impl_->queue commandBuffer];
    impl_->EncodeShuffleCont(shuf_cb, written_slot, other_slot,
                             static_cast<uint32_t>(count), shuf_seed);
    [shuf_cb commit];
    [shuf_cb waitUntilCompleted];
    // Swap ALL parallel slot arrays so cont[written_slot] holds the shuffled
    // data. Omitting cont_wl_idx_buf_ would silently desynchronize per-ray
    // wavelength tagging — Feistel / energy parity could not detect it.
    std::swap(impl_->cont_d[written_slot],          impl_->cont_d[other_slot]);
    std::swap(impl_->cont_w[written_slot],          impl_->cont_w[other_slot]);
    std::swap(impl_->cont_wl_idx_buf_[written_slot], impl_->cont_wl_idx_buf_[other_slot]);
    // task-331.5 (LANDMINE): the component mask is a parallel per-ray array — it
    // MUST swap with cont_d/cont_w/cont_wl_idx here, mirroring CPU
    // RayBuffer::SwapRay. Omitting it silently decorrelates the OR-accumulated
    // mask from its ray across the layer boundary (breaks the joint distribution
    // while preserving per-component marginals — invisible to a corr/marginal
    // check, caught only by the per-mask-value parity assertion).
    std::swap(impl_->cont_component_buf_[written_slot], impl_->cont_component_buf_[other_slot]);
    std::swap(impl_->cont_capacity[written_slot],   impl_->cont_capacity[other_slot]);
  }

  impl_->ms_idx++;

  DeviceRayBatch dev;
  // backend_ptr is an opaque token (we use `this` so callers can sanity-check
  // it against the producing backend; the actual buffers live in Impl::cont_*).
  dev.backend_ptr = this;
  dev.count = count;
  return RootRaySource::FromDevice(dev);
}

void MetalTraceBackend::ReadbackImage(XyzImageData& out) {
  assert(impl_->in_session);
  assert(out.data != nullptr);
  assert(out.width == impl_->width && out.height == impl_->height &&
         "XyzImageData dimensions must match BeginSession resolution");
  size_t pix = static_cast<size_t>(impl_->width) * static_cast<size_t>(impl_->height);
  std::memcpy(out.data, [impl_->xyz_image contents], pix * 3 * sizeof(float));
}

void MetalTraceBackend::SetCaptureRayMask(bool enable) {
  // Must be set before BeginSession — the capture-ring sizing + per-layer
  // memset/reset in TraceLayer read this flag.
  impl_->capture_ray_mask_ = enable;
}

void MetalTraceBackend::ReadbackRayMask(std::vector<uint64_t>& masks,
                                        std::vector<float>& weights) const {
  masks = impl_->captured_masks_;
  weights = impl_->captured_ws_;
}

// S1 device-fused: ReadbackExitRays returns empty — the kernel no longer
// materialises per-exit records. Filter+prob+projection now run on device
// inside the emit gate; XYZ accumulation goes directly into xyz_image.
// DrainExits delegates here and likewise returns 0; the simulator's
// per-layer drain loop receives an empty vector and skips ScatterOutgoingToXyz.
size_t MetalTraceBackend::ReadbackExitRays(std::vector<ExitRayRecord>& out) {
  assert(impl_->in_session);
  out.clear();
  return 0;
}

// scrum-312 third-clock drain: copy the PERSISTENT cross-batch xyz_image + landed
// weight to host and reset them for the next window. The simulator drains on
// display cadence (a whole window of batches), possibly BETWEEN sessions — so
// this must not depend on in_session/width/height (Reset clears those), and must
// itself guarantee GPU completion rather than rely on the caller having waited.
void MetalTraceBackend::ReadbackXyzAccum(XyzImageData& xyz, float& landed_weight) {
  // Release-safe gates (mirror the CUDA backend; asserts are no-ops under NDEBUG,
  // review-Minor: all three preconditions throw, not assert).
  if (impl_->xyz_image == nil || impl_->landed_weight_buf_ == nil || xyz.data == nullptr) {
    throw BackendUnavailableError("MetalTraceBackend::ReadbackXyzAccum: unallocated buffer or null xyz.data");
  }
  // Invariant (review-Minor): alloc_xyz_w_/h_ and xyz_pix_capacity encode the same
  // underlying allocation and are set together in EnsureImage — assert they agree.
  assert(static_cast<size_t>(impl_->alloc_xyz_w_) * static_cast<size_t>(impl_->alloc_xyz_h_) ==
             impl_->xyz_pix_capacity &&
         "alloc dims out of sync with xyz_pix_capacity");
  // Pixel count from the dims the buffer was ACTUALLY allocated for (persist
  // across Reset), cross-checked release-safe against the caller's declared dims.
  if (xyz.width != impl_->alloc_xyz_w_ || xyz.height != impl_->alloc_xyz_h_) {
    throw BackendUnavailableError("MetalTraceBackend::ReadbackXyzAccum: caller dims (" + std::to_string(xyz.width) +
                                  "x" + std::to_string(xyz.height) + ") != allocated buffer dims (" +
                                  std::to_string(impl_->alloc_xyz_w_) + "x" + std::to_string(impl_->alloc_xyz_h_) + ")");
  }
  // Defensive GPU-completion barrier: the per-batch WaitAndReadbackLayer (ci-loop
  // tail) + EndSession already drain pending_cb_, so this is normally nil, but a
  // between-session drain must not assume that — mirror CUDA's cudaDeviceSynchronize.
  // Thread-safety (review-Minor): pending_cb_ is only ever touched on the simulator
  // Run() thread — the trace (DispatchLayer sets it), the per-batch waits, and this
  // drain (via DrainDeviceXyz at Run()'s flush points / batch-cap) all run on that
  // single thread sequentially. No cross-thread access, so the bare-pointer r/w is
  // race-free (the display-cadence "clock" is the Run loop's own control flow, not a
  // separate GUI thread).
  if (impl_->pending_cb_ != nil) {
    [impl_->pending_cb_ waitUntilCompleted];
    impl_->pending_cb_ = nil;
  }
  size_t pix = static_cast<size_t>(impl_->alloc_xyz_w_) * static_cast<size_t>(impl_->alloc_xyz_h_);
  std::memcpy(xyz.data, [impl_->xyz_image contents], pix * 3 * sizeof(float));
  landed_weight += *static_cast<const float*>([impl_->landed_weight_buf_ contents]);
  // Reset the accumulators so the next drain window starts from zero (BeginSession
  // no longer clears them). Unified memory → a plain host memset/store suffices.
  std::memset([impl_->xyz_image contents], 0, pix * 3 * sizeof(float));
  *static_cast<float*>([impl_->landed_weight_buf_ contents]) = 0.0f;
}

// task-358.1 Step 4 (AC3 device-side Y-lane accumulation): drain the flattened
// per-color-class Y accumulator into `lane_data` and reset the device side for
// the next window. Called by Simulator::DrainDeviceXyz right after
// ReadbackXyzAccum, so any pending command buffer is already flushed. Layout
// (matches the MSL kernel's write side): lane_data[c * W*H + (py*W+px)].
//
// When class_count_==0 this is a no-op (empty vector, class_count=0) — the
// consumer's ConsumeDeviceFused stays byte-identical to pre-358.1 (AC4).
void MetalTraceBackend::ReadbackClassLanes(std::vector<float>& lane_data, size_t& class_count) {
  class_count = impl_->class_count_;
  if (impl_->class_count_ == 0 || impl_->class_lane_buf_ == nil) {
    lane_data.clear();
    return;
  }
  const size_t pix = static_cast<size_t>(impl_->alloc_xyz_w_) *
                     static_cast<size_t>(impl_->alloc_xyz_h_);
  assert(pix > 0);
  const size_t total = impl_->class_count_ * pix;
  // Runtime gate (not just the assert below, which -DNDEBUG release builds
  // compile out): alloc_xyz_w_/alloc_xyz_h_ (set by EnsureImage) and the w/h
  // EnsureClassLaneBuf sized class_lane_buf_ against (impl_->width/height)
  // are both driven from the same BeginSession snapshot with no intervening
  // resize path, so `total` should never exceed class_lane_pix_capacity_ in
  // practice — but if that invariant is ever broken by a future change,
  // degrade to an empty drain rather than reading past the allocation
  // (code-review-01 Minor #3, same failure class as the render.cpp Major).
  if (total > impl_->class_lane_pix_capacity_) {
    ILOG_ERROR(EffectiveLogger(impl_->logger_),
               "MetalTraceBackend::ReadbackClassLanes: class_lane_buf_ under-allocated "
               "(need {} floats, have {}) — alloc_xyz_w_/h_ and width/height diverged. "
               "Dropping this window's per-class lane drain.",
               total, impl_->class_lane_pix_capacity_);
    assert(false && "class_lane_buf_ under-allocated for the current class_count * W * H");
    lane_data.clear();
    class_count = 0;
    return;
  }
  lane_data.resize(total);
  std::memcpy(lane_data.data(), [impl_->class_lane_buf_ contents], total * sizeof(float));
  // Reset the device accumulator so the next window starts clean (mirrors
  // ReadbackXyzAccum's post-copy zero on xyz_image / landed_weight_buf_).
  std::memset([impl_->class_lane_buf_ contents], 0, total * sizeof(float));
}

// task-268.4 per-layer destructive drain. Identical to ReadbackExitRays
// except for the post-step that zeroes the device atomic slot counter so the
// exit_*_buf storage can be recycled by the next TraceLayer. Pairs with the
// simulator's per-layer drain loop (SimulateOneWavelengthWithBackend) and the
// grow-on-overflow retry inside TraceLayer.
size_t MetalTraceBackend::DrainExits(std::vector<ExitRayRecord>& out) {
  size_t n = ReadbackExitRays(out);
  if (impl_->exit_slot_buf != nil) {
    *static_cast<uint32_t*>([impl_->exit_slot_buf contents]) = 0u;
  }
  return n;
}

void MetalTraceBackend::EndSession() {
  impl_->Reset();
}

bool MetalTraceBackend::IsCompatible(const RenderConfig& render) const {
  // 315.3: the kernel exit tail projects via lm_proj::ProjectExitToPixel — the
  // SAME single source as the CPU parity oracle (scatter_accum.hpp) — so every
  // forward projection (single-lens linear/fisheye, rectangular, dual-fisheye
  // variants, and globe) is byte-identical to legacy CPU at any view/fov. The
  // trace geometry is projection-independent, so no per-type view constraint
  // remains — all lens types are supported on device.
  (void)render;
  return true;
}

uint32_t MetalTraceBackend::WlPoolSize() const {
  // scrum-268.8 (DR-3): report the *capability* — the configured pool size is
  // known from the env/default before BeginSession allocates the buffer. The
  // simulator queries this before the first trace to decide whether to hand the
  // backend a per-ray (zero-wl) WlParam, so returning 0 pre-session would make
  // it fall back to per-batch wl even on Metal. After BeginSession this equals
  // the allocated size (identical value).
  return impl_->wl_pool_size_ != 0u ? impl_->wl_pool_size_ : ResolveWlPoolSize(EffectiveLogger(impl_->logger_));
}

size_t MetalTraceBackend::GetLastBatchCrystalCount() const {
  // task-exit-seam-crystal-count: Impl::last_layer_crystals_ is resized/filled
  // only during the final layer's TraceLayer call (see the block at
  // metal_trace_backend.mm:2127-2168); reading before EndSession is safe.
  return impl_->last_layer_crystals_.size();
}

size_t MetalTraceBackend::TraceLayerKernelMaxThreadsForTest() const {
  if (impl_->pso == nil) {
    return 0;
  }
  return static_cast<size_t>(impl_->pso.maxTotalThreadsPerThreadgroup);
}

const void* MetalTraceBackend::GetDevicePtrForTest() const {
  return (__bridge const void*)impl_->device;
}

const void* MetalTraceBackend::GetPsoPtrForTest() const {
  return (__bridge const void*)impl_->pso;
}

// --- scrum-328.2 Step 3: MetalTraceBackendTestHooks method definitions ---
// Semantics identical to the pre-scrum-328.2 `MetalTraceBackend::XxxForTest`
// bodies; only the access seam moved (via `backend_.impl_->...`). Live in this
// .mm TU so Impl's complete definition (an Obj-C++ struct with id<MTL...>
// members) is visible.

void MetalTraceBackendTestHooks::SetInitialRayBase(size_t root_base, size_t transit_base) {
  auto& impl = *backend_.impl_;
  impl.root_ray_count = root_base;
  impl.transit_ray_count_ = transit_base;
}

size_t MetalTraceBackendTestHooks::ReadbackRootRot(std::vector<float>& out, size_t count) {
  // [TEST-ONLY] task-gpu-rng-ray-index-uint64: copy the first `count` per-ray
  // crystal→world rotation matrices (root_rot_buf, 9 floats/ray) back to host.
  // For a continuation (transit) layer these are the transit kernel's sampled
  // orientations R(tid) = build_crystal_rotation(sample_lat_lon_roll(
  // transit_mixed_seed, tid)) — a pure function of (transit_seed, tid), so
  // independent of ray physics and the atomic continuation compaction (unlike
  // root_d = R^-1·cont_dir, which mixes in whichever continuation ray landed at
  // this tid). This isolates the transit hi wiring: a non-zero transit hi moves
  // R(tid) at every tid; hi==0 runs are bit-identical. Metal's unified memory
  // makes this a plain contents() read (no metallib kernel change needed).
  auto& impl = *backend_.impl_;
  if (impl.root_rot_buf == nil || count == 0u) {
    out.clear();
    return 0u;
  }
  // Bound the copy to the buffer so a count larger than the allocation cannot
  // read past root_rot_buf; the caller's ASSERT_EQ then surfaces the truncation.
  const size_t cap_floats = static_cast<size_t>([impl.root_rot_buf length]) / sizeof(float);
  size_t n_floats = 9u * count;
  if (n_floats > cap_floats) {
    n_floats = cap_floats;
  }
  out.assign(n_floats, 0.0f);
  std::memcpy(out.data(), [impl.root_rot_buf contents], n_floats * sizeof(float));
  return n_floats;
}

size_t MetalTraceBackendTestHooks::ReadbackGenDirs(std::vector<float>& out, size_t count) {
  // [TEST-ONLY] scrum-328.2 Step 2: mirror of CUDA. root_d_buf is
  // MTLResourceStorageModeShared (unified memory) so this is a plain memcpy —
  // no kernel change needed.
  auto& impl = *backend_.impl_;
  if (impl.root_d_buf == nil || count == 0u) {
    out.clear();
    return 0u;
  }
  const size_t cap_floats = static_cast<size_t>([impl.root_d_buf length]) / sizeof(float);
  size_t n_floats = 3u * count;
  if (n_floats > cap_floats) {
    n_floats = cap_floats;
  }
  out.assign(n_floats, 0.0f);
  std::memcpy(out.data(), [impl.root_d_buf contents], n_floats * sizeof(float));
  return n_floats;
}

void MetalTraceBackendTestHooks::EnableGenAttemptCount(size_t count, size_t ci_start) {
  // scrum-328.2 Step 1 Metal-symmetric enable: grow the always-bound sibling
  // buffer to hold `count` ints, arm the write flag. Independent of the RNG
  // probe (Metal currently has no `EnableRngProbeForTest`; see hpp §"2. 范围
  // 与边界" declaration for why).
  //
  // code-review round 1 Major#2: `count` is the layer's total ray count — the
  // ci-loop's gen dispatch writes at tid + ci_start + <running per-ci
  // offset>, up to ci_start + count total across all ci's in the layer — so
  // the allocation must cover ci_start + count, not just count.
  auto& impl = *backend_.impl_;
  const size_t required_bytes = std::max<size_t>(ci_start + count, 1u) * sizeof(int);
  const size_t current_bytes = (impl.lat_attempts_buf_ == nil)
                                   ? 0u
                                   : static_cast<size_t>([impl.lat_attempts_buf_ length]);
  if (current_bytes < required_bytes) {
    impl.lat_attempts_buf_ =
        [impl.device newBufferWithLength:required_bytes options:MTLResourceStorageModeShared];
    assert(impl.lat_attempts_buf_ != nil);
  }
  // Zero the used region so a stale count from a prior Enable does not leak.
  std::memset([impl.lat_attempts_buf_ contents], 0, required_bytes);
  impl.lat_attempts_cap_ = ci_start + count;
  impl.lat_attempts_ci_start_ = ci_start;
  impl.attempts_enabled_ = (count > 0u);
}

size_t MetalTraceBackendTestHooks::ReadbackGenAttemptCount(std::vector<int>& out, size_t count) {
  auto& impl = *backend_.impl_;
  if (count > impl.lat_attempts_cap_) {
    count = impl.lat_attempts_cap_;
  }
  if (impl.lat_attempts_buf_ == nil || count == 0u) {
    out.clear();
    return 0u;
  }
  out.assign(count, 0);
  std::memcpy(out.data(), [impl.lat_attempts_buf_ contents], count * sizeof(int));
  return count;
}

}  // namespace lumice
