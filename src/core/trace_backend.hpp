#ifndef CORE_TRACE_BACKEND_H_
#define CORE_TRACE_BACKEND_H_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "config/proj_config.hpp"
#include "config/render_config.hpp"
#include "core/crystal.hpp"
#include "core/def.hpp"
#include "core/exit_seam.hpp"

namespace lumice {

// =============================================================================
// TraceBackend — session-scoped backend abstraction for the ray-tracing pipeline.
//
// Purpose
//   A coarse-grained seam under which alternative implementations (CPU / Metal /
//   CUDA) can plug in without leaking kernel-level details to upper layers.
//
// Design invariants (do not relax without revisiting GPU performance assumptions)
//
//   1) Coarse fusion. `TraceLayer` fuses the entire trace -> recorder ->
//      projection -> XYZ-accumulation pipeline for a single MS layer in one
//      call. The seam DOES NOT expose individual kernels or per-stage
//      dispatches. Fine-grained dispatches would force intermediate state to
//      DRAM and forfeit register-resident-fusion throughput on GPU backends.
//
//   2) Host pointers do not cross the seam, except at two explicit boundaries:
//        - HostRayBatch: the ingest boundary (only at the first MS layer).
//        - ReadbackExitRays: the sole device->host boundary (exit-ray
//          buffer-egress — N `ExitRayRecord`s, each 36B carrying
//          {dir, weight, path, crystal_id, ms_layer_idx}).
//      Ray data and layer-local buffers otherwise live as backend-owned,
//      opaque, device-resident handles. (scrum-258.1: ReadbackImage was
//      retired from this seam; CPU/Metal still expose it as a non-virtual
//      concrete helper for the parity harness only.)
//
//   3) Discrete-memory semantics. Even when the underlying memory is unified
//      (CPU heap, Apple M2 unified memory), the contract is written as if a
//      PCIe transfer separates host from device. A CUDA backend must be able
//      to implement this interface without any change to upper layers:
//        - HostRayBatch -> cudaMemcpyAsync(host, device, ...) inside TraceLayer
//          (or stashed in BeginSession state, depending on backend).
//        - DeviceRayBatch::backend_ptr binds to a cudaMalloc-allocated pointer.
//        - LayerHandle wraps backend-specific device-resident state.
//        - Recombine maps to a stream-compaction kernel (thrust or hand-written).
//          The only host-visible side-effect is reading ContinuationCount() —
//          a single 4-byte cudaMemcpy.
//        - ReadbackExitRays performs a single device->host copy of
//          N `ExitRayRecord`s (36B each) for N exit rays.
//
//   4) Metal co-design as a second orthogonal view. The contract is validated
//      by at least one Metal skeleton implementation so the seam does not
//      ossify around unified-memory assumptions that the CPU backend alone
//      could not detect (CPU + M2 Metal are both unified-memory — not
//      orthogonal). See `core/metal_trace_backend.hpp`.
//
//   5) Public C API stability. The backend split is an internal-to-core
//      refactor. `src/include/lumice.h` is unchanged; `src/gui/` and
//      `src/server/` are unaware of which backend is in use.
//
//   6) Ray-frame lifecycle. Rays crossing the seam are ALWAYS in world space:
//        - TraceLayer's input rays (HostRayBatch ingest or device continuation)
//          and any ray that LEAVES a backend are world-space.
//        - The crystal-local frame is a backend-internal detail. A backend may
//          transform world->local on ingest and MUST transform local->world
//          before a ray leaves it (projection, continuation, or readback). The
//          CPU backend already models this (InitRay* applies the inverse crystal
//          rotation on ingest; CollectData re-applies it before projection).
//        - Per-ray crystal orientation NEVER crosses the seam: each backend
//          samples and owns its own per-ray rotations internally. How they are
//          supplied (host-sampled upload vs device-side RNG) is a backend
//          performance choice; this invariant is supply-agnostic.
//      This is a DOCUMENTATION CONTRACT, not an automated gate — comments can
//      drift. Its closure is procedural: a future CUDA backend's plan-review
//      MUST list "exit rays returned to world space before leaving the kernel"
//      as a mandatory check. (Metal once shipped this bug: exit directions were
//      projected in crystal-local space, scattering the 22° halo into a band.)
//      As-built lifecycle (host/kernel transform points, multi-MS transit,
//      parity harness): see doc/trace-backend-frame-lifecycle.md.
//
// State machine
//   Legal call sequence (per backend instance):
//
//     BeginSession                                          \
//       (TraceLayer -> DrainExits -> Recombine)* n           | session
//       TraceLayer -> DrainExits          // final layer     |
//     EndSession                                            /
//
//   - `TraceLayer` may be invoked one or more times per session.
//   - `Recombine` consumes a LayerHandle; it must be paired with a preceding
//     TraceLayer call. The very last MS layer typically skips Recombine since
//     no further layer follows.
//   - `DrainExits` is the canonical per-layer egress (task-268.4). It is
//     DESTRUCTIVE: copies the device-side exit buffer into the caller's
//     vector AND resets the device counter so the buffer can be recycled
//     across MS layers (produce -> drain -> recycle). Single-MS callers
//     invoke it once after the only TraceLayer; multi-MS callers invoke it
//     between TraceLayer and Recombine for every layer.
//   - `ReadbackExitRays` predates DrainExits (scrum-258 exit seam). On
//     CpuTraceBackend it is destructive (same semantics as DrainExits); on
//     MetalTraceBackend it is non-destructive (does NOT reset the device
//     slot). Retained for parity-harness tests that read the cumulative
//     session-level egress; production simulator code SHOULD use DrainExits.
//   - Calling any method outside the BeginSession/EndSession bracket — or
//     interleaving sessions on the same backend instance — is undefined
//     behaviour.
//
// Lifetime contracts (mandatory)
//   - SessionSpec::scene / SessionSpec::render: backend stores a non-owning
//     pointer in BeginSession and accesses it throughout the session. Caller
//     MUST keep both objects alive until EndSession() returns. Backends do
//     NOT copy SceneConfig / RenderConfig.
//   - HostRayBatch buffers (d / p / w / tf): valid only during the
//     TraceLayer call that consumes them. Backends may copy or upload as
//     needed; they MUST NOT retain the host pointers beyond the call.
//   - LayerHandle: owned by the caller via LayerHandlePtr. Destroying a
//     LayerHandle releases any backend-local layer state.
//   - DeviceRayBatch::backend_ptr (returned by Recombine): valid from the
//     point Recombine returns until either (a) the NEXT Recombine on the same
//     backend instance, or (b) EndSession() returns — whichever comes first.
//     The caller MUST pass the returned RootRaySource into the very next
//     TraceLayer call and MUST NOT retain references to backend_ptr across a
//     subsequent Recombine boundary. GPU backends therefore need only a
//     single device-resident continuation buffer per session (no
//     multi-generation lifetime tracking).
//
// =============================================================================

// -----------------------------------------------------------------------------
// BackendUnavailableError — thrown by a backend when its runtime prerequisites
// cannot be satisfied (Metal kernel compile / entry-point lookup / PSO build
// failure on macOS 26.5; CUDA driver mismatch; etc.). Distinct from generic
// std::runtime_error so the simulator can route it to a CPU-backend fallback
// instead of bubbling it to the server as a hard failure. BeginSession is the
// only contract point that may throw this — callers are required to catch by
// type and downgrade their backend selection (drop the backend instance for
// the remainder of the Run()).
// -----------------------------------------------------------------------------
class BackendUnavailableError : public std::runtime_error {
 public:
  explicit BackendUnavailableError(const std::string& what) : std::runtime_error(what) {}
};

// -----------------------------------------------------------------------------
// SessionSpec — parameters for a single trace session.
//
// Non-owning pointers; lifetime contract is documented above.
// -----------------------------------------------------------------------------
struct SessionSpec {
  const SceneConfig* scene;    // crystal / light source / ms[] / max_hits
  const RenderConfig* render;  // lens / resolution / view pose
  WlParam wl;                  // wavelength + spectral weight
  uint32_t seed;               // 0 = non-deterministic
};

// -----------------------------------------------------------------------------
// HostRayBatch — host-side initial-ray ingest (first MS layer only).
//
// All pointers are caller-owned and remain valid only during the single
// TraceLayer call that receives this batch.
// -----------------------------------------------------------------------------
// Design note: d / p / w / tf are the reserved "external ray ingest" path.
// Current CpuTraceBackend and MetalTraceBackend skeleton do NOT consume these
// fields — they generate initial rays internally from the scene config via
// InitRayFirstMs. These fields will be wired in a future integration task once
// Simulator routes its pre-sampled rays through the seam. Until then, callers
// SHOULD leave d/p/w/tf null/zero. (crystal / refractive_index / crystal_id
// ARE consumed by CpuTraceBackend when crystal != nullptr.)
struct HostRayBatch {
  size_t count = 0;
  const float* d = nullptr;          // 3 * count, world-space direction (invariant 6).
  const float* p = nullptr;          // 3 * count, entry point.
  const float* w = nullptr;          // count, ray weight (spectral).
  const IdType* tf = nullptr;        // count, ingoing polygon-face id.
  const Crystal* crystal = nullptr;  // crystal geometry for this batch.
  float refractive_index = 0.0f;
  size_t crystal_id = 0;  // index into session-level crystal list.
};

// -----------------------------------------------------------------------------
// DeviceRayBatch — opaque device-resident handle for continuation rays.
//
// backend_ptr is a backend-specific pointer (CPU = RayBuffer*, Metal =
// id<MTLBuffer>, CUDA = device pointer). Lifetime: see contract above
// (next Recombine OR EndSession).
// -----------------------------------------------------------------------------
struct DeviceRayBatch {
  void* backend_ptr = nullptr;
  size_t count = 0;
};

// -----------------------------------------------------------------------------
// RootRaySource — tagged union of "host ingest" vs "device continuation".
//
// The first MS layer must be FromHost; subsequent layers must be FromDevice
// (the value returned by the preceding Recombine).
// -----------------------------------------------------------------------------
struct RootRaySource {
  bool is_device = false;
  HostRayBatch host{};
  DeviceRayBatch device{};

  static RootRaySource FromHost(const HostRayBatch& h) {
    RootRaySource r;
    r.is_device = false;
    r.host = h;
    return r;
  }
  static RootRaySource FromDevice(const DeviceRayBatch& d) {
    RootRaySource r;
    r.is_device = true;
    r.device = d;
    return r;
  }
};

// -----------------------------------------------------------------------------
// LayerStats — aggregate exit-ray statistics for the just-traced MS layer.
//
// Used by the parity harness (CPU-vs-Metal numeric oracle) to cross-check
// device compaction integrity: a counter alone is insufficient (silent
// out_cap overflow may still report N rays produced while truncating the
// last fraction), so the weight sum is paired with the count.
//
// Semantics across backends:
//   - exit_count : number of rays that left the crystal in this layer. In
//     append (non-final) mode, equals the number of rays written to the
//     continuation buffer; in accumulate (final) mode, equals the number of
//     rays that contributed to the XYZ image (before projection clipping).
//   - exit_w_sum : sum of `w_` over those same rays.
//
// Backends MUST track these for every TraceLayer call. Float accumulation
// is acceptable (Metal MSL has no double atomic); the CPU oracle's
// per-layer relative comparison aims at rel ≤ 5e-4.
// -----------------------------------------------------------------------------
struct LayerStats {
  size_t exit_count = 0;
  float exit_w_sum = 0.0f;
};

// -----------------------------------------------------------------------------
// LayerHandle — opaque device-resident result of TraceLayer.
//
// Holds backend-local state for the just-traced MS layer (continuation rays
// + any per-layer device buffers). Move-only; passed into Recombine which
// consumes it.
// -----------------------------------------------------------------------------
class LayerHandle {
 public:
  virtual ~LayerHandle() = default;

  // Number of continuation rays produced by this layer (rays that survived
  // filter-pass + prob-pass and should feed the next MS layer).
  virtual size_t ContinuationCount() const = 0;

  // Aggregate exit-ray stats for this layer. See LayerStats above.
  // Intended consumer: CPU-vs-Metal parity harness (test_metal_trace_parity.cpp).
  // No production code path calls this. Default implementation returns a
  // zero-filled LayerStats{}; backends that support parity testing override it.
  virtual LayerStats GetLayerStats() const { return LayerStats{}; }

  LayerHandle(const LayerHandle&) = delete;
  LayerHandle(LayerHandle&&) = delete;
  LayerHandle& operator=(const LayerHandle&) = delete;
  LayerHandle& operator=(LayerHandle&&) = delete;

 protected:
  LayerHandle() = default;
};

using LayerHandlePtr = std::unique_ptr<LayerHandle>;

// -----------------------------------------------------------------------------
// RecombineSpec — declarative control for the inter-layer compaction stage.
//
// `shuffle`: for CPU backends, applies a Fisher-Yates shuffle to the
// continuation set so per-crystal-batch ordering does not bias the next
// layer's small-batch dispatch. GPU stream compaction is naturally unordered;
// backends may interpret `shuffle` as a no-op when compaction already breaks
// any locality assumptions.
// -----------------------------------------------------------------------------
struct RecombineSpec {
  bool shuffle = true;
};

// -----------------------------------------------------------------------------
// XyzImageData — host-side XYZ image (target of ReadbackImage).
//
// `data` is caller-owned; backend writes width * height * 3 floats.
// -----------------------------------------------------------------------------
struct XyzImageData {
  float* data = nullptr;
  int width = 0;
  int height = 0;
};

// -----------------------------------------------------------------------------
// TraceBackend — the seam.
// -----------------------------------------------------------------------------
class TraceBackend {
 public:
  virtual ~TraceBackend() = default;

  // Open a session. The backend captures non-owning pointers to spec.scene
  // and spec.render; the caller MUST keep them alive until EndSession()
  // returns.
  virtual void BeginSession(const SessionSpec& spec) = 0;

  // Trace one MS layer. Fuses trace -> recorder -> projection -> XYZ
  // accumulation. The returned LayerHandle is opaque; pass it into
  // Recombine() (or destroy it to discard the layer).
  //
  // The first call in a session MUST receive a host-mode RootRaySource and
  // the backend captures the host count as the session-level root_ray_count
  // (used for normalization). Subsequent calls in the same session MUST
  // receive a device-mode RootRaySource produced by the preceding Recombine.
  virtual LayerHandlePtr TraceLayer(const RootRaySource& roots) = 0;

  // Compact the continuation rays produced by a layer into a device-resident
  // batch usable as input to the next TraceLayer. Consumes the handle.
  // See "Lifetime contracts" above for the device_batch.backend_ptr lifetime.
  virtual RootRaySource Recombine(LayerHandlePtr handle, const RecombineSpec& spec) = 0;

  // Buffer-egress boundary (exit seam, scrum-258) — the canonical exit-ray
  // contract. Copies the world-space exit rays captured this session
  // (one entry per ray that left the crystal in the final MS layer) into
  // the caller-provided vector and returns the exit-ray count. Each record
  // is a 36B `ExitRayRecord` carrying {dir, weight, path, crystal_id,
  // ms_layer_idx}; see core/exit_seam.hpp. The simulator routes these
  // through the legacy consumer projection (O(exit rays)), replacing the
  // per-batch O(W*H) image readback.
  //
  // Structure note (scrum-258.2): the rich record replaces the prior
  // {dir, weight} pair (scrum-258.1). 258.2 only PRODUCES metadata; 258.3
  // will CONSUME path/crystal_id for filter + symmetry fold.
  //
  // Default returns 0 for partial or stub backends; production backends
  // (Cpu, Metal) should override. SILENT-ZERO FAILURE MODE: a backend that
  // forgets to override will produce 0 exit rays per batch — the simulator
  // discards the batch (exit_count == 0 early-return), yielding a black image
  // with no compile-time or run-time error. New backend authors must override.
  //
  // Replaces the prior ReadbackImage seam (scrum-258.1 Step 5): the
  // exit-seam payload is the canonical out path; ReadbackImage was demoted
  // to a non-virtual, concrete CPU/Metal helper used only by the parity
  // harness (test 接缝 与 生产契约 结构性隔离).
  virtual size_t ReadbackExitRays(std::vector<ExitRayRecord>& out) {
    out.clear();
    return 0;
  }

  // Per-layer DESTRUCTIVE drain (task-268.4 commit-batch decoupling).
  //
  // Copies the device-side exit buffer into `out` and resets the device
  // accumulator so the buffer can be recycled across MS layers
  // (produce -> drain -> recycle). Intended to be called after every
  // TraceLayer in multi-MS sessions; single-MS sessions call it once after
  // the only TraceLayer.
  //
  // Contrast with ReadbackExitRays (scrum-258): Readback's contract was
  // session-level egress (cumulative across layers, idempotent on Metal —
  // does not reset the slot counter). Drain is per-layer egress: each call
  // returns only what was emitted since the previous Drain (or BeginSession)
  // and atomically clears the counter so the next TraceLayer writes into
  // slot 0 again. This is the seam-level mechanism that lets a multi-MS
  // session keep a SINGLE-layer-sized exit buffer (vs. num_ms × per_layer
  // in the pre-268.4 scheme), and is the foundation for grow-on-overflow
  // when a single layer's fan-out exceeds the static cap.
  //
  // Overflow handling: backends MUST detect produced > capacity and grow
  // internally before returning; the returned vector MUST contain every
  // emitted record (no silent clamp). The pre-268.4 ReadbackExitRays
  // "produced > capacity → log + clamp" path is RETIRED on the Drain
  // contract. Default returns 0 for stub backends.
  virtual size_t DrainExits(std::vector<ExitRayRecord>& out) {
    out.clear();
    return 0;
  }

  // Close the session. Releases per-session backend state. Calling any
  // method other than BeginSession after EndSession is undefined.
  virtual void EndSession() = 0;

  // Returns false if this backend cannot handle the given render config.
  // Each backend self-describes its constraints, keeping the base class backend-agnostic.
  virtual bool IsCompatible(const RenderConfig& render) const {
    (void)render;
    return true;
  }

  // scrum-268.8 (DR-3): per-ray wavelength pool size, > 0 only when the
  // backend tags ExitRayRecord::wl_idx with a meaningful pool index. The
  // simulator inverts pool_idx → wl via `380 + (idx + 0.5) * 400 / M` for
  // illuminant-mode scenes. Backends without a pool (CPU + legacy) return 0
  // and the simulator falls back to per-batch wl in SimData.outgoing_wl_.
  virtual uint32_t WlPoolSize() const { return 0; }

  TraceBackend(const TraceBackend&) = delete;
  TraceBackend(TraceBackend&&) = delete;
  TraceBackend& operator=(const TraceBackend&) = delete;
  TraceBackend& operator=(TraceBackend&&) = delete;

 protected:
  TraceBackend() = default;
};

}  // namespace lumice

#endif  // CORE_TRACE_BACKEND_H_
