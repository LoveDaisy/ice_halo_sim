#ifndef CORE_CPU_TRACE_BACKEND_H_
#define CORE_CPU_TRACE_BACKEND_H_

#include <cstddef>
#include <memory>

#include "config/sim_data.hpp"
#include "core/geo3d.hpp"
#include "core/math.hpp"
#include "core/trace_backend.hpp"

namespace lumice {

// CPU implementation of the TraceBackend seam.
//
// Mirrors Simulator::SimulateOneWavelength's loop structure (small-batch = 32
// rays per init+trace pass) per MS layer / per crystal-batch, but exposes the
// trace -> projection -> XYZ-accumulation as a coarse-grained `TraceLayer`.
//
// In this initial cut, the contract is: ONE TraceLayer call per (MS layer,
// crystal_id). Multi-crystal MS layers require multiple TraceLayer calls
// before invoking Recombine. The session-level crystal cache lives in
// `layer_crystals_` so deterministic-crystal layers reuse the same Crystal
// across TraceLayer calls.
class CpuLayerHandle : public LayerHandle {
 public:
  CpuLayerHandle() = default;
  size_t ContinuationCount() const override;

 private:
  friend class CpuTraceBackend;
  RayBuffer continuation_;
};

class CpuTraceBackend : public TraceBackend {
 public:
  CpuTraceBackend();
  ~CpuTraceBackend() override = default;

  void BeginSession(const SessionSpec& spec) override;
  LayerHandlePtr TraceLayer(const RootRaySource& roots) override;
  RootRaySource Recombine(LayerHandlePtr handle, const RecombineSpec& spec) override;
  void ReadbackImage(XyzImageData& out) override;
  void EndSession() override;

  // Diagnostic accessors (unit tests).
  size_t RootRayCount() const { return root_ray_count_; }
  float TotalLandedWeight() const { return total_landed_weight_; }

 private:
  SessionSpec spec_{};
  RandomNumberGenerator rng_;
  Rotation camera_rot_;
  std::unique_ptr<float[]> xyz_buf_;
  int width_ = 0;
  int height_ = 0;

  size_t root_ray_count_ = 0;
  size_t ms_idx_ = 0;  // advances on each Recombine.
  float total_landed_weight_ = 0.0f;

  // Backend-owned storage for the device-resident handle returned by Recombine.
  // Lifetime: from the Recombine that writes it through to the NEXT Recombine
  // (or EndSession). The DeviceRayBatch::backend_ptr handed out by Recombine
  // points at &continuation_buf_.
  RayBuffer continuation_buf_;

  bool in_session_ = false;
};

}  // namespace lumice

#endif  // CORE_CPU_TRACE_BACKEND_H_
