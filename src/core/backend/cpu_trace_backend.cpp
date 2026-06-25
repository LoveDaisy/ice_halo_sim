#include "core/backend/cpu_trace_backend.hpp"

#include <algorithm>
#include <cassert>
#include <cstring>
#include <memory>
#include <utility>
#include <vector>

#include "config/proj_config.hpp"
#include "config/sim_data.hpp"
#include "core/crystal.hpp"
#include "core/filter_spec.hpp"
#include "core/math.hpp"
#include "core/scatter_accum.hpp"
#include "core/simulator.hpp"  // CollectData, PartitionCrystalRayNum
#include "core/trace_ops.hpp"

namespace lumice {

namespace {

constexpr size_t kSmallBatchRayNum = 32;

// Parameter bundles for TraceCrystalBatch. Grouped by lifetime/mutability so the
// trace entry point stays under the clang-tidy parameter threshold without the
// readability-function-size NOLINT suppression (was 19 bare params).

// Immutable per-crystal trace inputs (rebuilt per ci population).
struct CrystalTraceSpec {
  const Crystal& crystal;
  size_t crystal_id;
  const AxisDistribution& axis_dist;
  float refractive_index;
};

// Immutable per-layer batch parameters (constant across the ci loop).
struct BatchTraceSpec {
  const MsInfo& ms_info;
  const FilterSpec* filter_spec;
  const SunParam& sun_param;
  const WlParam& wl_param;
  size_t ci_ray_num;     // this population's ray count (cn loop bound)
  size_t layer_ray_num;  // whole layer's ray count (workspace sizing)
  size_t max_hits;
  bool first_ms;
  uint8_t ms_layer_idx;  // current MS layer index (carried into ExitRayRecord)
  // Host-ray injection (first_ms only). When non-null, TraceCrystalBatch
  // bypasses InitRayFirstMs's light-source sampling and populates workspace[0]
  // directly from host->d/p/w/tf (already in crystal-local space). Used by
  // golden-ray tests (test/golden-analytic/backend/test_cpu_golden_rays.cpp)
  // to inject analytically-known rays; production code passes nullptr.
  const HostRayBatch* host = nullptr;
};

// Mutable I/O buffers threaded across the layer's ci iterations. Reference
// members bind to the caller's locals; const on the struct does not propagate
// to the referents, so the trace loop mutates them through this bundle.
struct BatchTraceBuffers {
  RayBuffer* prev_init;     // [2]; when !first_ms, [0] is input rays
  size_t& init_ray_offset;  // advances inside prev_init across batches
  RayBuffer& all_data;      // bookkeeping
  RayBuffer& cont_collect;  // backend's per-layer continuation buffer
  std::vector<ExitRayRecord>& outgoing;
};

// Drive the per-(crystal-batch) trace main loop. Mirrors the inner
// `for cn -> InitRay* -> for hit -> Trace/Fill/Collect` block of
// Simulator::SimulateOneWavelength so RNG order is preserved across batches.
//
// Inputs:
//   - prev_init: previous-layer rays (used by InitRayOtherMs when !first_ms).
//                init_ray_offset advances inside it across batches.
//   - host_rays_d/p/w/tf: when first_ms, the per-crystal-batch's slice of
//                the host initial-ray ingest (already in crystal-local
//                space). When non-null, the backend bypasses
//                InitRayFirstMs's light-source sampling and uses these
//                values directly.
// Outputs:
//   - outgoing_d/w: appended-to (3*n_outgoing floats and n_outgoing floats).
//   - cont_collect: the rays that survived as continuation (CollectData
//                   pushes into init_data[1]; this function passes
//                   cont_collect there).
//   - all_data: simulator-side bookkeeping buffer (Recorder lookups during
//               FilterSpec::Match traverse it).
void TraceCrystalBatch(RandomNumberGenerator& rng, const CrystalTraceSpec& crystal_spec, const BatchTraceSpec& batch,
                       const BatchTraceBuffers& buffers) {
  // workspace[0] = input rays for the current hit; workspace[1] = traced output.
  RayBuffer workspace[2]{};

  // CollectData expects an `init_data[2]` whose [1] slot is appended to.
  // Wire [1] to point at cont_collect by moving it in/out around the call.
  // We swap cont_collect with init_for_collect[1] before and after the loop
  // so that EmplaceBack writes into cont_collect's storage.

  for (size_t cn = 0; cn < batch.ci_ray_num; cn += kSmallBatchRayNum) {
    size_t curr_ray_num = std::min(kSmallBatchRayNum, batch.ci_ray_num - cn);

    // workspace[0]: CollectData refills it with Normal rays each hit
    // iteration; capacity = layer_ray_num*2 allows up to 2× fan-out growth.
    // workspace[1]: HitSurface fans each workspace[0] ray into 2 outputs.
    // Worst case: workspace[0].size_ == layer_ray_num*2 - 1, so workspace[1]
    // needs (layer_ray_num*2 - 1)*2 ≈ layer_ray_num*4. Using *2 here caused
    // heap-buffer-overflow (scrum-258.x SIGABRT, diagnosed by ASan).
    // Reset() only re-allocates when capacity grows; subsequent small batches
    // reuse storage. (Sizing *2 for workspace[0] was the scrum-253.4 fix.)
    workspace[0].Reset(batch.layer_ray_num * 2);
    workspace[1].Reset(batch.layer_ray_num * 4);

    if (batch.first_ms && batch.host != nullptr) {
      // Host-ray injection bypass: rays are pre-sampled in crystal-local
      // space; skip light-source sampling and InitRay_p_fid. Slice host arrays
      // by `cn` (offset within ci, which is the only crystal population for
      // host-ingest per the 252.3 single-population contract).
      workspace[0].size_ = curr_ray_num;
      const auto* host = batch.host;
      for (size_t i = 0; i < curr_ray_num; i++) {
        auto& r = workspace[0][i];
        size_t hi = cn + i;
        r.crystal_rot_ = Rotation{};  // identity: rays already crystal-local
        r.d_[0] = host->d[hi * 3 + 0];
        r.d_[1] = host->d[hi * 3 + 1];
        r.d_[2] = host->d[hi * 3 + 2];
        r.p_[0] = host->p[hi * 3 + 0];
        r.p_[1] = host->p[hi * 3 + 1];
        r.p_[2] = host->p[hi * 3 + 2];
        r.w_ = host->w[hi];
        r.from_face_ = kInvalidId;
        r.to_face_ = host->tf[hi];
        r.prev_ray_idx_ = kInfSize;
      }
      InitRay_other_info(crystal_spec.crystal, crystal_spec.crystal_id, buffers.all_data.size_, workspace);
      buffers.all_data.EmplaceBack(workspace[0]);
    } else if (batch.first_ms) {
      InitRayFirstMs(rng, batch.sun_param, batch.wl_param, curr_ray_num,                     //
                     crystal_spec.crystal, crystal_spec.crystal_id, crystal_spec.axis_dist,  //
                     workspace, buffers.all_data);
    } else {
      InitRayOtherMs(rng, buffers.prev_init, curr_ray_num,                                   //
                     crystal_spec.crystal, crystal_spec.crystal_id, crystal_spec.axis_dist,  //
                     workspace, buffers.all_data, buffers.init_ray_offset);
    }

    for (size_t i = 0; i < batch.max_hits; i++) {
      // CollectData (called below) routes IsContinue() rays to cont_collect
      // AND refills workspace[0] with IsNormal() rays for the next hit.
      // The workspace[0] refill is the side-effect that drives this loop
      // past depth 0: without it, curr_batch_size at i >= 1 is always 0.
      size_t curr_batch_size = workspace[0].size_;
      TraceRayBasicInfo(crystal_spec.crystal, crystal_spec.refractive_index, curr_batch_size, workspace);
      FillRayOtherInfo(crystal_spec.crystal, workspace);

      // CollectData writes continuation rays via init_for_collect[1].EmplaceBack.
      // We thread cont_collect through that slot via a temporary swap.
      RayBuffer init_for_collect[2]{};
      std::swap(init_for_collect[1], buffers.cont_collect);
      CollectData(rng, batch.ms_info, batch.filter_spec, workspace, init_for_collect);
      std::swap(init_for_collect[1], buffers.cont_collect);

      // Copy traced rays to all_data + collect outgoing.
      // RecorderDataPtr(j) is read BEFORE all_data.EmplaceBack(workspace[1])
      // mutates workspace[1] (EmplaceBack only writes into all_data here, so
      // workspace[1]'s recorders are still valid). The recorder pointer routes
      // inline vs overflow arena correctly via the buffer-level resolver, and
      // since task-284 ExitFaceSeq::kCap == kMaxHits the min() below is a
      // tautological clamp (kept for explicitness; full path always copied).
      buffers.all_data.EmplaceBack(workspace[1]);
      for (size_t j = 0; j < workspace[1].size_; j++) {
        const auto& r = workspace[1][j];
        if (r.IsOutgoing()) {
          ExitRayRecord rec{};
          rec.dir[0] = r.d_[0];
          rec.dir[1] = r.d_[1];
          rec.dir[2] = r.d_[2];
          rec.weight = r.w_;
          rec.crystal_id = static_cast<uint16_t>(crystal_spec.crystal_id);
          rec.ms_layer_idx = batch.ms_layer_idx;
          const RaypathRecorder& rp = workspace[1].RecorderAt(j);
          const uint8_t* seq = workspace[1].RecorderDataPtr(j);
          uint8_t seq_len = static_cast<uint8_t>(std::min<size_t>(rp.size_, ExitFaceSeq::kCap));
          rec.path.size_ = seq_len;
          std::memcpy(rec.path.data_, seq, seq_len);
          buffers.outgoing.push_back(rec);
        }
      }
    }  // hit loop
  }  // small-batch loop
}

}  // namespace


// ============================== CpuLayerHandle ===============================

size_t CpuLayerHandle::ContinuationCount() const {
  return continuation_.size_;
}

LayerStats CpuLayerHandle::GetLayerStats() const {
  return stats_;
}


// ============================== CpuTraceBackend ==============================

CpuTraceBackend::CpuTraceBackend() : rng_(0) {}

void CpuTraceBackend::BeginSession(const SessionSpec& spec) {
  assert(!in_session_ && "BeginSession called on an already-open session");
  assert(spec.scene != nullptr && "SessionSpec.scene must be non-null");
  assert(spec.render != nullptr && "SessionSpec.render must be non-null");
  // Cross-seed reuse: once seeded, spec.seed must be 0 (no reseed intent)
  // or the same seed (repeated SimBatches within the same render). A
  // different non-zero seed on the same instance would silently no-op;
  // assert here to make that programming error visible at runtime.
  assert(!seeded_ || spec.seed == 0 || spec.seed == seeded_seed_);

  spec_ = spec;
  in_session_ = true;
  ms_idx_ = 0;
  root_ray_count_ = 0;
  total_landed_weight_ = 0.0f;

  width_ = spec.render->resolution_[0];
  height_ = spec.render->resolution_[1];
  size_t pix = static_cast<size_t>(width_) * static_cast<size_t>(height_);
  xyz_buf_ = std::make_unique<float[]>(pix * 3);
  std::memset(xyz_buf_.get(), 0, pix * 3 * sizeof(float));

  camera_rot_ = MakeCameraRotation(*spec.render);

  // Seed RNGs once per backend lifetime, not once per session. Simulator
  // drives BeginSession / EndSession per 128-ray SimBatch, so reseeding here
  // would collapse axis-sample diversity to the first 128 draws repeated.
  // Mirrors Simulator::Run's seed-at-entry semantics (simulator.cpp:592-594).
  if (spec.seed != 0 && !seeded_) {
    rng_.SetSeed(spec.seed);
    RandomNumberGenerator::GetInstance().SetSeed(spec.seed);
    seeded_seed_ = spec.seed;
    seeded_ = true;
  }

  continuation_buf_ = RayBuffer{};
  exit_records_.clear();
}


LayerHandlePtr CpuTraceBackend::TraceLayer(const RootRaySource& roots) {
  assert(in_session_ && "TraceLayer called outside BeginSession/EndSession");
  assert(spec_.scene != nullptr);
  assert(ms_idx_ < spec_.scene->ms_.size() && "TraceLayer beyond configured MS layers");

  const auto& ms_info = spec_.scene->ms_[ms_idx_];
  assert(!ms_info.setting_.empty() && "MS layer has no scattering settings");

  bool first_ms = !roots.is_device;

  size_t total_ray_num = first_ms ? roots.host.count : roots.device.count;
  if (first_ms) {
    root_ray_count_ = total_ray_num;
  }
  if (total_ray_num == 0) {
    return std::make_unique<CpuLayerHandle>();
  }

  // Partition rays across crystal populations within this MS layer.
  // Mirrors Simulator::SimulateOneWavelength (simulator.cpp:572-586): build
  // proportions from setting_[ci].crystal_proportion_ + zeros carry (one
  // session = one wavelength here, no cross-wavelength accumulation).
  size_t crystal_cnt = ms_info.setting_.size();
  std::vector<float> proportions;
  proportions.reserve(crystal_cnt);
  for (size_t ci = 0; ci < crystal_cnt; ci++) {
    proportions.push_back(ms_info.setting_[ci].crystal_proportion_);
  }
  std::vector<double> carry(crystal_cnt, 0.0);
  auto crystal_ray_num = PartitionCrystalRayNum(proportions, total_ray_num, carry);

  // Prepare bookkeeping buffer + the input init_data for InitRayOtherMs.
  RayBuffer all_data = AllocateAllData(*spec_.scene, total_ray_num);
  RayBuffer prev_init[2]{};
  if (!first_ms) {
    // Take ownership of the continuation buffer that was handed back to the
    // caller by Recombine. The DeviceRayBatch::backend_ptr points to
    // &continuation_buf_, but at this point we are about to invalidate the
    // backend_ptr lifetime by moving the buffer into a fresh slot — exactly
    // what the contract permits when TraceLayer is the next call after
    // Recombine.
    assert(roots.device.backend_ptr == &continuation_buf_ &&
           "device.backend_ptr does not match this backend's continuation buffer");
    prev_init[0] = std::move(continuation_buf_);
  }

  // Per-layer continuation collection buffer (shared across all ci).
  RayBuffer cont_collect;
  cont_collect.Reset(total_ray_num * spec_.scene->max_hits_);

  std::vector<ExitRayRecord> outgoing_records;
  outgoing_records.reserve(total_ray_num * spec_.scene->max_hits_);

  // ci loop: mirrors simulator.cpp:593-686 — per crystal population, resolve
  // crystal + filter + n_idx, then drive TraceCrystalBatch over crystal_ray_num[ci]
  // rays. init_ray_offset advances across ci (passed by reference into
  // InitRayOtherMs), preserving the simulator's RNG draw order.
  size_t init_ray_offset = 0;
  for (size_t ci = 0; ci < crystal_cnt; ci++) {
    size_t ci_n = crystal_ray_num[ci];
    if (ci_n == 0) {
      continue;
    }
    const auto& setting = ms_info.setting_[ci];
    const auto& crystal_axis = setting.crystal_.axis_;

    Crystal crystal;
    size_t crystal_id = 0;
    float refractive_index = 0.0f;
    if (first_ms && ci == 0 && roots.host.crystal != nullptr) {
      // Host-supplied crystal path: only ci==0 of the first MS layer uses
      // the caller's crystal (the host-ingest path is single-population by
      // construction in 252.3). ci>0 always builds via MakeCrystal.
      crystal = *roots.host.crystal;
      crystal_id = roots.host.crystal_id;
      refractive_index = roots.host.refractive_index;
    } else {
      crystal = MakeCrystal(rng_, setting.crystal_.param_);
      crystal_id = ci;
      refractive_index = crystal.GetRefractiveIndex(spec_.wl.wl_);
    }

    auto filter_spec = FilterSpec::Create(setting.filter_, crystal, crystal_axis);

    // ci_n = this population's ray count (cn loop bound); total_ray_num = the
    // whole layer's ray count, used to size the trace workspace (see
    // TraceCrystalBatch capacity comment).
    CrystalTraceSpec crystal_spec{ crystal, crystal_id, crystal_axis, refractive_index };
    // Host-ray injection applies only to ci==0 of the first MS layer (single-
    // population contract from 252.3) AND only when the caller provided
    // d/p/w/tf pointers. Production paths leave host_inject == nullptr.
    const HostRayBatch* host_inject = (first_ms && ci == 0 && roots.host.d != nullptr && roots.host.p != nullptr &&
                                       roots.host.w != nullptr && roots.host.tf != nullptr) ?
                                          &roots.host :
                                          nullptr;
    BatchTraceSpec batch{ ms_info,
                          filter_spec.get(),
                          spec_.scene->light_source_.param_,
                          spec_.wl,
                          ci_n,
                          total_ray_num,
                          spec_.scene->max_hits_,
                          first_ms,
                          static_cast<uint8_t>(ms_idx_),
                          host_inject };
    BatchTraceBuffers buffers{ prev_init, init_ray_offset, all_data, cont_collect, outgoing_records };
    TraceCrystalBatch(rng_, crystal_spec, batch, buffers);
  }

  // Exit seam (scrum-258.2): append this layer's rich outgoing records to
  // the session-level accumulator BEFORE projection. ReadbackExitRays
  // returns these for the simulator to drive the legacy consumer projection
  // (which still consumes dir/weight extracted from each record).
  //
  // [PARITY-ORACLE] The ScatterOutgoingToXyz accumulation below is
  // DELIBERATELY RETAINED (not exit-seam leftover): it keeps xyz_buf_
  // populated so ReadbackImage() can serve as the CPU-vs-Metal parity
  // reference image. Production GUI/CLI never reads this image — they use
  // ReadbackExitRays + legacy projection — only the parity harness does
  // (see the [TEST-ONLY] note on ReadbackImage in cpu_trace_backend.hpp).
  // Do NOT remove without retiring the Metal/Cpu parity battery.
  size_t exit_n = outgoing_records.size();
  std::vector<float> outgoing_d(exit_n * 3);
  std::vector<float> outgoing_w(exit_n);
  for (size_t i = 0; i < exit_n; i++) {
    outgoing_d[i * 3 + 0] = outgoing_records[i].dir[0];
    outgoing_d[i * 3 + 1] = outgoing_records[i].dir[1];
    outgoing_d[i * 3 + 2] = outgoing_records[i].dir[2];
    outgoing_w[i] = outgoing_records[i].weight;
  }
  exit_records_.insert(exit_records_.end(), std::make_move_iterator(outgoing_records.begin()),
                       std::make_move_iterator(outgoing_records.end()));

  // Drain the per-layer outgoing into the accumulator.
  ScatterOutgoingToXyz(outgoing_d.data(), outgoing_w.data(), outgoing_w.size(),  //
                       *spec_.render, camera_rot_, spec_.wl.wl_,                 //
                       xyz_buf_.get(), &total_landed_weight_);

  auto handle = std::make_unique<CpuLayerHandle>();
  handle->continuation_ = std::move(cont_collect);
  // Aggregate exit-ray stats for parity harness (CPU-vs-Metal oracle).
  // All rays that left the crystal = XYZ-bound (outgoing_w) + continuation
  // (handle->continuation_). Note: CPU applies ms.prob_ routing so
  // exit_count < Metal v1 (pass-all) on non-final layers by design.
  handle->stats_.exit_count = outgoing_w.size() + handle->continuation_.size_;
  float w_sum = 0.0f;
  for (float w : outgoing_w) {
    w_sum += w;
  }
  for (size_t i = 0; i < handle->continuation_.size_; i++) {
    w_sum += handle->continuation_[i].w_;
  }
  handle->stats_.exit_w_sum = w_sum;
  return handle;
}


RootRaySource CpuTraceBackend::Recombine(LayerHandlePtr handle, const RecombineSpec& spec) {
  assert(in_session_ && "Recombine called outside BeginSession/EndSession");
  assert(handle && "Recombine called with null handle");

  auto* cpu_handle = dynamic_cast<CpuLayerHandle*>(handle.get());
  assert(cpu_handle != nullptr && "Recombine handed a non-Cpu LayerHandle");

  // Take ownership of the continuation rays. This frees the previous
  // continuation_buf_'s storage — matching the documented lifetime
  // (backend_ptr from the previous Recombine is now invalid).
  continuation_buf_ = std::move(cpu_handle->continuation_);

  if (spec.shuffle && continuation_buf_.size_ > 1) {
    // Fisher-Yates shuffle — mirror Simulator's swap-based shuffle order
    // (same loop shape so RNG draws stay aligned with the legacy path).
    for (size_t i = 0; i < continuation_buf_.size_; i++) {
      size_t j = static_cast<size_t>(rng_.GetUniform() * (continuation_buf_.size_ - i)) + i;
      std::swap(continuation_buf_[i], continuation_buf_[j]);
    }
  }

  ms_idx_++;

  DeviceRayBatch dev;
  dev.backend_ptr = &continuation_buf_;
  dev.count = continuation_buf_.size_;
  return RootRaySource::FromDevice(dev);
}


void CpuTraceBackend::ReadbackImage(XyzImageData& out) {
  assert(in_session_ && "ReadbackImage called outside BeginSession/EndSession");
  assert(out.data != nullptr && "XyzImageData.data must be non-null");
  assert(out.width == width_ && out.height == height_ &&
         "XyzImageData dimensions must match SessionSpec.render resolution");
  size_t pix = static_cast<size_t>(width_) * static_cast<size_t>(height_);
  std::memcpy(out.data, xyz_buf_.get(), pix * 3 * sizeof(float));
}


// Exit seam (scrum-258.2): move the session-accumulated rich exit records
// out. Single-MS: equals the only layer's outgoing set. NOT idempotent —
// caller must consume the result within a single ReadbackExitRays call;
// subsequent calls in the same session return 0 records (move semantics).
// Cleared by EndSession even when the caller never reads.
size_t CpuTraceBackend::ReadbackExitRays(std::vector<ExitRayRecord>& out) {
  assert(in_session_ && "ReadbackExitRays called outside BeginSession/EndSession");
  out = std::move(exit_records_);
  exit_records_.clear();
  return out.size();
}

// task-268.4 per-layer destructive drain. CPU semantics identical to
// ReadbackExitRays — the only difference vs Metal is that the CPU backend
// never accumulated session-level device state.
size_t CpuTraceBackend::DrainExits(std::vector<ExitRayRecord>& out) {
  assert(in_session_ && "DrainExits called outside BeginSession/EndSession");
  out = std::move(exit_records_);
  exit_records_.clear();
  return out.size();
}


void CpuTraceBackend::EndSession() {
  in_session_ = false;
  ms_idx_ = 0;
  root_ray_count_ = 0;
  total_landed_weight_ = 0.0f;
  xyz_buf_.reset();
  continuation_buf_ = RayBuffer{};
  exit_records_.clear();
  spec_ = SessionSpec{};
}

}  // namespace lumice
