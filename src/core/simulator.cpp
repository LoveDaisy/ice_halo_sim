#include "core/simulator.hpp"

#include <chrono>
#include <cstddef>
#include <cstring>
#include <memory>
#include <thread>

#include "config/crystal_config.hpp"
#include "config/light_config.hpp"
#include "config/proj_config.hpp"
#include "config/sim_data.hpp"
#include "core/beam_tracer.hpp"
#include "core/buffer.hpp"
#include "core/crystal.hpp"
#include "core/filter.hpp"
#include "core/math.hpp"
#include "core/optics.hpp"
#include "util/illuminant.hpp"
#include "util/logger.hpp"
#include "util/queue.hpp"

namespace lumice {

/**
 * @brief Sample on crystal & init origin (p & fid) of rays.
 *        NOTE: direction of rays should be given.
 */
void InitRay_p_fid(const Crystal& curr_crystal, RayBuffer* ray_buf_ptr) {
  if (!ray_buf_ptr) {
    return;
  }

  RayBuffer& ray_buf = *ray_buf_ptr;
  // p & fid: sample on crystal faces
  auto total_faces = curr_crystal.TotalTriangles();
  const auto* face_area = curr_crystal.GetTirangleArea();
  const auto* face_norm = curr_crystal.GetTriangleNormal();
  const auto* face_vtx = curr_crystal.GetTriangleVtx();

  constexpr size_t kMaxTriangles = 64;
  float proj_prob_buf[kMaxTriangles];
  std::unique_ptr<float[]> proj_prob_heap;
  float* proj_prob = proj_prob_buf;
  if (total_faces > kMaxTriangles) {
    proj_prob_heap = std::make_unique<float[]>(total_faces);
    proj_prob = proj_prob_heap.get();
  }
  for (auto& r : ray_buf) {
    const auto* d = r.d_;
    for (size_t j = 0; j < total_faces; j++) {
      proj_prob[j] = std::max(-Dot3(d, face_norm + j * 3) * face_area[j], 0.0f);
    }
    // fid
    RandomSample(total_faces, proj_prob, &r.fid_);
    // p
    SampleTrianglePoint(face_vtx + r.fid_ * 9, r.p_);
  }
}

static void SampleRayDir(const SunParam& p, float* d, size_t num, size_t step) {
  SampleSphCapPoint(p.azimuth_ + 180.0f, -p.altitude_, p.diameter_ / 2.0f, d, num, step);
}


// Sample a single crystal rotation from the axis distribution.
// Shared by InitRay_rot (MC path, entire RayBuffer) and BT path (single orientation).
static Rotation SampleOneRotation(RandomNumberGenerator& rng, const AxisDistribution& crystal_axis) {
  float lon_lat[2]{};
  if (crystal_axis.azimuth_dist.type != DistributionType::kUniform ||
      crystal_axis.latitude_dist.type != DistributionType::kUniform) {
    RandomSampler::SampleSphericalPointsSph(crystal_axis, lon_lat);
  } else {
    RandomSampler::SampleSphericalPointsSph(lon_lat, 1, 2);
  }
  float roll = rng.Get(crystal_axis.roll_dist) * math::kDegreeToRad;
  constexpr float kZ[3]{ 0, 0, 1 };
  constexpr float kY[3]{ 0, 1, 0 };
  return Rotation(kZ, roll).Chain(kY, math::kPi_2 - lon_lat[1]).Chain(kZ, lon_lat[0]);
}

/**
 * @brief Set initial value of d & w (direction and intensity) for rays.
 */
void InitRay_d_w_previdx(const SunParam& light_param, const WlParam& wl_param, size_t ray_num,  // input
                         RayBuffer* ray_buf_ptr) {                                              // output
  if (!ray_buf_ptr) {
    return;
  }
  const auto& ray_buf = *ray_buf_ptr;

  // w, prev_ray_idx: set init weight & previous ray index
  for (auto& r : ray_buf) {
    r.w_ = wl_param.weight_;
    r.prev_ray_idx_ = kInfSize;
  }

  // d: sample direction
  SampleRayDir(light_param, ray_buf[0].d_, ray_num, sizeof(RaySeg));

  // Then rotate
  for (auto& r : ray_buf) {
    r.crystal_rot_.ApplyInverse(r.d_);
  }
}


void InitRay_rot(RandomNumberGenerator& rng, const AxisDistribution& crystal_axis,  // input
                 RayBuffer buffer_data[2]) {                                        // output
  for (auto& r : buffer_data[0]) {
    r.crystal_rot_ = SampleOneRotation(rng, crystal_axis);
  }
}


void InitRay_other_info(const Crystal& curr_crystal, size_t curr_crystal_id, size_t all_data_idx,  // input
                        RayBuffer buffer_data[2]) {                                                // output
  for (auto& r : buffer_data[0]) {
    r.crystal_idx_ = curr_crystal_id;
    r.crystal_config_id_ = curr_crystal.config_id_;
    r.root_ray_idx_ = all_data_idx++;
    r.state_ = RaySeg::kNormal;
    r.rp_.Clear();
    r.rp_ << curr_crystal.GetFn(r.fid_);
  }
}


// NOLINTNEXTLINE(readability-function-size)
void InitRayFirstMs(RandomNumberGenerator& rng, const SunParam& light_param, const WlParam& wl_param,
                    size_t curr_ray_num,                                                                        // input
                    const Crystal& curr_crystal, size_t curr_crystal_id, const AxisDistribution& crystal_axis,  // input
                    RayBuffer buffer_data[2], RayBuffer& all_data) {  // output
  buffer_data[0].size_ = curr_ray_num;

  // 1.0 init crystal_rot
  InitRay_rot(rng, crystal_axis, buffer_data);

  // 1.1 init d & w & (prev_ray_idx)
  InitRay_d_w_previdx(light_param, wl_param, curr_ray_num, buffer_data + 0);

  // 1.2 init p & fid
  InitRay_p_fid(curr_crystal, buffer_data + 0);

  // 1.3 init crystal_id, root_ray_idx, rp, state
  InitRay_other_info(curr_crystal, curr_crystal_id, all_data.size_, buffer_data);

  all_data.EmplaceBack(buffer_data[0]);
}


// NOLINTNEXTLINE(readability-function-size)
void InitRayOtherMs(RandomNumberGenerator& rng, const RayBuffer init_data[2], size_t curr_ray_num,              // input
                    const Crystal& curr_crystal, size_t curr_crystal_id, const AxisDistribution& crystal_axis,  // input
                    RayBuffer buffer_data[2], RayBuffer& all_data, size_t& init_ray_offset) {  // output
  buffer_data[0].size_ = 0;

  // 1.0 copy previous rays.
  buffer_data[0].EmplaceBack(init_data[0], init_ray_offset, curr_ray_num);
  init_ray_offset += curr_ray_num;

  // 1.1 init crystal_rot
  InitRay_rot(rng, crystal_axis, buffer_data);
  // Then rotate d
  for (auto& r : buffer_data[0]) {
    r.crystal_rot_.ApplyInverse(r.d_);
  }

  // 1.2 init p & fid
  InitRay_p_fid(curr_crystal, buffer_data + 0);

  // 1.3 init crystal_id, crystal_config_id, root_ray_idx, rp, state
  InitRay_other_info(curr_crystal, curr_crystal_id, all_data.size_, buffer_data);

  all_data.EmplaceBack(buffer_data[0]);
}


struct CrystalMaker {
  RandomNumberGenerator& rng_;

  Crystal operator()(const PrismCrystalParam& p) {
    // Check all face distributions. If they are all no-random, then create a regular prism.
    bool regular = !std::any_of(std::begin(p.d_), std::end(p.d_),
                                [](const auto& d) { return d.type != DistributionType::kNoRandom; });

    float h = std::abs(rng_.Get(p.h_));

    if (regular) {
      return Crystal::CreatePrism(h);
    } else {
      float dist[6]{};
      for (int i = 0; i < 6; i++) {
        dist[i] = std::abs(rng_.Get(p.d_[i]));
      }
      return Crystal::CreatePrism(h, dist);
    }
  }

  Crystal operator()(const PyramidCrystalParam& p) {
    // Check all face distributions. If they are all no-random, then it is face-regular.
    bool face_regular = !std::any_of(std::begin(p.d_), std::end(p.d_),
                                     [](const auto& d) { return d.type != DistributionType::kNoRandom; });
    // Check Miller index. If they are [1, 0, 1], then it is miller-regular.
    bool miller_regular_u = p.miller_indices_u_[0] == 1 && p.miller_indices_u_[1] == 0 && p.miller_indices_u_[2] == 1;
    bool miller_regular_l = p.miller_indices_l_[0] == 1 && p.miller_indices_l_[1] == 0 && p.miller_indices_l_[2] == 1;

    float h1 = std::abs(rng_.Get(p.h_pyr_u_));
    float h2 = std::abs(rng_.Get(p.h_prs_));
    float h3 = std::abs(rng_.Get(p.h_pyr_l_));

    if (face_regular && miller_regular_u && miller_regular_l) {
      return Crystal::CreatePyramid(h1, h2, h3);
    } else {
      float dist[6]{};
      for (int i = 0; i < 6; i++) {
        dist[i] = std::abs(rng_.Get(p.d_[i]));
      }
      return Crystal::CreatePyramid(p.miller_indices_u_[0], p.miller_indices_u_[2], p.miller_indices_l_[0],
                                    p.miller_indices_l_[2], h1, h2, h3, dist);
    }
    return Crystal{};
  }
};


bool IsDeterministic(const CrystalParam& param) {
  auto all_no_random = [](const Distribution* d, size_t n) {
    return !std::any_of(d, d + n, [](const auto& x) { return x.type != DistributionType::kNoRandom; });
  };
  return std::visit(
      [&](const auto& p) {
        if constexpr (std::is_same_v<std::decay_t<decltype(p)>, PrismCrystalParam>) {
          return p.h_.type == DistributionType::kNoRandom && all_no_random(p.d_, 6);
        } else {
          return p.h_prs_.type == DistributionType::kNoRandom &&  //
                 p.h_pyr_u_.type == DistributionType::kNoRandom && p.h_pyr_l_.type == DistributionType::kNoRandom &&
                 all_no_random(p.d_, 6);
        }
      },
      param);
}


RayBuffer AllocateAllData(const SceneConfig& config, size_t ray_num) {
  // Calculate total rays (expected value) used in the whole simulation.
  // total = n * (2 * k + 1) * (1 + k * p1 + k^2 * p1 * p2 + k^3 * p1 * p2 * p3 + ...)
  // where k = max_hits
  size_t total_ray_num = ray_num * (2 * config.max_hits_ + 1);
  float x = 1;
  float y = 1;
  float x_max = 1;
  float y_max = 1;
  for (size_t i = 0; i + 1 < config.ms_.size(); i++) {
    y *= config.max_hits_ * config.ms_[i].prob_;
    x += y;
    y_max *= config.max_hits_;
    x_max += y_max;
  }
  total_ray_num = static_cast<size_t>(total_ray_num * std::min(x * 1.5f, x_max));

  return RayBuffer(total_ray_num);
}


std::unique_ptr<size_t[]> PartitionCrystalRayNum(RandomNumberGenerator& rng, const MsInfo& ms_info, int ray_num) {
  auto crystal_cnt = ms_info.setting_.size();
  auto c_num = std::make_unique<size_t[]>(crystal_cnt);
  auto prob = std::make_unique<float[]>(crystal_cnt + 1);
  for (size_t ci = 0; ci < crystal_cnt; ci++) {
    prob[ci + 1] = ms_info.setting_[ci].crystal_proportion_ + prob[ci];
  }
  for (size_t ci = 0; ci < crystal_cnt; ci++) {
    prob[ci] /= prob[crystal_cnt];
  }

  for (int i = 0; i < ray_num; i++) {
    auto u = rng.GetUniform();
    for (size_t ci = 0; ci < crystal_cnt; ci++) {
      if (prob[ci] <= u && u < prob[ci + 1]) {
        c_num[ci]++;
        break;
      }
    }
  }

  return c_num;
}


void TraceRayBasicInfo(const Crystal& curr_crystal, float refractive_index, size_t curr_ray_num,
                       RayBuffer* buffer_data) {
  // 1 HitSurface.
  {
    float_bf_t d_in{ buffer_data[0][0].d_, sizeof(RaySeg) };
    float_bf_t w_in{ &buffer_data[0][0].w_, sizeof(RaySeg) };
    int_bf_t fid_in{ &buffer_data[0][0].fid_, sizeof(RaySeg) };
    float_bf_t d_out{ buffer_data[1][0].d_, sizeof(RaySeg) };
    float_bf_t w_out{ &buffer_data[1][0].w_, sizeof(RaySeg) };
    HitSurface(curr_crystal, refractive_index, curr_ray_num,  // Input
               d_in, w_in, fid_in,                            // Input, d, w, fid
               d_out, w_out);                                 // Output, d, w
  }

  // 2 Propagate.
  {
    float_bf_t d_in{ buffer_data[1][0].d_, sizeof(RaySeg) };
    float_bf_t w_in{ &buffer_data[1][0].w_, sizeof(RaySeg) };
    float_bf_t p_in{ buffer_data[0][0].p_, sizeof(RaySeg) };
    float_bf_t p_out{ buffer_data[1][0].p_, sizeof(RaySeg) };
    int_bf_t fid_out{ &buffer_data[1][0].fid_, sizeof(RaySeg) };
    Propagate(curr_crystal, curr_ray_num * 2, 2,  // Input
              d_in, p_in, w_in,                   // Input, d, w, p(1/2)
              p_out, fid_out);                    // Output, p, fid
  }

  for (size_t i = 0; i < curr_ray_num; i++) {
    buffer_data[1][i * 2 + 0].rp_ = buffer_data[0][i].rp_;
    buffer_data[1][i * 2 + 1].rp_ = buffer_data[0][i].rp_;
  }

  buffer_data[0].size_ = 0;
  buffer_data[1].size_ = curr_ray_num * 2;
}


void FillRayOtherInfo(size_t curr_ray_num, size_t i,                           // input
                      const Crystal& curr_crystal, const RayBuffer& all_data,  // input
                      RayBuffer buffer_data[2]) {                              // output
  size_t ray_id_offset = all_data.size_;
  for (size_t j = 0; j < buffer_data[1].size_; j++) {
    auto& r = buffer_data[1][j];
    if (r.fid_ > 0) {
      r.rp_ << curr_crystal.GetFn(r.fid_);
    }

    if (i == 0) {
      r.prev_ray_idx_ = j / 2 + ray_id_offset - curr_ray_num;
    } else if (i == 1) {
      r.prev_ray_idx_ = j / 2 * 2 + 1 + ray_id_offset - curr_ray_num * 2;
    } else {
      r.prev_ray_idx_ = j / 2 * 2 + 0 + ray_id_offset - curr_ray_num * 2;
    }

    r.crystal_idx_ = all_data[r.prev_ray_idx_].crystal_idx_;
    r.crystal_config_id_ = all_data[r.prev_ray_idx_].crystal_config_id_;
    r.crystal_rot_ = all_data[r.prev_ray_idx_].crystal_rot_;
    r.root_ray_idx_ = all_data[r.prev_ray_idx_].root_ray_idx_;
  }
}


void CollectData(RandomNumberGenerator& rng, const MsInfo& ms_info, const Filter* filter,  // input
                 RayBuffer* buffer_data, RayBuffer* init_data) {                           // output
  for (auto& r : buffer_data[1]) {
    if (r.w_ < 0) {
      // 0. Total reflection.
      r.state_ = RaySeg::kStopped;
    } else if (r.fid_ < 0) {
      // 1. Outgoing rays.
      if (!filter->Check(r)) {
        // 1.1 Filter out. Marked as stopped.
        r.state_ = RaySeg::kStopped;
      } else if (rng.GetUniform() < ms_info.prob_) {
        // 1.2 Copy outgoing rays into initial data, with probability of p
        r.state_ = RaySeg::kContinue;
      } else {
        // 1.3 Final outgoing rays.
        r.state_ = RaySeg::kOutgoing;
      }
    } else {
      // 2. Normal rays. Squeeze (or better swap?) buffers.
      r.state_ = RaySeg::kNormal;
    }

    if (r.state_ != RaySeg::kNormal) {
      r.crystal_rot_.Apply(r.d_);
      r.crystal_rot_.Apply(r.p_);
    } else {
      buffer_data[0].EmplaceBack(r);
    }
    if (r.state_ == RaySeg::kContinue) {
      init_data[1].EmplaceBack(r);
    }
  }
}


Simulator::Simulator(QueuePtrS<SimBatch> config_queue, QueuePtrS<SimData> data_queue, uint32_t seed)
    : config_queue_(std::move(config_queue)), data_queue_(std::move(data_queue)), stop_(false), idle_(true),
      seed_(seed), rng_(seed != 0 ? seed :
                                    static_cast<uint32_t>(std::chrono::system_clock::now().time_since_epoch().count() ^
                                                          (std::hash<std::thread::id>{}(std::this_thread::get_id())))) {
}

Simulator::Simulator(Simulator&& other) noexcept
    : config_queue_(std::move(other.config_queue_)), data_queue_(std::move(other.data_queue_)),
      stop_(other.stop_.load()), idle_(other.idle_.load()), seed_(other.seed_), rng_(other.rng_),
      logger_(std::move(other.logger_)) {}

Simulator& Simulator::operator=(Simulator&& other) noexcept {
  if (this == &other) {
    return *this;
  }

  config_queue_ = std::move(other.config_queue_);
  data_queue_ = std::move(other.data_queue_);
  stop_ = other.stop_.load();
  idle_ = other.idle_.load();
  seed_ = other.seed_;
  rng_ = other.rng_;
  logger_ = std::move(other.logger_);
  return *this;
}

void Simulator::Run() {
  stop_ = false;
  ILOG_DEBUG(logger_, "Simulator::Run: entry");

  // When a fixed seed is provided, also seed the thread-local global RNG singleton
  // used by sampling functions (RandomSample, SampleTrianglePoint, SampleSphCapPoint, etc.)
  // to ensure fully deterministic behavior.
  if (seed_ != 0) {
    RandomNumberGenerator::GetInstance().SetSeed(seed_);
  }

  CrystalCache crystal_cache;
  SimWorkspace workspace;

  while (true) {
    auto batch = config_queue_->Get();  // Will block until get one
    if (batch.ray_num_ == 0 || stop_) {
      ILOG_DEBUG(logger_, "Simulator::Run: exit (ray_num={}, stop={})", batch.ray_num_, stop_.load());
      break;
    }

    if (!batch.scene_) {
      ILOG_DEBUG(logger_, "Simulator::Run: exit (null scene)");
      break;
    }
    const auto& config = *batch.scene_;
    auto generation = batch.generation_;
    idle_ = false;

    const auto& spectrum = config.light_source_.spectrum_;
    if (auto* illuminant = std::get_if<IlluminantType>(&spectrum)) {
      // Standard illuminant: uniform wavelength sampling + SPD weight
      float wl = 380.0f + rng_.GetUniform() * 400.0f;  // [380, 780] nm
      float weight = GetIlluminantSpd(*illuminant, wl);
      SimulateOneWavelength(config, WlParam{ wl, weight }, batch.ray_num_, crystal_cache, workspace, generation);
    } else {
      // Discrete wavelength list
      const auto& wl_params = std::get<std::vector<WlParam>>(spectrum);
      for (const auto& wl_param : wl_params) {
        if (stop_) {
          break;
        }
        SimulateOneWavelength(config, wl_param, batch.ray_num_, crystal_cache, workspace, generation);
      }
    }

    idle_ = true;
  }
}


void Simulator::SimulateOneWavelength(const SceneConfig& config, const WlParam& wl_param, size_t ray_num,
                                      CrystalCache& crystal_cache, SimWorkspace& workspace, uint64_t generation) {
  ILOG_TRACE(logger_, "Run: get config: ray({}), wl({:.1f},{:.2f})",  //
             ray_num, wl_param.wl_, wl_param.weight_);

  float wl = wl_param.wl_;
  size_t original_ray_num = ray_num;  // ray_num is overwritten in the ms loop; keep original for normalization.

  RayBuffer all_data = AllocateAllData(config, ray_num);
  std::vector<size_t> outgoing_indices;
  std::vector<float> outgoing_d;
  std::vector<float> outgoing_w;
  auto& init_data = workspace.init_data;
  auto& buffer_data = workspace.buffer_data;
  init_data[0].size_ = 0;
  init_data[1].Reset(ray_num * config.max_hits_);

  std::vector<Crystal> all_crystals;
  all_crystals.reserve(16);

  bool first_ms = true;
  bool use_bt = config.use_beam_tracing_;  // BT currently used for first scattering only (performance trade-off)
  float bt_total_intensity = 0.0f;
  size_t bt_orientation_count = 0;

  for (size_t mi = 0; mi < config.ms_.size() && !stop_; mi++) {
    const auto& m = config.ms_[mi];
    auto ms_crystal_cnt = m.setting_.size();
    auto crystal_ray_num = PartitionCrystalRayNum(rng_, m, ray_num);

    // NOTE: ray_num will change between scatterings.
    buffer_data[0].Reset(ray_num * 2);
    buffer_data[1].Reset(ray_num * 2);

    size_t init_ray_offset = 0;
    for (size_t ci = 0; ci < ms_crystal_cnt && !stop_; ci++) {
      const auto& s = m.setting_[ci];
      auto filter = Filter::Create(s.filter_);

      bool deterministic = IsDeterministic(s.crystal_.param_);

      // Look up cross-call cache for deterministic crystal params.
      const CrystalParam* param_ptr = &s.crystal_.param_;
      const Crystal* cached_crystal = nullptr;
      if (deterministic) {
        for (const auto& [key, val] : crystal_cache) {
          if (key == param_ptr) {
            cached_crystal = &val;
            break;
          }
        }
      }

      // For deterministic params, create/copy crystal once per ci iteration.
      size_t ci_crystal_id = kInfSize;

      // --- Beam tracing path (first scattering only, performance trade-off: beam count grows exponentially) ---
      bool bt_handled = false;
      if (use_bt && first_ms) {
        // Create one crystal for beam tracing
        Crystal bt_crystal = [&]() -> Crystal {
          if (cached_crystal != nullptr) {
            return *cached_crystal;
          }
          Crystal c = std::visit(CrystalMaker{ rng_ }, s.crystal_.param_);
          if (deterministic) {
            crystal_cache.emplace_back(param_ptr, c);
            cached_crystal = &crystal_cache.back().second;
          }
          return c;
        }();

        if (bt_crystal.PolygonFaceCount() > 0) {
          filter->InitCrystalSymmetry(bt_crystal);

          // Store BT crystal in all_crystals (once per ci) for RenderConsumer chain walk.
          size_t bt_crystal_idx = all_crystals.size();
          all_crystals.emplace_back(bt_crystal);

          const auto& sun_param = config.light_source_.param_;

          float ri = bt_crystal.GetRefractiveIndex(wl);
          size_t orientation_num = crystal_ray_num[ci];


          for (size_t oi = 0; oi < orientation_num && !stop_; oi++) {
            float light_dir[3];
            SampleRayDir(sun_param, light_dir, 1, sizeof(light_dir));
            Rotation rot = SampleOneRotation(rng_, s.crystal_.axis_);
            auto bt_result = BeamTrace(bt_crystal, rot, light_dir, ri, config.max_hits_);

            // Skip orientations with negligible entry area (numerical stability)
            if (bt_result.total_entry_area < kMinBeamArea) {
              continue;
            }

            for (size_t k = 0; k < bt_result.outgoing_w.size(); k++) {
              // Build outgoing RaySeg for Simulator-level filter + RenderConsumer chain walk.
              RaySeg outgoing_ray{};
              outgoing_ray.state_ = RaySeg::kOutgoing;
              outgoing_ray.d_[0] = bt_result.outgoing_d[3 * k];
              outgoing_ray.d_[1] = bt_result.outgoing_d[3 * k + 1];
              outgoing_ray.d_[2] = bt_result.outgoing_d[3 * k + 2];
              outgoing_ray.crystal_config_id_ = bt_crystal.config_id_;
              outgoing_ray.crystal_idx_ = bt_crystal_idx;
              for (size_t fi = 0; fi < std::min(bt_result.outgoing_raypath[k].size(), static_cast<size_t>(kMaxHits));
                   fi++) {
                outgoing_ray.rp_ << static_cast<IdType>(bt_result.outgoing_raypath[k][fi]);
              }

              if (!filter->Check(outgoing_ray)) {
                continue;
              }

              // Scale weight: w_scaled = w_bt * wl_weight / total_entry_area
              float w_scaled = bt_result.outgoing_w[k] * wl_param.weight_ / bt_result.total_entry_area;
              outgoing_ray.w_ = w_scaled;

              // Workaround: construct a minimal 2-node RaySeg chain (entry + outgoing) so that
              // RenderConsumer's FilterRay chain walk can work. BT has no per-bounce intermediate
              // ray state, so prev_ray_idx_ on the outgoing ray points directly to the entry ray
              // (not a literal "previous ray in the trace"). The chain satisfies FilterRay's
              // termination condition (root.prev == kInfSize) for single-scattering filters.
              // Limitation: supports at most 1 render-level ms_filter (chain depth = 2 nodes).
              RaySeg entry_ray{};
              entry_ray.prev_ray_idx_ = kInfSize;
              size_t entry_idx = all_data.size_;
              entry_ray.root_ray_idx_ = entry_idx;
              entry_ray.crystal_idx_ = bt_crystal_idx;
              entry_ray.crystal_config_id_ = bt_crystal.config_id_;
              entry_ray.state_ = RaySeg::kNormal;
              std::memcpy(entry_ray.d_, light_dir, sizeof(float) * 3);
              entry_ray.w_ = wl_param.weight_;
              all_data.EmplaceBack(entry_ray);

              outgoing_ray.prev_ray_idx_ = entry_idx;
              outgoing_ray.root_ray_idx_ = entry_idx;
              size_t outgoing_idx = all_data.size_;
              all_data.EmplaceBack(outgoing_ray);

              outgoing_indices.push_back(outgoing_idx);
              outgoing_d.push_back(outgoing_ray.d_[0]);
              outgoing_d.push_back(outgoing_ray.d_[1]);
              outgoing_d.push_back(outgoing_ray.d_[2]);
              outgoing_w.push_back(w_scaled);
            }
          }

          bt_total_intensity += wl_param.weight_ * static_cast<float>(orientation_num);
          bt_orientation_count += orientation_num;
          bt_handled = true;
        } else {
          ILOG_WARN(logger_, "Beam tracing: crystal has no polygon faces, falling back to MC");
        }
      }

      if (!bt_handled) {
        for (size_t cn = 0; cn < crystal_ray_num[ci] && !stop_; cn += kSmallBatchRayNum) {  // for a same crystal
          size_t curr_ray_num = std::min(kSmallBatchRayNum, crystal_ray_num[ci] - cn);

          size_t curr_crystal_id = 0;
          if (deterministic && ci_crystal_id != kInfSize) {
            // Reuse crystal from earlier batch in this ci loop
            curr_crystal_id = ci_crystal_id;
          } else if (cached_crystal != nullptr) {
            // Copy from cross-call cache (deterministic, first batch of this ci)
            curr_crystal_id = all_crystals.size();
            all_crystals.emplace_back(*cached_crystal);
            ci_crystal_id = curr_crystal_id;
          } else {
            // Create new crystal (random params, or first-ever deterministic creation)
            curr_crystal_id = all_crystals.size();
            all_crystals.emplace_back(std::visit(CrystalMaker{ rng_ }, s.crystal_.param_));
            if (deterministic) {
              crystal_cache.emplace_back(param_ptr, all_crystals.back());
              cached_crystal = &crystal_cache.back().second;
              ci_crystal_id = curr_crystal_id;
            }
          }
          const auto& curr_crystal = all_crystals[curr_crystal_id];
          filter->InitCrystalSymmetry(curr_crystal);

          // 1. Initialize data
          if (first_ms) {
            InitRayFirstMs(rng_, config.light_source_.param_, wl_param, curr_ray_num,  // input
                           curr_crystal, curr_crystal_id, s.crystal_.axis_,            // input
                           buffer_data, all_data);                                     // output
          } else {
            InitRayOtherMs(rng_, init_data, curr_ray_num,                    // input
                           curr_crystal, curr_crystal_id, s.crystal_.axis_,  // input
                           buffer_data, all_data, init_ray_offset);          // output
          }

          // 2. Start tracing
          float refractive_index = curr_crystal.GetRefractiveIndex(wl);
          for (size_t i = 0; i < config.max_hits_ && !stop_; i++) {
            // 2.1 Trace rays. Deal with d, p, w, fid
            TraceRayBasicInfo(curr_crystal, refractive_index, curr_ray_num,  // input
                              buffer_data);                                  // output
            // After tracing, ray data will be in buffer_data[1], and there are curr_ray_num * 2 rays.

            // 2.2 Fill other information: all but (d, p, w, fid)
            FillRayOtherInfo(curr_ray_num, i, curr_crystal, all_data,  // input
                             buffer_data);                             // output

            // 2.3 Collect data. And set ray properties: state
            CollectData(rng_, m, filter.get(),    // input
                        buffer_data, init_data);  // output

            // 2.4 Copy to all_data + collect outgoing indices
            size_t base_index = all_data.size_;
            all_data.EmplaceBack(buffer_data[1]);
            for (size_t j = 0; j < buffer_data[1].size_; j++) {
              if (buffer_data[1][j].state_ == RaySeg::kOutgoing) {
                outgoing_indices.push_back(base_index + j);
                const auto& r = buffer_data[1][j];
                outgoing_d.push_back(r.d_[0]);
                outgoing_d.push_back(r.d_[1]);
                outgoing_d.push_back(r.d_[2]);
                outgoing_w.push_back(r.w_);
              }
            }
          }  // hit loop
        }  // small batch loop
      }  // if (!bt_handled)
    }  // crystal loop

    ray_num = init_data[1].size_;
    init_data[0].Reset(ray_num * (config.max_hits_ + 1));
    std::swap(init_data[0], init_data[1]);

    // Shuffle init_data
    for (size_t i = 0; i < init_data[0].size_; i++) {
      size_t j = static_cast<size_t>(rng_.GetUniform() * (init_data[0].size_ - i)) + i;
      std::swap(init_data[0][i], init_data[0][j]);
    }

    first_ms = false;
  }  // ms loop
  if (stop_) {
    return;
  }

  SimData sim_data;
  sim_data.curr_wl_ = wl;
  sim_data.generation_ = generation;
  sim_data.total_intensity_ = use_bt ? bt_total_intensity : wl_param.weight_ * original_ray_num;
  sim_data.root_ray_count_ = use_bt ? bt_orientation_count : original_ray_num;
  sim_data.crystals_ = std::move(all_crystals);
  sim_data.rays_ = std::move(all_data);
  sim_data.outgoing_indices_ = std::move(outgoing_indices);
  sim_data.outgoing_d_ = std::move(outgoing_d);
  sim_data.outgoing_w_ = std::move(outgoing_w);
  data_queue_->Emplace(std::move(sim_data));
}

void Simulator::Stop() {
  stop_ = true;
  config_queue_->Shutdown();
  data_queue_->Shutdown();
}

bool Simulator::IsIdle() const {
  return !stop_ && idle_;
}

void Simulator::SetLogLevel(LogLevel level) {
  logger_.SetLevel(level);
}

}  // namespace lumice
