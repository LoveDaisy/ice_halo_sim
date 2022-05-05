#include "core/simulator.hpp"

#include <cstddef>
#include <thread>

#include "config/light_config.hpp"
#include "config/proj_config.hpp"
#include "config/sim_data.hpp"
#include "core/buffer.hpp"
#include "core/crystal.hpp"
#include "core/filter.hpp"
#include "core/math.hpp"
#include "core/optics.hpp"
#include "include/log.hpp"
#include "util/queue.hpp"

namespace icehalo {
namespace v3 {

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
  std::unique_ptr<float[]> proj_prob{ new float[curr_crystal.TotalFaces()]{} };
  const auto* face_norm = curr_crystal.GetFaceNorm();
  const auto* face_area = curr_crystal.GetFaceArea();
  auto total_faces = curr_crystal.TotalFaces();
  for (auto& r : ray_buf) {
    const auto* d = r.d_;
    for (size_t j = 0; j < total_faces; j++) {
      proj_prob[j] = std::max(-Dot3(d, face_norm + j * 3) * face_area[j], 0.0f);
    }
    // fid
    RandomSample(total_faces, proj_prob.get(), &r.fid_);
    // p
    SampleTrianglePoint(curr_crystal.GetFaceVtx() + r.fid_ * 9, r.p_);
  }
}

struct RayDirSampler {
  float* d_;
  size_t num_;
  size_t step_;

  void operator()(const SunParam& p) {
    SampleSphCapPoint(p.azimuth_ + 180.0f, -p.altitude_, p.diameter_ / 2.0f, d_, num_, step_);
  }

  void operator()(const StreetLightParam& /* p */) {
    // NOTE: current we do **NOT** support street light source
    LOG_WARNING("we will support street light later.");
  }
};

/**
 * @brief Set initial value of d & w (direction and intensity) for rays.
 */
void InitRay_d_w_previdx(const LightSourceParam& light_param, const WlParam& wl_param, size_t ray_num,  // input
                         RayBuffer* ray_buf_ptr) {                                                      // output
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
  std::visit(RayDirSampler{ ray_buf[0].d_, ray_num, sizeof(RaySeg) }, light_param);
}


void InitRay_other_info(const Crystal& curr_crystal, size_t curr_crystal_id, size_t all_data_idx,  // input
                        RayBuffer buffer_data[2]) {                                                // output
  for (auto& r : buffer_data[0]) {
    r.crystal_id_ = curr_crystal_id;
    r.crystal_config_id_ = curr_crystal.config_id_;
    r.root_ray_idx_ = all_data_idx++;
    r.state_ = RaySeg::kNormal;
    r.rp_.Clear();
    r.rp_ << curr_crystal.GetFn(r.fid_);

    LOG_DEBUG("init ray d: %.6f,%.6f,%.6f", r.d_[0], r.d_[1], r.d_[2]);
    LOG_DEBUG("init ray p: %.6f,%.6f,%.6f", r.p_[0], r.p_[1], r.p_[2]);
  }
}


void InitRayFirstMs(const LightSourceParam& light_param, const WlParam& wl_param, size_t curr_ray_num,  // input
                    const Crystal& curr_crystal, size_t curr_crystal_id,                                // input
                    RayBuffer buffer_data[2], RayBuffer& all_data) {                                    // output
  buffer_data[0].size_ = 0;

  // 1.1 init d & w & (prev_ray_idx)
  buffer_data[0].size_ = curr_ray_num;
  InitRay_d_w_previdx(light_param, wl_param, curr_ray_num, buffer_data + 0);

  // 1.2 init p & fid
  InitRay_p_fid(curr_crystal, buffer_data + 0);

  // 1.3 init crystal_id, root_ray_idx, rp, state
  InitRay_other_info(curr_crystal, curr_crystal_id, all_data.size_, buffer_data);

  all_data.EmplaceBack(buffer_data[0]);
}


void InitRayOtherMs(const RayBuffer init_data[2], size_t curr_ray_num,                         // input
                    const Crystal& curr_crystal, size_t curr_crystal_id,                       // input
                    RayBuffer buffer_data[2], RayBuffer& all_data, size_t& init_ray_offset) {  // output
  buffer_data[0].size_ = 0;

  // 1.1 copy previous rays
  buffer_data[0].EmplaceBack(init_data[0], init_ray_offset, curr_ray_num);
  init_ray_offset += curr_ray_num;

  // 1.2 init p & fid
  InitRay_p_fid(curr_crystal, buffer_data + 0);

  // 1.3 init crystal_id, crystal_config_id, root_ray_idx, rp, state
  InitRay_other_info(curr_crystal, curr_crystal_id, all_data.size_, buffer_data);

  all_data.EmplaceBack(buffer_data[0]);
}


struct CrystalMaker {
  RandomNumberGenerator& rng_;

  Crystal operator()(const PrismCrystalParam& p) {
    float h = rng_.Get(p.h_);
    // TODO: face distance
    return Crystal::CreatePrism(h);
  }

  Crystal operator()(const PyramidCrystalParam& /* p */) {
    // TODO:
    return Crystal{};
  }
};


Crystal SampleCrystal(RandomNumberGenerator& rng, const CrystalConfig& crystal_config) {
  auto crystal = std::visit(CrystalMaker{ rng }, crystal_config.param_);
  crystal.config_id_ = crystal_config.id_;

  float lon = rng.Get(crystal_config.axis_.azimuth_dist) * math::kDegreeToRad;
  float lat = rng.Get(crystal_config.axis_.latitude_dist) * math::kDegreeToRad;
  if (lat > math::kPi_2) {
    lat = math::kPi - lat;
  }
  float roll = rng.Get(crystal_config.axis_.roll_dist) * math::kDegreeToRad;
  float axis[9]{ 1, 0, 0, 0, 1, 0, 0, 0, 1 };
  auto rot = Rotation(axis + 6, roll).Chain(axis + 3, math::kPi_2 - lat).Chain(axis + 6, lon);
  crystal.Rotate(rot);

  LOG_DEBUG("crystal axis: %.4f,%.4f,%.4f", lon, lat, roll);
  return crystal;
}


RayBuffer AllocateAllData(const SceneConfig& config) {
  size_t ray_num = config.ray_num_;

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
  std::unique_ptr<size_t[]> c_num{ new size_t[crystal_cnt]{} };
  std::unique_ptr<float[]> prob{ new float[crystal_cnt + 1]{} };
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

    r.crystal_id_ = all_data[r.prev_ray_idx_].crystal_id_;
    r.crystal_config_id_ = all_data[r.prev_ray_idx_].crystal_config_id_;
    r.root_ray_idx_ = all_data[r.prev_ray_idx_].root_ray_idx_;

    LOG_DEBUG("hit loop ray p: %.6f,%.6f,%.6f,%zu", r.p_[0], r.p_[1], r.p_[2], i);
    LOG_DEBUG("hit loop ray d: %.6f,%.6f,%.6f,%zu", r.d_[0], r.d_[1], r.d_[2], i);
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
        init_data[1].EmplaceBack(r);
      } else {
        // 1.3 Final outgoing rays.
        r.state_ = RaySeg::kOutgoing;
      }
    } else {
      // 2. Normal rays. Squeeze (or better swap?) buffers.
      buffer_data[0].EmplaceBack(r);
      r.state_ = RaySeg::kNormal;
    }
  }
}


Simulator::Simulator(QueuePtrS<SceneConfig> config_queue, QueuePtrS<SimData> data_queue)
    : config_queue_(config_queue), data_queue_(data_queue), stop_(false), idle_(true),
#ifdef RANDOM_SEED
      rng_(std::chrono::system_clock::now().time_since_epoch().count() ^
           (std::hash<std::thread::id>{}(std::this_thread::get_id())))
#else
      rng_(0)
#endif
{
}

Simulator::Simulator(Simulator&& other)
    : config_queue_(std::move(other.config_queue_)), data_queue_(std::move(other.data_queue_)),
      stop_(other.stop_.load()), idle_(other.idle_.load()), rng_(std::move(other.rng_)) {}

Simulator& Simulator::operator=(Simulator&& other) {
  if (this == &other) {
    return *this;
  }

  config_queue_ = std::move(other.config_queue_);
  data_queue_ = std::move(other.data_queue_);
  stop_ = other.stop_.load();
  idle_ = other.idle_.load();
  rng_ = std::move(other.rng_);
  return *this;
}

#define CHECK_STOP \
  if (stop_) {     \
    break;         \
  }

void Simulator::Run() {
  stop_ = false;
  while (true) {
    // Config in the config_queue is processed by frontend of simulator (NOT GUI) so that
    // it has small ray_num and contains only one wavelength parameter of light source.
    auto config = config_queue_->Get();  // Will block until get one
    if (config.ray_num_ == 0) {
      // If no data in the queue or recieve a terminal signal, abort simulation
      break;
    }
    CHECK_STOP

    size_t curr_rng_seed = std::chrono::system_clock::now().time_since_epoch().count() ^
                           (std::hash<std::thread::id>{}(std::this_thread::get_id()));

    idle_ = false;
    for (const auto& curr_wl_param : config.light_source_.wl_param_) {
      LOG_DEBUG("Simulator::Run: get config(%u): ray(%zu), wl(%.1f,%.2f)",  //
                config.id_, config.ray_num_, curr_wl_param.wl_, curr_wl_param.weight_);

      rng_.SetSeed(curr_rng_seed);
      float wl = curr_wl_param.wl_;  // Take first wl **ONLY**. Single wl in a single run.

      // For memory saving, it's better to keep ray_num small.
      // Actually, if ms_prob = 1.0, i.e. all outgoing rays will be sent to next crystal,
      // then the final ray number for init_data will be (num0 * (max_hits + 1)^ms_num),
      // which increase rapidily.
      size_t ray_num = config.ray_num_;

      RayBuffer all_data = AllocateAllData(config);
      RayBuffer init_data[2]{ RayBuffer(), RayBuffer(ray_num * config.max_hits_) };
      RayBuffer buffer_data[2]{};

      std::vector<Crystal> all_crystals(16);

      bool first_ms = true;
      for (const auto& m : config.ms_) {
        auto ms_crystal_cnt = m.setting_.size();
        auto crystal_ray_num = PartitionCrystalRayNum(rng_, m, ray_num);

        // NOTE: ray_num will change between scatterings.
        buffer_data[0].Reset(ray_num * 2);
        buffer_data[1].Reset(ray_num * 2);

        size_t init_ray_offset = 0;
        for (size_t ci = 0; ci < ms_crystal_cnt; ci++) {
          const auto& s = m.setting_[ci];

          for (size_t cn = 0; cn < crystal_ray_num[ci]; cn += kSmallBatchRayNum) {
            size_t curr_ray_num = std::min(kSmallBatchRayNum, crystal_ray_num[ci] - cn);

            size_t curr_crystal_id = all_crystals.size();
            all_crystals.emplace_back(SampleCrystal(rng_, s.crystal_));
            const auto& curr_crystal = all_crystals.back();

            // 1. Initialize data
            if (first_ms) {
              InitRayFirstMs(config.light_source_.param_, curr_wl_param, curr_ray_num,  // input
                             curr_crystal, curr_crystal_id,                             // input
                             buffer_data, all_data);                                    // output
            } else {
              InitRayOtherMs(init_data, curr_ray_num, curr_crystal, curr_crystal_id,  // input
                             buffer_data, all_data, init_ray_offset);                 // output
            }

            // 2. Start tracing
            float refractive_index = curr_crystal.GetRefractiveIndex(wl);
            for (size_t i = 0; i < config.max_hits_; i++) {
              // 2.1 Trace rays. Deal with d, p, w, fid
              TraceRayBasicInfo(curr_crystal, refractive_index, curr_ray_num,  // input
                                buffer_data);                                  // output
              // After tracing, ray data will be in buffer_data[1], and there are curr_ray_num * 2 rays.

              // 2.2 Fill some information: crystal_id, rp, prev_ray_idx, root_ray_idx
              FillRayOtherInfo(curr_ray_num, i, curr_crystal, all_data,  // input
                               buffer_data);                             // output

              // 2.3 Collect data. And set ray properties: state
              auto filter = Filter::Create(s.filter_);
              filter->InitCrystalSymmetry(curr_crystal);
              CollectData(rng_, m, filter.get(),    // input
                          buffer_data, init_data);  // output

              // 2.4 Copy to all_data
              all_data.EmplaceBack(buffer_data[1]);
              CHECK_STOP
            }  // hit loop
            CHECK_STOP
          }  // small batch loop
          CHECK_STOP
        }  // crystal loop
        CHECK_STOP

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
      CHECK_STOP

      SimData sim_data;
      sim_data.curr_wl_ = wl;
      sim_data.total_intensity_ = curr_wl_param.weight_ * config.ray_num_;
      sim_data.crystals_ = std::move(all_crystals);
      sim_data.rays_ = std::move(all_data);
      data_queue_->Emplace(std::move(sim_data));

      CHECK_STOP
    }  // wl_param loop
    CHECK_STOP
    idle_ = true;
  }
}

void Simulator::Stop() {
  stop_ = true;
  config_queue_->Shutdown();
  data_queue_->Shutdown();
}

bool Simulator::IsIdle() const {
  return !stop_ && idle_;
}

}  // namespace v3
}  // namespace icehalo
