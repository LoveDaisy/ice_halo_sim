#include "core/simulator.hpp"

#include "core/buffer.hpp"
#include "core/filter.hpp"
#include "core/optics.hpp"
#include "util/log.hpp"
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
void InitRay_d_w_previdx(const LightSourceConfig& light_config, size_t ray_num,  // input
                         RayBuffer* ray_buf_ptr) {                               // output
  if (!ray_buf_ptr) {
    return;
  }
  const auto& ray_buf = *ray_buf_ptr;
  const auto& light_param = light_config.param_;
  float w0 = light_config.wl_param_[0].weight_;

  // w, prev_ray_idx: set init weight & previous ray index
  for (auto& r : ray_buf) {
    r.w_ = w0;
    r.prev_ray_idx_ = kInfSize;
  }

  // d: sample direction
  std::visit(RayDirSampler{ ray_buf[0].d_, ray_num, sizeof(RaySeg) }, light_param);
}


struct CrystalSampler {
  Crystal operator()(const PrismCrystalParam& p) {
    auto* rng = RandomNumberGenerator::GetInstance();
    float h = rng->Get(p.h_);
    // TODO: face distance
    return Crystal::CreatePrism(h);
  }

  Crystal operator()(const PyramidCrystalParam& /* p */) {
    // TODO:
    return Crystal{};
  }
};


std::vector<Crystal> SampleMsCrystal(const SceneConfig& config) {
  size_t total = 0;
  for (const auto& m : config.ms_) {
    total += m.setting_.size();
  }
  auto* rng = RandomNumberGenerator::GetInstance();
  std::vector<Crystal> crystals;
  crystals.reserve(total);

  for (const auto& m : config.ms_) {
    for (const auto& c : m.setting_) {
      // Sample current scattering crystals
      crystals.emplace_back(std::visit(CrystalSampler{}, c.crystal_.param_));

      // Set axis orientation
      {
        float lon = rng->Get(c.crystal_.axis_.azimuth_dist) * math::kDegreeToRad;
        float lat = rng->Get(c.crystal_.axis_.latitude_dist) * math::kDegreeToRad;
        if (lat > math::kPi_2) {
          lat = math::kPi - lat;
        }
        float roll = rng->Get(c.crystal_.axis_.roll_dist) * math::kDegreeToRad;
        float axis[9]{ 1, 0, 0, 0, 1, 0, 0, 0, 1 };
        auto rot = Rotation(axis + 6, roll).Chain(axis + 3, math::kPi_2 - lat).Chain(axis + 6, lon);
        crystals.back().Rotate(rot);
      }

      // Set origin
      // NOTE: Since we do **NOT** support streetlight cases, we ignore the initialization for origin.
    }
  }
  return crystals;
}


std::unique_ptr<size_t[]> PartitionCrystalRayNum(const MsInfo& ms_info, int ray_num) {
  auto crystal_cnt = ms_info.setting_.size();
  std::unique_ptr<size_t[]> c_num{ new size_t[crystal_cnt]{} };
  std::unique_ptr<float[]> prob{ new float[crystal_cnt + 1]{} };
  for (size_t ci = 0; ci < crystal_cnt; ci++) {
    prob[ci + 1] = ms_info.setting_[ci].crystal_proportion_ + prob[ci];
  }
  for (size_t ci = 0; ci < crystal_cnt; ci++) {
    prob[ci] /= prob[crystal_cnt];
  }

  auto* rng = RandomNumberGenerator::GetInstance();
  for (int i = 0; i < ray_num; i++) {
    auto u = rng->GetUniform();
    for (size_t ci = 0; ci < crystal_cnt; ci++) {
      if (prob[ci] <= u && u < prob[ci + 1]) {
        c_num[ci]++;
        break;
      }
    }
  }

  return c_num;
}

void TraceRays(const Crystal& curr_crystal, float refractive_index, size_t curr_ray_num, RayBuffer* buffer_data) {
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

  buffer_data[0].size_ = 0;
  buffer_data[1].size_ = curr_ray_num * 2;
}

void CollectData(const MsInfo& ms_info, const Filter* filter,     // input
                 RayBuffer* buffer_data, RayBuffer* init_data) {  // output
  auto* rng = RandomNumberGenerator::GetInstance();
  for (size_t j = 0; j < buffer_data[1].size_; j++) {
    auto& r = buffer_data[1][j];
    if (r.w_ < 0) {
      // 0. Total reflection.
      r.state_ = RaySeg::kStopped;
    } else if (r.fid_ < 0) {
      // 1. Outgoing rays.
      if (!filter->Check(r)) {
        // 1.1 Filter out. Marked as stopped.
        r.state_ = RaySeg::kStopped;
      } else if (rng->GetUniform() < ms_info.prob_) {
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
    : config_queue_(config_queue), data_queue_(data_queue), stop_(false), idle_(true) {}

Simulator::Simulator(Simulator&& other)
    : config_queue_(std::move(other.config_queue_)), data_queue_(std::move(other.data_queue_)),
      stop_(other.stop_.load()), idle_(other.idle_.load()) {}

Simulator& Simulator::operator=(Simulator&& other) {
  if (this == &other) {
    return *this;
  }

  config_queue_ = std::move(other.config_queue_);
  data_queue_ = std::move(other.data_queue_);
  stop_ = other.stop_.load();
  idle_ = other.idle_.load();
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

    LOG_DEBUG("Simulator::Run: get config(%u): ray(%zu), wl(%.1f,%.2f)", config.id_, config.ray_num_,
              config.light_source_.wl_param_[0].wl_, config.light_source_.wl_param_[0].weight_);

    idle_ = false;
    auto ms_crystals = SampleMsCrystal(config);

    float wl = config.light_source_.wl_param_[0].wl_;  // Take first wl **ONLY**. Single wl in a single run.

    // For memory saving, it's better to keep ray_num small.
    // Actually, if ms_prob = 1.0, i.e. all outgoing rays will be sent to next crystal,
    // then the final ray number for init_data will be (num0 * (max_hits + 1)^ms_num),
    // which increase rapidily.
    size_t ray_num = config.ray_num_;

    // Calculate total rays (expected value) used in the whole simulation.
    // total = n * (2 * k + 1) * (1 + k * p1 + k^2 * p1 * p2 + k^3 * p1 * p2 * p3 + ...)
    // where k = max_hits
    size_t total_ray_num = ray_num * (2 * config.max_hits_ + 1);
    {
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
    }

    RayBuffer all_data(total_ray_num);
    RayBuffer init_data[2]{ RayBuffer(), RayBuffer(ray_num * config.max_hits_) };
    RayBuffer buffer_data[2]{};

    bool first_ms = true;
    size_t ms_ci = 0;
    for (const auto& m : config.ms_) {
      auto ms_crystal_cnt = m.setting_.size();
      auto crystal_ray_num = PartitionCrystalRayNum(m, ray_num);

      // NOTE: ray_num will change between scatterings.
      buffer_data[0].Reset(ray_num * 2);
      buffer_data[1].Reset(ray_num * 2);

      size_t init_ray_offset = 0;
      for (size_t ci = 0; ci < ms_crystal_cnt; ci++) {
        const auto& curr_crystal = ms_crystals[ms_ci + ci];
        float refractive_index = curr_crystal.GetRefractiveIndex(wl);
        auto curr_ray_num = crystal_ray_num[ci];

        // 1. Initialize data
        {
          buffer_data[0].size_ = 0;
          size_t all_data_idx = all_data.size_;

          // 1.1 init d & w & (prev_ray_idx)
          if (first_ms) {
            buffer_data[0].size_ = curr_ray_num;
            InitRay_d_w_previdx(config.light_source_, curr_ray_num, buffer_data + 0);
          } else {
            buffer_data[0].EmplaceBack(init_data[0], init_ray_offset, curr_ray_num);
            init_ray_offset += curr_ray_num;
          }

          // 1.2 init p & fid
          InitRay_p_fid(curr_crystal, buffer_data + 0);

          // 1.3 init crystal_id, root_ray_idx, rp, state
          for (auto& r : buffer_data[0]) {
            const auto& c = ms_crystals[ms_ci + ci];
            r.crystal_id_ = ms_ci + ci;
            r.root_ray_idx_ = all_data_idx++;
            r.state_ = RaySeg::kNormal;
            r.rp_ << c.GetFn(r.fid_);
          }

          all_data.EmplaceBack(buffer_data[0]);
        }

        // 2. Start tracing
        for (size_t i = 0; i < config.max_hits_; i++) {
          // 2.1 Trace rays. Deal with d, p, w, fid
          TraceRays(curr_crystal, refractive_index, curr_ray_num, buffer_data);
          // After tracing, ray data will be in buffer_data[1], and there are curr_ray_num * 2 rays.

          // 2.2 Fill some information: crystal_id, rp, prev_ray_idx, root_ray_idx
          {
            size_t ray_id_offset = all_data.size_;
            for (size_t j = 0; j < buffer_data[1].size_; j++) {
              auto& r = buffer_data[1][j];
              const auto& c = ms_crystals[ms_ci + ci];
              r.rp_ << c.GetFn(r.fid_);
              r.crystal_id_ = ms_ci + ci;

              if (i == 0) {
                r.prev_ray_idx_ = j / 2 + ray_id_offset - curr_ray_num;
              } else if (i == 1) {
                r.prev_ray_idx_ = j / 2 * 2 + 1 + ray_id_offset - curr_ray_num * 2;
              } else {
                r.prev_ray_idx_ = j / 2 * 2 + 0 + ray_id_offset - curr_ray_num * 2;
              }

              r.root_ray_idx_ = all_data[r.prev_ray_idx_].root_ray_idx_;
            }
          }

          // 2.3 Collect data. And set ray properties: state
          auto filter = Filter::Create(m.setting_[ci].filter_);
          filter->InitCrystalSymmetry(curr_crystal);
          CollectData(m, filter.get(), buffer_data, init_data);

          // 2.4 Copy to all_data
          all_data.EmplaceBack(buffer_data[1]);
          CHECK_STOP
        }  // hit loop
        CHECK_STOP
      }  // crystal loop
      CHECK_STOP

      ms_ci += ms_crystal_cnt;
      ray_num = init_data[1].size_;
      init_data[0].Reset(ray_num * (config.max_hits_ + 1));
      std::swap(init_data[0], init_data[1]);

      // Shuffle init_data
      auto* rng = RandomNumberGenerator::GetInstance();
      for (size_t i = 0; i < init_data[0].size_; i++) {
        size_t j = static_cast<size_t>(rng->GetUniform() * (init_data[0].size_ - i)) + i;
        std::swap(init_data[0][i], init_data[0][j]);
      }

      first_ms = false;
    }  // ms loop
    CHECK_STOP

    SimData sim_data;
    sim_data.curr_wl_ = wl;
    sim_data.total_intensity_ = config.light_source_.wl_param_[0].weight_ * config.ray_num_;
    sim_data.crystals_ = std::move(ms_crystals);
    sim_data.rays_ = std::move(all_data);
    data_queue_->Emplace(std::move(sim_data));

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
