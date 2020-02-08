#include "simulation.h"

#include <cstdio>
#include <stack>
#include <utility>

#include "mymath.h"
#include "obj_pool.h"
#include "threadingpool.h"

namespace icehalo {

void SimulationRayData::Clear() {
  rays_.clear();
  exit_ray_segments_.clear();
  final_ray_segments_.clear();
  RayInfoPool::GetInstance()->Clear();
}


void SimulationRayData::PrepareNewScatter(size_t ray_num) {
  rays_.emplace_back();
  rays_.back().reserve(ray_num);
  exit_ray_segments_.emplace_back();
  exit_ray_segments_.back().reserve(ray_num * 2);
}


void SimulationRayData::AddRay(RayInfo* ray) {
  rays_.back().emplace_back(ray);
}


void SimulationRayData::AddExitRaySegment(RaySegment* r) {
  exit_ray_segments_.back().emplace_back(r);
}


void SimulationRayData::CollectFinalRaySegments() {
  final_ray_segments_.clear();
  for (const auto& sr : exit_ray_segments_) {
    for (const auto& r : sr) {
      if (r->state == RaySegmentState::kFinished) {
        final_ray_segments_.emplace_back(r);
      }
    }
  }
}


const std::vector<RaySegment*>& SimulationRayData::GetFinalRaySegments() const {
  return final_ray_segments_;
}


const std::vector<RaySegment*>& SimulationRayData::GetLastExitRaySegments() const {
  return exit_ray_segments_.back();
}


#ifdef FOR_TEST
const std::vector<std::vector<RaySegment*>>& SimulationRayData::GetExitRaySegments() const {
  return exit_ray_segments_;
}
#endif


Simulator::BufferData::BufferData()
    : pt{ nullptr }, dir{ nullptr }, w{ nullptr }, face_id{ nullptr }, ray_seg{ nullptr }, ray_num(0) {}


Simulator::BufferData::~BufferData() {
  Clear();
}


void Simulator::BufferData::Clear() {
  for (int i = 0; i < 2; i++) {
    DeleteBuffer(i);
  }
  ray_num = 0;
}


void Simulator::BufferData::DeleteBuffer(int idx) {
  delete[] pt[idx];
  delete[] dir[idx];
  delete[] w[idx];
  delete[] face_id[idx];
  delete[] ray_seg[idx];

  pt[idx] = nullptr;
  dir[idx] = nullptr;
  w[idx] = nullptr;
  face_id[idx] = nullptr;
  ray_seg[idx] = nullptr;
}


void Simulator::BufferData::Allocate(size_t ray_number) {
  for (int i = 0; i < 2; i++) {
    auto tmp_pt = new float[ray_number * 3];
    auto tmp_dir = new float[ray_number * 3];
    auto tmp_w = new float[ray_number];
    auto tmp_face_id = new int[ray_number];
    auto tmp_ray_seg = new RaySegment*[ray_number];

    if (pt[i]) {
      size_t n = std::min(this->ray_num, ray_number);
      std::memcpy(tmp_pt, pt[i], sizeof(float) * 3 * n);
      std::memcpy(tmp_dir, dir[i], sizeof(float) * 3 * n);
      std::memcpy(tmp_w, w[i], sizeof(float) * n);
      std::memcpy(tmp_face_id, face_id[i], sizeof(int) * n);
      std::memcpy(tmp_ray_seg, ray_seg[i], sizeof(void*) * n);

      DeleteBuffer(i);
    }

    pt[i] = tmp_pt;
    dir[i] = tmp_dir;
    w[i] = tmp_w;
    face_id[i] = tmp_face_id;
    ray_seg[i] = tmp_ray_seg;
  }
  this->ray_num = ray_number;
}


void Simulator::BufferData::Print() {
  std::printf("pt[0]                    dir[0]                   w[0]\n");
  for (decltype(ray_num) i = 0; i < ray_num; i++) {
    std::printf("%+.4f,%+.4f,%+.4f  ", pt[0][i * 3 + 0], pt[0][i * 3 + 1], pt[0][i * 3 + 2]);
    std::printf("%+.4f,%+.4f,%+.4f  ", dir[0][i * 3 + 0], dir[0][i * 3 + 1], dir[0][i * 3 + 2]);
    std::printf("%+.4f\n", w[0][i]);
  }

  std::printf("pt[1]                    dir[1]                   w[1]\n");
  for (decltype(ray_num) i = 0; i < ray_num; i++) {
    std::printf("%+.4f,%+.4f,%+.4f  ", pt[1][i * 3 + 0], pt[1][i * 3 + 1], pt[1][i * 3 + 2]);
    std::printf("%+.4f,%+.4f,%+.4f  ", dir[1][i * 3 + 0], dir[1][i * 3 + 1], dir[1][i * 3 + 2]);
    std::printf("%+.4f\n", w[1][i]);
  }
}


Simulator::EntryRayData::EntryRayData() : ray_dir(nullptr), ray_seg(nullptr), ray_num(0), buf_size(0) {}


Simulator::EntryRayData::~EntryRayData() {
  delete[] ray_dir;
  delete[] ray_seg;
}


void Simulator::EntryRayData::Clear() {
  std::fill(ray_dir, ray_dir + buf_size * 3, 0.0f);
  std::fill(ray_seg, ray_seg + buf_size, nullptr);
  ray_num = 0;
}


void Simulator::EntryRayData::Allocate(size_t ray_number) {
  if (ray_number > buf_size) {
    delete[] ray_dir;
    delete[] ray_seg;

    ray_dir = new float[ray_number * 3]{};
    ray_seg = new RaySegment* [ray_number] {};

    buf_size = ray_number;
  } else {
    Clear();
  }

  ray_num = ray_number;
}


Simulator::Simulator(ProjectContextPtr context)
    : context_(std::move(context)), simulation_ray_data_{}, current_wavelength_index_(-1), total_ray_num_(0),
      active_ray_num_(0), buffer_size_(0), buffer_{}, entry_ray_data_{}, entry_ray_offset_(0) {}


void Simulator::SetCurrentWavelengthIndex(int index) {
  if (index < 0 || static_cast<size_t>(index) >= context_->wavelengths_.size()) {
    current_wavelength_index_ = -1;
    return;
  }

  current_wavelength_index_ = index;
}


// Start simulation
void Simulator::Run() {
  simulation_ray_data_.Clear();
  RaySegmentPool::GetInstance()->Clear();
  RayInfoPool::GetInstance()->Clear();
  entry_ray_data_.Clear();
  entry_ray_offset_ = 0;

  if (current_wavelength_index_ < 0) {
    std::fprintf(stderr, "Warning! wavelength is not set!");
    return;
  }
  simulation_ray_data_.wavelength_info_ = context_->wavelengths_[current_wavelength_index_];

  InitSunRays();

  for (auto it = context_->multi_scatter_info_.begin(); it != context_->multi_scatter_info_.end(); ++it) {
    simulation_ray_data_.PrepareNewScatter(total_ray_num_);

    for (const auto& c : it->GetCrystalInfo()) {
      active_ray_num_ = static_cast<size_t>(c.population * total_ray_num_);
      if (buffer_size_ < total_ray_num_ * kBufferSizeFactor) {
        buffer_size_ = total_ray_num_ * kBufferSizeFactor;
        buffer_.Allocate(buffer_size_);
      }
      InitEntryRays(c.crystal_ctx);
      entry_ray_offset_ += active_ray_num_;
      TraceRays(c.crystal_ctx->crystal.get(), c.filter);
    }

    if (it != context_->multi_scatter_info_.end() - 1) {
      PrepareMultiScatterRays(it->GetProbability());  // total_ray_num_ is updated.
    }
    entry_ray_offset_ = 0;
  }

  simulation_ray_data_.CollectFinalRaySegments();
}


// Init sun rays, and fill into dir[1]. They will be rotated and fill into dir[0] in InitEntryRays().
// In world frame.
void Simulator::InitSunRays() {
  using math::RandomSampler;

  total_ray_num_ = context_->GetInitRayNum();
  float sun_r = context_->sun_ctx_->GetSunDiameter() / 2;  // In degree
  const float* sun_ray_dir = context_->sun_ctx_->GetSunPosition();
  if (entry_ray_data_.ray_num < total_ray_num_) {
    entry_ray_data_.Allocate(total_ray_num_);
  }

  RandomSampler::SampleSphericalPointsCart(sun_ray_dir, sun_r, entry_ray_data_.ray_dir, entry_ray_data_.ray_num);
  for (size_t i = 0; i < entry_ray_data_.ray_num; i++) {
    entry_ray_data_.ray_seg[i] = nullptr;
  }
}


// Init entry rays into a crystal. Fill pt[0], face_id[0], w[0] and ray_seg[0].
// Rotate entry rays into crystal frame
// Add RayContext and main axis rotation
void Simulator::InitEntryRays(const CrystalContext* ctx) {
  auto& crystal = ctx->crystal;
  auto total_faces = crystal->TotalFaces();

  auto* prob = ctx->face_prob_buf.get();
  const auto* face_norm = crystal->GetFaceNorm();
  const auto* face_vertex = crystal->GetFaceVertex();
  const auto* face_area = crystal->GetFaceArea();

  auto ray_pool = RaySegmentPool::GetInstance();
  auto ray_info_pool = RayInfoPool::GetInstance();

  using math::RandomSampler;
  float axis_rot[3];
  for (size_t i = 0; i < active_ray_num_; i++) {
    InitMainAxis(ctx, axis_rot);
    math::RotateZ(axis_rot, entry_ray_data_.ray_dir + (i + entry_ray_offset_) * 3, buffer_.dir[0] + i * 3);

    float sum = 0;
    for (int k = 0; k < total_faces; k++) {
      prob[k] = 0;
      if (!std::isnan(face_norm[k * 3 + 0]) && face_area[k] > 0) {
        prob[k] = std::max(-math::Dot3(face_norm + k * 3, buffer_.dir[0] + i * 3) * face_area[k], 0.0f);
        sum += prob[k];
      }
    }
    for (int k = 0; k < total_faces; k++) {
      prob[k] /= sum;
    }

    buffer_.face_id[0][i] = RandomSampler::SampleInt(prob, total_faces);
    RandomSampler::SampleTriangularPoints(face_vertex + buffer_.face_id[0][i] * 9, buffer_.pt[0] + i * 3);

    auto prev_r = entry_ray_data_.ray_seg[entry_ray_offset_ + i];
    buffer_.w[0][i] = prev_r ? prev_r->w : 1.0f;

    auto r = ray_pool->GetObject(buffer_.pt[0] + i * 3, buffer_.dir[0] + i * 3, buffer_.w[0][i], buffer_.face_id[0][i]);
    buffer_.ray_seg[0][i] = r;
    r->root_ctx = ray_info_pool->GetObject(r, ctx->crystal.get(), axis_rot);
    r->root_ctx->prev_ray_segment = prev_r;
    simulation_ray_data_.AddRay(r->root_ctx);
  }
}


// Init crystal main axis.
// Random sample points on a sphere with given parameters.
void Simulator::InitMainAxis(const CrystalContext* ctx, float* axis) {
  auto rng = math::RandomNumberGenerator::GetInstance();

  if (ctx->axis.latitude_dist == math::Distribution::kUniform) {
    // Random sample on full sphere, ignore other parameters.
    math::RandomSampler::SampleSphericalPointsSph(axis);
  } else {
    math::RandomSampler::SampleSphericalPointsSph(ctx->axis, axis);
  }

  if (ctx->axis.roll_dist == math::Distribution::kUniform) {
    // Random roll, ignore other parameters.
    axis[2] = rng->GetUniform() * 2 * math::kPi;
  } else {
    axis[2] = rng->Get(ctx->axis.roll_dist, ctx->axis.roll_mean, ctx->axis.roll_std) * math::kDegreeToRad;
  }
}


// Restore and shuffle resulted rays, and fill into dir[0].
void Simulator::PrepareMultiScatterRays(float prob) {
  auto last_exit_ray_seg_num = simulation_ray_data_.GetLastExitRaySegments().size();
  if (buffer_size_ < last_exit_ray_seg_num * 2) {
    buffer_size_ = last_exit_ray_seg_num * 2;
    buffer_.Allocate(buffer_size_);
  }
  if (entry_ray_data_.ray_num < last_exit_ray_seg_num) {
    entry_ray_data_.Allocate(last_exit_ray_seg_num);
  }

  auto rng = math::RandomNumberGenerator::GetInstance();
  size_t idx = 0;
  for (const auto& r : simulation_ray_data_.GetLastExitRaySegments()) {
    if (r->w < context_->kScatMinW) {
      r->state = RaySegmentState::kAirAbsorbed;
      continue;
    }
    if (rng->GetUniform() > prob) {
      continue;
    }
    r->state = RaySegmentState::kContinued;
    const auto axis_rot = r->root_ctx->main_axis.val();
    math::RotateZBack(axis_rot, r->dir.val(), entry_ray_data_.ray_dir + idx * 3);
    entry_ray_data_.ray_seg[idx] = r;
    idx++;
  }
  total_ray_num_ = idx;

  // Shuffle
  for (size_t i = 0; i < total_ray_num_; i++) {
    int tmp_idx = math::RandomSampler::SampleInt(static_cast<int>(total_ray_num_ - i));

    float tmp_dir[3];
    std::memcpy(tmp_dir, entry_ray_data_.ray_dir + (i + tmp_idx) * 3, sizeof(float) * 3);
    std::memcpy(entry_ray_data_.ray_dir + (i + tmp_idx) * 3, entry_ray_data_.ray_dir + i * 3, sizeof(float) * 3);
    std::memcpy(entry_ray_data_.ray_dir + i * 3, tmp_dir, sizeof(float) * 3);

    RaySegment* tmp_r = entry_ray_data_.ray_seg[i + tmp_idx];
    entry_ray_data_.ray_seg[i + tmp_idx] = entry_ray_data_.ray_seg[i];
    entry_ray_data_.ray_seg[i] = tmp_r;
  }
}


// Trace rays.
// Start from dir[0] and pt[0].
void Simulator::TraceRays(const Crystal* crystal, AbstractRayPathFilter* filter) {
  auto pool = ThreadingPool::GetInstance();

  int max_recursion_num = context_->GetRayHitNum();
  auto n = static_cast<float>(IceRefractiveIndex::Get(simulation_ray_data_.wavelength_info_.wavelength));
  for (int i = 0; i < max_recursion_num; i++) {
    if (buffer_size_ < active_ray_num_ * 2) {
      buffer_size_ = active_ray_num_ * kBufferSizeFactor;
      buffer_.Allocate(buffer_size_);
    }
    pool->AddRangeBasedJobs(active_ray_num_, [=](size_t idx0, size_t idx1) {
      size_t current_num = idx1 - idx0;
      Optics::HitSurface(crystal, n, current_num,                                                       //
                         buffer_.dir[0] + idx0 * 3, buffer_.face_id[0] + idx0, buffer_.w[0] + idx0,     //
                         buffer_.dir[1] + idx0 * 6, buffer_.w[1] + idx0 * 2);                           //
      Optics::Propagate(crystal, current_num * 2, buffer_.pt[0] + idx0 * 3,                             //
                        buffer_.dir[1] + idx0 * 6, buffer_.w[1] + idx0 * 2, buffer_.face_id[0] + idx0,  //
                        buffer_.pt[1] + idx0 * 6, buffer_.face_id[1] + idx0 * 2);                       //
    });
    pool->WaitFinish();
    StoreRaySegments(crystal, filter);
    RefreshBuffer();  // active_ray_num_ is updated.
  }
}


// Save rays
void Simulator::StoreRaySegments(const Crystal* crystal, AbstractRayPathFilter* filter) {
  filter->ApplySymmetry(crystal);
  auto ray_pool = RaySegmentPool::GetInstance();
  for (size_t i = 0; i < active_ray_num_ * 2; i++) {
    if (buffer_.w[1][i] <= 0) {  // Refractive rays in total reflection case
      continue;
    }

    auto r = ray_pool->GetObject(buffer_.pt[0] + i / 2 * 3, buffer_.dir[1] + i * 3, buffer_.w[1][i],
                                 buffer_.face_id[0][i / 2]);
    if (buffer_.face_id[1][i] < 0) {
      r->state = RaySegmentState::kFinished;
    }
    if (r->w < ProjectContext::kPropMinW) {
      r->state = RaySegmentState::kCrystalAbsorbed;
    }

    auto prev_ray_seg = buffer_.ray_seg[0][i / 2];
    if (i % 2 == 0) {
      prev_ray_seg->next_reflect = r;
    } else {
      prev_ray_seg->next_refract = r;
    }
    r->prev = prev_ray_seg;
    r->root_ctx = prev_ray_seg->root_ctx;
    buffer_.ray_seg[1][i] = r;

    if (r->state == RaySegmentState::kFinished && filter->Filter(crystal, r)) {
      simulation_ray_data_.AddExitRaySegment(r);
    }
  }
}


// Squeeze data, copy into another buffer_ (from buf[1] to buf[0])
// Update active_ray_num_.
void Simulator::RefreshBuffer() {
  size_t idx = 0;
  for (size_t i = 0; i < active_ray_num_ * 2; i++) {
    if (buffer_.face_id[1][i] >= 0 && buffer_.w[1][i] > ProjectContext::kPropMinW) {
      std::memcpy(buffer_.pt[0] + idx * 3, buffer_.pt[1] + i * 3, sizeof(float) * 3);
      std::memcpy(buffer_.dir[0] + idx * 3, buffer_.dir[1] + i * 3, sizeof(float) * 3);
      buffer_.w[0][idx] = buffer_.w[1][i];
      buffer_.face_id[0][idx] = buffer_.face_id[1][i];
      buffer_.ray_seg[0][idx] = buffer_.ray_seg[1][i];
      idx++;
    }
  }
  active_ray_num_ = idx;
}


const SimulationRayData& Simulator::GetSimulationRayData() {
  return simulation_ray_data_;
}


void Simulator::SaveFinalDirections(const char* filename) {
  File file(context_->GetDataDirectory().c_str(), filename);
  if (!file.Open(openmode::kWrite | openmode::kBinary)) {
    return;
  }

  if (current_wavelength_index_ < 0 || current_wavelength_index_ >= static_cast<int>(context_->wavelengths_.size())) {
    return;
  }

  file.Write(static_cast<float>(simulation_ray_data_.wavelength_info_.wavelength));
  file.Write(simulation_ray_data_.wavelength_info_.weight);

  const auto& ray_seg_set = simulation_ray_data_.GetFinalRaySegments();
  auto ray_num = ray_seg_set.size();
  size_t idx = 0;
  std::unique_ptr<float[]> data{ new float[ray_num * 4] };  // dx, dy, dz, w

  float* curr_data = data.get();
  for (const auto& r : ray_seg_set) {
    const auto axis_rot = r->root_ctx->main_axis.val();
    assert(r->root_ctx);

    math::RotateZBack(axis_rot, r->dir.val(), curr_data);
    curr_data[3] = r->w;
    curr_data += 4;
    idx++;
  }
  file.Write(data.get(), idx * 4);
}


#ifdef FOR_TEST
void Simulator::PrintRayInfo() {
  std::stack<RaySegment*> s;
  for (const auto& rs : simulation_ray_data_.GetExitRaySegments()) {
    for (const auto& r : rs) {
      auto p = r;
      while (p) {
        s.push(p);
        p = p->prev;
      }
      std::printf("%zu,0,0,0,0,0,-1\n", s.size());
      while (!s.empty()) {
        p = s.top();
        s.pop();
        std::printf("%+.4f,%+.4f,%+.4f,%+.4f,%+.4f,%+.4f,%+.4f\n",  //
                    p->pt.x(), p->pt.y(), p->pt.z(),                // point
                    p->dir.x(), p->dir.y(), p->dir.z(),             // direction
                    p->w);                                          // weight
      }
    }
  }
}
#endif

}  // namespace icehalo
