#include "simulation.h"
#include "mymath.h"
#include "threadingpool.h"

#include <stack>
#include <cstdio>

namespace IceHalo {

SimulationBufferData::SimulationBufferData()
    : pt{nullptr}, dir{nullptr}, w{nullptr}, face_id{nullptr}, ray_seg{nullptr}, ray_num(0) {}


SimulationBufferData::~SimulationBufferData() {
  Clean();
}


void SimulationBufferData::Clean() {
  for (int i = 0; i < 2; i++) {
    DeleteBuffer(i);
  }
}


void SimulationBufferData::DeleteBuffer(int idx) {
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


void SimulationBufferData::Allocate(size_t ray_num) {
  for (int i = 0; i < 2; i++) {
    auto tmp_pt = new float[ray_num * 3];
    auto tmp_dir = new float[ray_num * 3];
    auto tmp_w = new float[ray_num];
    auto tmp_face_id = new int[ray_num];
    auto tmp_ray_seg = new RaySegment*[ray_num];

    if (pt[i]) {
      size_t n = std::min(this->ray_num, ray_num);
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
  this->ray_num = ray_num;
}


void SimulationBufferData::Print() {
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


Simulator::Simulator(const SimulationContextPtr& context)
    : context_(context), total_ray_num_(0), active_ray_num_(0), buffer_size_(0) {}


// Start simulation
void Simulator::Start() {
  context_->FillActiveCrystal(&active_crystal_ctxs_);
  total_ray_num_ = context_->GetTotalInitRays();
  auto multi_scatter_times = context_->GetMultiScatterTimes();

  rays_.clear();
  final_ray_segments_.clear();
  RaySegmentPool::GetInstance()->Clear();

  for (int i = 0; i < multi_scatter_times; i++) {
    rays_.emplace_back();

    for (const auto& ctx : active_crystal_ctxs_) {
      active_ray_num_ = static_cast<size_t>(ctx->GetPopulation() * total_ray_num_);
      if (buffer_size_ < active_ray_num_ * kBufferSizeFactor) {
        buffer_size_ = active_ray_num_ * kBufferSizeFactor;
        buffer_.Allocate(buffer_size_);
      }
      if (i == 0) {
        InitSunRays();
      }
      InitEntryRays(ctx, i);   // total_ray_num_ may be updated
      TraceRays(ctx->GetCrystal());
    }
    if (i < multi_scatter_times - 1) {
      RestoreResultRays(i);    // total_ray_num_ is updated.
    }
  }
}


// Init sun rays, and fill into dir[1]. They will be rotated and fill into dir[0] in InitEntryRays().
// In world frame.
void Simulator::InitSunRays() {
  float sun_r = context_->GetSunDiameter() / 2;   // In degree
  const float* sun_ray_dir = context_->GetSunRayDir();
  auto sampler = Math::RandomSampler::GetInstance();
  sampler->SampleSphericalPointsCart(sun_ray_dir, sun_r, buffer_.dir[1], active_ray_num_);
}


// Init entry rays into a crystal. Fill pt[0], face_id[0], w[0] and ray_seg[0].
// Rotate entry rays into crystal frame
// Add RayPtr and main axis rotation
void Simulator::InitEntryRays(const CrystalContextPtr& ctx, int multi_scatter_idx) {
  auto crystal = ctx->GetCrystal();
  auto total_faces = crystal->TotalFaces();

  auto* face_area = new float[total_faces];
  auto* face_norm = new float[total_faces * 3];
  auto* prob = new float[total_faces];
  auto* face_point = crystal->GetFaceVertex();

  crystal->CopyFaceAreaData(face_area);
  crystal->CopyNormData(face_norm);

  rays_[multi_scatter_idx].reserve(active_ray_num_);

  auto ray_pool = RaySegmentPool::GetInstance();
  auto rng = Math::RandomNumberGenerator::GetInstance();
  auto sampler = Math::RandomSampler::GetInstance();

  float axis_rot[3];
  for (decltype(active_ray_num_) i = 0; i < active_ray_num_; i++) {
    InitMainAxis(ctx, axis_rot);
    Math::RotateZ(axis_rot, buffer_.dir[1] + i * 3, buffer_.dir[0] + i * 3);

    float sum = 0;
    for (int k = 0; k < total_faces; k++) {
      prob[k] = std::max(-Math::Dot3(face_norm + k * 3, buffer_.dir[0] + i * 3) * face_area[k], 0.0f);
      sum += prob[k];
    }
    for (int k = 0; k < total_faces; k++) {
      prob[k] /= sum;
    }

    buffer_.face_id[0][i] = sampler->SampleInt(prob, total_faces);
    sampler->SampleTriangularPoints(face_point + buffer_.face_id[0][i] * 9, buffer_.pt[0] + i * 3);

    buffer_.w[0][i] = 1.0f;

    auto r = ray_pool->GetRaySegment(buffer_.pt[0] + i * 3, buffer_.dir[0] + i * 3, buffer_.w[0][i],
                                     buffer_.face_id[0][i]);
    buffer_.ray_seg[0][i] = r;
    r->root_ = new Ray(r, ctx, axis_rot);
    rays_[multi_scatter_idx].emplace_back(r->root_);
  }

  delete[] face_area;
  delete[] face_norm;
  delete[] prob;
}


// Init crystal main axis.
// Random sample points on a sphere with given parameters.
void Simulator::InitMainAxis(const CrystalContextPtr& ctx, float* axis) {
  auto rng = Math::RandomNumberGenerator::GetInstance();
  auto sampler = Math::RandomSampler::GetInstance();

  if (ctx->GetAxisDist() == Math::Distribution::UNIFORM) {
    // Random sample on full sphere, ignore other parameters.
    sampler->SampleSphericalPointsSph(axis);
  } else {
    sampler->SampleSphericalPointsSph(ctx->GetAxisDist(), ctx->GetAxisMean(), ctx->GetAxisStd(), axis);
  }
  if (ctx->GetRollDist() == Math::Distribution::UNIFORM) {
    // Random roll, ignore other parameters.
    axis[2] = rng->GetUniform() * 2 * Math::kPi;
  } else {
    axis[2] = rng->Get(ctx->GetRollDist(), ctx->GetRollMean(), ctx->GetRollStd()) * Math::kDegreeToRad;
  }
}


// Restore and shuffle resulted rays, and fill into dir[0].
void Simulator::RestoreResultRays(int multi_scatter_idx) {
  final_ray_segments_.clear();

  auto rng = Math::RandomNumberGenerator::GetInstance();
  std::stack<RaySegment*> s;
  size_t idx = 0;
  for (auto& r : rays_[multi_scatter_idx]) {
    s.push(r->first_ray_segment_);
    while (!s.empty()) {
      auto tmp_r = s.top();
      s.pop();

      if (tmp_r->is_finished_ && tmp_r->w_ > SimulationContext::kScatMinW &&
          rng->GetUniform() < context_->GetMultiScatterProb()) {
        assert(tmp_r->root_);
        const auto axis_rot = tmp_r->root_->main_axis_rot_.val();
        Math::RotateZBack(axis_rot, tmp_r->dir_.val(), buffer_.dir[0] + idx * 3);
        idx++;
      } else {
        if (tmp_r->next_reflect_) {
          s.push(tmp_r->next_reflect_);
        }
        if (tmp_r->next_refract_) {
          s.push(tmp_r->next_refract_);
        }
      }
    }
  }
  total_ray_num_ = idx;

  // Shuffle
  auto sampler = Math::RandomSampler::GetInstance();
  float tmp_dir[3];
  for (decltype(total_ray_num_) i = 0; i < total_ray_num_; i++) {
    int tmp_idx = sampler->SampleInt(static_cast<int>(total_ray_num_ - i));
    std::memcpy(tmp_dir, buffer_.dir[0] + (i + tmp_idx) * 3, sizeof(float) * 3);
    std::memcpy(buffer_.dir[0] + i * 3, tmp_dir, sizeof(float) * 3);
    std::memcpy(buffer_.dir[0] + (i + tmp_idx) * 3, buffer_.dir[0] + i * 3, sizeof(float) * 3);
  }
}


// Trace rays.
// Start from dir[0] and pt[0].
void Simulator::TraceRays(const CrystalPtr& crystal) {
  auto pool = ThreadingPool::GetInstance();

  int max_recursion_num = context_->GetMaxRecursionNum();
  float n = IceRefractiveIndex::n(context_->GetCurrentWavelength());
  for (int i = 0; i < max_recursion_num; i++) {
    if (buffer_size_ < active_ray_num_ * 2) {
      buffer_size_ = active_ray_num_ * kBufferSizeFactor;
      buffer_.Allocate(buffer_size_);
    }
    auto step = std::max(active_ray_num_ / 100, static_cast<size_t>(10));
    for (decltype(active_ray_num_) j = 0; j < active_ray_num_; j += step) {
      decltype(active_ray_num_) current_num = std::min(active_ray_num_ - j, step);
      pool->AddJob([=] {
        Optics::HitSurface(crystal, n, current_num,
                           buffer_.dir[0] + j * 3, buffer_.face_id[0] + j, buffer_.w[0] + j,
                           buffer_.dir[1] + j * 6, buffer_.w[1] + j * 2);
        Optics::Propagate(crystal, current_num * 2,
                          buffer_.pt[0] + j * 3, buffer_.dir[1] + j * 6, buffer_.w[1] + j * 2, buffer_.face_id[0] + j,
                          buffer_.pt[1] + j * 6, buffer_.face_id[1] + j * 2);
      });
    }
    pool->WaitFinish();
    StoreRaySegments();
    RefreshBuffer();    // active_ray_num_ is updated.
  }
}


// Save rays
void Simulator::StoreRaySegments() {
  auto ray_pool = RaySegmentPool::GetInstance();
  for (size_t i = 0; i < active_ray_num_ * 2; i++) {
    if (buffer_.w[1][i] <= 0) {   // Refractive rays in total reflection case
      continue;
    }

    auto r = ray_pool->GetRaySegment(
      buffer_.pt[0] + i / 2 * 3, buffer_.dir[1] + i * 3, buffer_.w[1][i], buffer_.face_id[0][i / 2]);
    if (buffer_.face_id[1][i] < 0) {
      r->is_finished_ = true;
    }
    if (r->is_finished_ || r->w_ < SimulationContext::kPropMinW) {
      final_ray_segments_.emplace_back(r);
    }

    auto prev_ray_seg = buffer_.ray_seg[0][i / 2];
    if (i % 2 == 0) {
      prev_ray_seg->next_reflect_ = r;
    } else {
      prev_ray_seg->next_refract_ = r;
    }
    r->prev_ = prev_ray_seg;
    r->root_ = prev_ray_seg->root_;
    buffer_.ray_seg[1][i] = r;
  }
}


// Squeeze data, copy into another buffer_ (from buf[1] to buf[0])
// Update active_ray_num_.
void Simulator::RefreshBuffer() {
  size_t idx = 0;
  for (size_t i = 0; i < active_ray_num_ * 2; i++) {
    if (buffer_.face_id[1][i] >= 0 && buffer_.w[1][i] > SimulationContext::kPropMinW) {
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


void Simulator::SaveFinalDirections(const char* filename) {
  File file(context_->GetDataDirectory().c_str(), filename);
  if (!file.Open(OpenMode::kWrite | OpenMode::kBinary)) return;

  file.Write(context_->GetCurrentWavelength());

  auto ray_num = final_ray_segments_.size();
  size_t idx = 0;
  auto* data = new float[ray_num * 4];       // dx, dy, dz, w

  float* curr_data = data;
  for (const auto& r : final_ray_segments_) {
    assert(r->root_);
    if (!r->root_->crystal_ctx_->FilterRay(r)) {
      continue;
    }

    const auto axis_rot = r->root_->main_axis_rot_.val();
    Math::RotateZBack(axis_rot, r->dir_.val(), curr_data);
    curr_data[3] = r->w_;
    curr_data += 4;
    idx++;
  }
  file.Write(data, idx * 4);
  file.Close();

  delete[] data;
}


void Simulator::PrintRayInfo() {
  std::stack<RaySegment*> s;
  for (const auto& rs : rays_) {
    for (const auto& r : rs) {
      s.push(r->first_ray_segment_);

      while (!s.empty()) {
        auto p = s.top();
        s.pop();
        if (p->next_refract_ && !p->is_finished_) {
          s.push(p->next_refract_);
        }
        if (p->next_reflect_ && !p->is_finished_) {
          s.push(p->next_reflect_);
        }
        if (!p->next_reflect_ && !p->next_refract_ && p->IsValidEnd()) {
          std::stack<RaySegment*> tmp_stack;
          tmp_stack.push(p);
          while (p->prev_) {
            tmp_stack.push(p->prev_);
            p = p->prev_;
          }

          std::printf("%li,0,0,0,0,0,-1\n", tmp_stack.size());
          while (!tmp_stack.empty()) {
            p = tmp_stack.top();
            tmp_stack.pop();
            std::printf("%+.4f,%+.4f,%+.4f,%+.4f,%+.4f,%+.4f,%+.4f\n",
              p->pt_.x(), p->pt_.y(), p->pt_.z(),
              p->dir_.x(), p->dir_.y(), p->dir_.z(),
              p->w_);
          }
        }
      }
    }
  }
}

}  // namespace IceHalo
