#include "simulation.hpp"

#include <cstdio>
#include <functional>
#include <stack>
#include <utility>

#include "core/mymath.hpp"
#include "util/log.hpp"
#include "util/obj_pool.hpp"
#include "util/threadingpool.hpp"

namespace icehalo {

SimpleRayData::SimpleRayData(size_t num)
    : wavelength(0), wavelength_weight(1.0f), buf{ new float[num * 4] }, buf_ray_num(num), init_ray_num(0) {}


void SimpleRayData::Serialize(File& file, bool with_boi) const {
  if (with_boi) {
    file.Write(ISerializable::kDefaultBoi);
  }

  file.Write(wavelength);
  file.Write(wavelength_weight);
  file.Write(static_cast<uint64_t>(buf_ray_num));
  file.Write(static_cast<uint64_t>(init_ray_num));

  float* p = buf.get();
  for (size_t i = 0; i < buf_ray_num; i++) {
    file.Write(p, 4);
    p += 4;
  }
}


void SimpleRayData::Deserialize(File& file, endian::Endianness endianness) {
  endianness = CheckEndianness(file, endianness);
  bool need_swap = (endianness != endian::kCompileEndian);

  file.Read(&wavelength);
  if (need_swap) {
    endian::ByteSwap::Swap(&wavelength);
  }
  file.Read(&wavelength_weight);
  if (need_swap) {
    endian::ByteSwap::Swap(&wavelength_weight);
  }

  uint64_t num;
  file.Read(&num);
  if (need_swap) {
    endian::ByteSwap::Swap(&num);
  }
  buf_ray_num = num;

  file.Read(&num);
  if (need_swap) {
    endian::ByteSwap::Swap(&num);
  }
  init_ray_num = num;

  buf.reset(new float[buf_ray_num * 4]);
  float* p = buf.get();
  for (size_t i = 0; i < buf_ray_num; i++) {
    file.Read(p, 4);
    if (need_swap) {
      endian::ByteSwap::Swap(p, 4);
    }
    p += 4;
  }
}


void SimulationData::Clear() {
  rays_.clear();
  exit_ray_segments_.clear();
  exit_ray_seg_num_.clear();
  ray_path_map_.clear();
  wavelength_info_ = {};
}


void SimulationData::PrepareNewScatter(size_t ray_num) {
  rays_.emplace_back();
  rays_.back().reserve(ray_num);
  exit_ray_segments_.emplace_back();
  exit_ray_segments_.back().reserve(ray_num * 2);
  exit_ray_seg_num_.emplace_back(0);
}


void SimulationData::AddRay(RayInfo* ray) {
  rays_.back().emplace_back(ray);
}


void SimulationData::AddExitRaySegment(RaySegment* r) {
  exit_ray_segments_.back().emplace_back(r);
  if (r->state == RaySegmentState::kFinished) {
    exit_ray_seg_num_.back()++;
  }
}


std::pair<RayCollectionInfo, SimpleRayData> SimulationData::CollectFinalRayData() {
  size_t num = 0;
  for (const auto& n : exit_ray_seg_num_) {
    num += n;
  }

  SimpleRayData final_ray_data(num);
  RayCollectionInfo collection_info{};
  if (!rays_.empty()) {
    collection_info.init_ray_num = rays_[0].size();
    final_ray_data.init_ray_num = rays_[0].size();
  }
  final_ray_data.wavelength = wavelength_info_.wavelength;
  final_ray_data.wavelength_weight = wavelength_info_.weight;
  float* p = final_ray_data.buf.get();
  for (const auto& sr : exit_ray_segments_) {
    for (const auto& r : sr) {
      if (r->state != RaySegmentState::kFinished) {
        continue;
      }
      const auto* axis = r->root_ctx->main_axis.val();
      math::RotateZBack(axis, r->dir.val(), p);
      p[3] = r->w;
      collection_info.total_energy += r->w;
      p += 4;
    }
  }
  return std::make_pair(std::move(collection_info), std::move(final_ray_data));
}


std::tuple<RayCollectionInfoList, SimpleRayData> SimulationData::CollectSplitRayData(const ProjectContextPtr& ctx,
                                                                                     const RenderSplitter& splitter) {
  switch (splitter.type) {
    case RenderSplitterType::kNone:
      return {};
    case RenderSplitterType::kTopHalo:
      return CollectSplitHaloRayData(ctx);
    case RenderSplitterType::kFilter:
      return CollectSplitFilterRayData(ctx, splitter);
  }
}


std::tuple<RayCollectionInfoList, SimpleRayData> SimulationData::CollectSplitHaloRayData(const ProjectContextPtr& ctx) {
  size_t num = 0;
  for (const auto& n : exit_ray_seg_num_) {
    num += n;
  }

  SimpleRayData result_ray_data(num);
  result_ray_data.buf_ray_num = num;
  result_ray_data.wavelength = wavelength_info_.wavelength;
  result_ray_data.wavelength_weight = wavelength_info_.weight;
  if (!rays_.empty()) {
    result_ray_data.init_ray_num = rays_[0].size();
  }
  float* result_buf_p = result_ray_data.buf.get();

  MakeRayPathMap(ctx);

  size_t hash_table_init_size = num / 3;
  std::unordered_map<size_t, RayCollectionInfo> ray_collection_info_map;
  ray_collection_info_map.reserve(hash_table_init_size);

  std::unordered_map<size_t, std::unordered_set<RayInfo*>> ray_info_map;
  ray_info_map.reserve(hash_table_init_size);

  std::vector<RayPathRecorder*> recorder_list{};
  recorder_list.reserve(ctx->multi_scatter_info_.size());
  RayPathRecorder recorder;
  size_t ray_seg_idx = 0;
  for (const auto& sr : exit_ray_segments_) {
    for (const auto& r : sr) {
      if (r->state != RaySegmentState::kFinished) {
        continue;
      }

      // 1. Get hash for the whole path
      recorder_list.clear();
      RaySegment* p = r;
      while (p) {
        recorder_list.emplace_back(&(p->recorder));
        p = p->root_ctx->prev_ray_segment;
      }
      recorder.Clear();
      for (auto it = recorder_list.rbegin(); it != recorder_list.rend(); ++it) {
        recorder << **it;
      }
      auto ray_path_hash = recorder.Hash();

      // 3. Fill data
      auto normalized_hash = ray_path_map_.at(ray_path_hash).second;
      if (!ray_collection_info_map.count(normalized_hash)) {
        RayCollectionInfo tmp_collection{};
        tmp_collection.identifier = normalized_hash;
        tmp_collection.is_partial_data = true;
        ray_collection_info_map.emplace(normalized_hash, std::move(tmp_collection));
      }
      if (!ray_info_map.count(normalized_hash)) {
        std::unordered_set<RayInfo*> tmp_set{};
        ray_info_map.emplace(normalized_hash, std::move(tmp_set));
      }
      auto& collection_info = ray_collection_info_map[normalized_hash];
      auto& ray_set = ray_info_map[normalized_hash];
      collection_info.idx.emplace_back(ray_seg_idx);
      collection_info.total_energy += r->w;
      ray_set.emplace(r->root_ctx);
      collection_info.init_ray_num = ray_set.size();

      // 4. Fill in result_ray_data
      const auto* axis = r->root_ctx->main_axis.val();
      math::RotateZBack(axis, r->dir.val(), result_buf_p);
      result_buf_p[3] = r->w;
      result_buf_p += 4;

      ray_seg_idx++;
    }
  }

  // 5. Make result
  RayCollectionInfoList ray_collection_info_list{};
  for (auto& kv : ray_collection_info_map) {
    ray_collection_info_list.emplace_back(std::move(kv.second));
  }

  // 6. Sort
  std::sort(ray_collection_info_list.begin(), ray_collection_info_list.end(),
            [](const RayCollectionInfo& a, const RayCollectionInfo& b) { return a.total_energy > b.total_energy; });

  // return the final result
  return std::make_tuple(std::move(ray_collection_info_list), std::move(result_ray_data));
}


std::tuple<RayCollectionInfoList, SimpleRayData> SimulationData::CollectSplitFilterRayData(
    const ProjectContextPtr& ctx, const RenderSplitter& splitter) {
  size_t num = 0;
  for (const auto& n : exit_ray_seg_num_) {
    num += n;
  }

  auto ms_num = ctx->multi_scatter_info_.size();

  // 1. prepare crystal filters
  std::vector<std::vector<std::pair<const CrystalContext*, RayPathFilterPtrU>>> crystal_filter;
  for (const auto& cf : splitter.crystal_filters) {
    crystal_filter.emplace_back();
    const CrystalContext* cp = nullptr;
    AbstractRayPathFilter* fp = nullptr;
    for (size_t i = 0; i < std::min(ms_num * 2, cf.size()); i += 2) {
      cp = ctx->GetCrystalContext(cf[i]);
      fp = ctx->GetRayPathFilter(cf[i + 1]);
      crystal_filter.back().emplace_back(std::make_pair(cp, fp->MakeCopy()));
      crystal_filter.back().back().second->ApplySymmetry(cp);
    }
  }

  // 2. prepare some data storage.
  SimpleRayData result_ray_data(num);
  result_ray_data.buf_ray_num = num;
  result_ray_data.wavelength = wavelength_info_.wavelength;
  result_ray_data.wavelength_weight = wavelength_info_.weight;
  if (!rays_.empty()) {
    result_ray_data.init_ray_num = rays_[0].size();
  }
  float* result_buf_p = result_ray_data.buf.get();

  std::vector<RayCollectionInfo> ray_collection_info_list(crystal_filter.size());
  for (size_t i = 0; i < ray_collection_info_list.size(); i++) {
    ray_collection_info_list[i].identifier = i;
    ray_collection_info_list[i].is_partial_data = true;
  }
  std::vector<std::unordered_set<RayInfo*>> ray_info_map(crystal_filter.size());
  MakeRayPathMap(ctx);

  // 3. start filter all rays
  size_t ray_seg_idx = 0;
  for (size_t ms_idx = 0; ms_idx < exit_ray_segments_.size(); ms_idx++) {
    for (const auto& r : exit_ray_segments_[ms_idx]) {
      if (r->state != RaySegmentState::kFinished) {
        continue;
      }

      // 4. filter
      for (size_t i = 0; i < crystal_filter.size(); i++) {
        const auto& cf = crystal_filter[i];
        if (cf.size() != ms_idx + 1) {
          continue;
        }
        RaySegment* p = r;
        RayInfo* ray_info = r->root_ctx;
        bool pass = true;
        for (auto riter = cf.rbegin(); riter != cf.rend(); ++riter) {
          ray_info = p->root_ctx;
          if (p->root_ctx->crystal_id != riter->first->GetId()) {
            pass = false;
            break;
          }
          auto curr_crystal = riter->first->GetCrystal();
          if (!riter->second->Filter(curr_crystal, p)) {
            pass = false;
            break;
          }
          p = p->root_ctx->prev_ray_segment;
        }

        if (!pass) {
          continue;
        }

        // 6. add to collection
        auto& collection = ray_collection_info_list[i];
        collection.idx.emplace_back(ray_seg_idx);
        collection.total_energy += r->w;
        ray_info_map[i].emplace(ray_info);
        collection.init_ray_num = ray_info_map[i].size();
        break;
      }

      // 7. Fill in result_ray_data
      const auto* axis = r->root_ctx->main_axis.val();
      math::RotateZBack(axis, r->dir.val(), result_buf_p);
      result_buf_p[3] = r->w;
      result_buf_p += 4;

      ray_seg_idx++;
    }
  }

  return std::make_tuple(std::move(ray_collection_info_list), std::move(result_ray_data));
}


const std::vector<RaySegment*>& SimulationData::GetLastExitRaySegments() const {
  return exit_ray_segments_.back();
}


void SimulationData::MakeRayPathMap(const ProjectContextPtr& proj_ctx) {
  if (!ray_path_map_.empty()) {
    return;
  }

  std::vector<RayPathRecorder*> recorder_list{};
  recorder_list.reserve(proj_ctx->multi_scatter_info_.size());
  RayPathRecorder recorder;
  for (const auto& sr : exit_ray_segments_) {
    for (const auto& r : sr) {
      if (r->state != RaySegmentState::kFinished) {
        continue;
      }

      // 1. Get hash for the whole path
      recorder_list.clear();
      RaySegment* p = r;
      while (p) {
        recorder_list.emplace_back(&(p->recorder));
        p = p->root_ctx->prev_ray_segment;
      }
      recorder.Clear();
      for (auto it = recorder_list.rbegin(); it != recorder_list.rend(); ++it) {
        recorder << **it;
      }
      auto ray_path_hash = recorder.Hash();
      if (ray_path_map_.count(ray_path_hash)) {
        continue;
      }

      // 2. Normalize ray path
      auto curr_path = proj_ctx->GetRayPath(r);
      RayPath normalized_path;
      size_t normalized_hash;
      std::tie(normalized_path, normalized_hash) =
          NormalizeRayPath(curr_path, proj_ctx, RenderSplitter::kDefaultSymmetry);
      ray_path_map_.emplace(ray_path_hash, std::make_pair(curr_path, normalized_hash));
      if (!ray_path_map_.count(normalized_hash)) {
        ray_path_map_.emplace(normalized_hash, std::make_pair(normalized_path, normalized_hash));
      }
    }
  }
}


#ifdef FOR_TEST
const std::vector<std::vector<RaySegment*>>& SimulationData::GetExitRaySegments() const {
  return exit_ray_segments_;
}
#endif


void SimulationData::Serialize(File& file, bool with_boi) const {
  if (with_boi) {
    file.Write(ISerializable::kDefaultBoi);
  }

  int32_t wl = wavelength_info_.wavelength;
  file.Write(wl);
  file.Write(wavelength_info_.weight);

  auto ray_info_pool = RayInfoPool::GetInstance();
  auto ray_seg_pool = RaySegmentPool::GetInstance();
  ray_info_pool->Serialize(file, false);
  ray_seg_pool->Serialize(file, false);

  uint32_t multi_scatters = rays_.size();
  file.Write(multi_scatters);
  for (const auto& sc : rays_) {
    uint32_t num = sc.size();
    file.Write(num);
    for (const auto& r : sc) {
      uint32_t chunk_id, obj_id;
      std::tie(chunk_id, obj_id) = ray_info_pool->GetObjectSerializeIndex(r);
      file.Write(chunk_id);
      file.Write(obj_id);
    }
  }
  multi_scatters = exit_ray_segments_.size();
  file.Write(multi_scatters);
  for (const auto& sc : exit_ray_segments_) {
    uint32_t num = sc.size();
    file.Write(num);
    for (const auto& r : sc) {
      uint32_t chunk_id, obj_id;
      std::tie(chunk_id, obj_id) = ray_seg_pool->GetObjectSerializeIndex(r);
      file.Write(chunk_id);
      file.Write(obj_id);
    }
  }
  for (const auto& n : exit_ray_seg_num_) {
    uint64_t num = n;
    file.Write(num);
  }
}


void SimulationData::Deserialize(File& file, endian::Endianness endianness) {
  endianness = CheckEndianness(file, endianness);
  bool need_swap = (endianness != endian::kCompileEndian);

  Clear();

  int32_t wl;
  file.Read(&wl);
  if (need_swap) {
    endian::ByteSwap::Swap(&wl);
  }
  wavelength_info_.wavelength = wl;

  file.Read(&wavelength_info_.weight);
  if (need_swap) {
    endian::ByteSwap::Swap(&wavelength_info_.weight);
  }

  auto ray_info_pool = RayInfoPool::GetInstance();
  auto ray_seg_pool = RaySegmentPool::GetInstance();
  ray_info_pool->Deserialize(file, endianness);
  ray_seg_pool->Deserialize(file, endianness);

  ray_info_pool->Map([=](RayInfo& r) {
    r.first_ray_segment = ray_seg_pool->GetPointerFromSerializeData(r.first_ray_segment);
    r.prev_ray_segment = ray_seg_pool->GetPointerFromSerializeData(r.prev_ray_segment);
  });
  ray_seg_pool->Map([=](RaySegment& r) {
    r.next_reflect = ray_seg_pool->GetPointerFromSerializeData(r.next_reflect);
    r.next_refract = ray_seg_pool->GetPointerFromSerializeData(r.next_refract);
    r.prev = ray_seg_pool->GetPointerFromSerializeData(r.prev);
    r.root_ctx = ray_info_pool->GetPointerFromSerializeData(r.root_ctx);
  });

  uint32_t multi_scatters;
  file.Read(&multi_scatters);
  if (need_swap) {
    endian::ByteSwap::Swap(&multi_scatters);
  }
  for (size_t k = 0; k < multi_scatters; k++) {
    rays_.emplace_back();
    uint32_t num;
    file.Read(&num);
    if (need_swap) {
      endian::ByteSwap::Swap(&num);
    }
    for (size_t i = 0; i < num; i++) {
      uint32_t chunk_id, obj_id;
      file.Read(&chunk_id);
      file.Read(&obj_id);
      if (need_swap) {
        endian::ByteSwap::Swap(&chunk_id);
        endian::ByteSwap::Swap(&obj_id);
      }
      rays_.back().emplace_back(ray_info_pool->GetPointerFromSerializeData(chunk_id, obj_id));
    }
  }

  file.Read(&multi_scatters);
  if (need_swap) {
    endian::ByteSwap::Swap(&multi_scatters);
  }
  for (size_t k = 0; k < multi_scatters; k++) {
    exit_ray_segments_.emplace_back();
    uint32_t num;
    file.Read(&num);
    if (need_swap) {
      endian::ByteSwap::Swap(&num);
    }
    for (size_t i = 0; i < num; i++) {
      uint32_t chunk_id, obj_id;
      file.Read(&chunk_id);
      file.Read(&obj_id);
      if (need_swap) {
        endian::ByteSwap::Swap(&chunk_id);
        endian::ByteSwap::Swap(&obj_id);
      }
      auto r = ray_seg_pool->GetPointerFromSerializeData(chunk_id, obj_id);
      exit_ray_segments_.back().emplace_back(r);
    }
  }
  for (size_t k = 0; k < multi_scatters; k++) {
    uint64_t n;
    file.Read(&n);
    if (need_swap) {
      endian::ByteSwap::Swap(&n);
    }
    exit_ray_seg_num_.emplace_back(n);
  }
}


Simulator::BufferData::BufferData() : pt{}, dir{}, w{}, face_id{}, ray_seg{}, ray_num(0) {}


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


#ifdef FOR_TEST
void Simulator::BufferData::Print() {
  LOG_DEBUG("pt[0]                    dir[0]                   w[0]");
  for (decltype(ray_num) i = 0; i < ray_num; i++) {
    LOG_DEBUG("%+.4f,%+.4f,%+.4f  %+.4f,%+.4f,%+.4f  %+.4f",            //
              pt[0][i * 3 + 0], pt[0][i * 3 + 1], pt[0][i * 3 + 2],     // pt
              dir[0][i * 3 + 0], dir[0][i * 3 + 1], dir[0][i * 3 + 2],  // dir
              w[0][i]);
  }

  LOG_DEBUG("pt[1]                    dir[1]                   w[1]");
  for (decltype(ray_num) i = 0; i < ray_num; i++) {
    LOG_DEBUG("%+.4f,%+.4f,%+.4f  %+.4f,%+.4f,%+.4f  %+.4f",            //
              pt[1][i * 3 + 0], pt[1][i * 3 + 1], pt[1][i * 3 + 2],     // pt
              dir[1][i * 3 + 0], dir[1][i * 3 + 1], dir[1][i * 3 + 2],  // dir
              w[1][i]);
  }
}
#endif


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
    LOG_INFO("NOTE! wavelength is not set!");
    return;
  }
  simulation_ray_data_.wavelength_info_ = context_->wavelengths_[current_wavelength_index_];

  InitSunRays();

  const auto& multi_scatter_info = context_->multi_scatter_info_;
  for (size_t i = 0; i < multi_scatter_info.size(); i++) {
    simulation_ray_data_.PrepareNewScatter(total_ray_num_);

    for (const auto& c : multi_scatter_info[i]->GetCrystalInfo()) {
      active_ray_num_ = static_cast<size_t>(c.population * total_ray_num_);
      if (buffer_size_ < total_ray_num_ * kBufferSizeFactor) {
        buffer_size_ = total_ray_num_ * kBufferSizeFactor;
        buffer_.Allocate(buffer_size_);
      }
      InitEntryRays(context_->GetCrystalContext(c.crystal_id));
      entry_ray_offset_ += active_ray_num_;
      TraceRays(context_->GetCrystalContext(c.crystal_id), context_->GetRayPathFilter(c.filter_id));
    }

    if (i != multi_scatter_info.size() - 1) {
      PrepareMultiScatterRays(multi_scatter_info[i]->GetProbability());  // total_ray_num_ is updated.
    }
    entry_ray_offset_ = 0;
  }
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
  const auto* crystal = ctx->GetCrystal();
  auto crystal_id = ctx->GetId();
  const auto* face_vertex = crystal->GetFaceVertex();

  auto ray_seg_pool = RaySegmentPool::GetInstance();
  auto ray_info_pool = RayInfoPool::GetInstance();

  using math::RandomSampler;
  float axis_rot[3];
  for (size_t i = 0; i < active_ray_num_; i++) {
    InitMainAxis(ctx, axis_rot);
    math::RotateZ(axis_rot, entry_ray_data_.ray_dir + (i + entry_ray_offset_) * 3, buffer_.dir[0] + i * 3);

    buffer_.face_id[0][i] = ctx->RandomSampleFace(buffer_.dir[0] + i * 3);
    RandomSampler::SampleTriangularPoints(face_vertex + buffer_.face_id[0][i] * 9, buffer_.pt[0] + i * 3);

    auto prev_r = entry_ray_data_.ray_seg[entry_ray_offset_ + i];
    buffer_.w[0][i] = prev_r ? prev_r->w : 1.0f;

    auto r =
        ray_seg_pool->GetObject(buffer_.pt[0] + i * 3, buffer_.dir[0] + i * 3, buffer_.w[0][i], buffer_.face_id[0][i]);
    buffer_.ray_seg[0][i] = r;
    r->root_ctx = ray_info_pool->GetObject(r, crystal_id, axis_rot);
    r->root_ctx->prev_ray_segment = prev_r;
    r->recorder << crystal_id;
    simulation_ray_data_.AddRay(r->root_ctx);
  }
}


// Init crystal main axis.
// Random sample points on a sphere with given parameters.
void Simulator::InitMainAxis(const CrystalContext* ctx, float* axis) {
  auto rng = math::RandomNumberGenerator::GetInstance();

  auto axis_dist = ctx->GetAxisDistribution();
  if (axis_dist.latitude_dist == math::Distribution::kUniform) {
    // Random sample on full sphere, ignore other parameters.
    math::RandomSampler::SampleSphericalPointsSph(axis);
  } else {
    math::RandomSampler::SampleSphericalPointsSph(axis_dist, axis);
  }

  if (axis_dist.roll_dist == math::Distribution::kUniform) {
    // Random roll, ignore other parameters.
    axis[2] = rng->GetUniform() * 2 * math::kPi;
  } else {
    axis[2] = rng->Get(axis_dist.roll_dist, axis_dist.roll_mean, axis_dist.roll_std) * math::kDegreeToRad;
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
void Simulator::TraceRays(const CrystalContext* crystal_ctx, AbstractRayPathFilter* filter) {
  auto pool = ThreadingPool::GetInstance();

  auto crystal = crystal_ctx->GetCrystal();
  int max_recursion_num = context_->GetRayHitNum();
  auto n = static_cast<float>(IceRefractiveIndex::Get(simulation_ray_data_.wavelength_info_.wavelength));
  for (int i = 0; i < max_recursion_num; i++) {
    if (buffer_size_ < active_ray_num_ * 2) {
      buffer_size_ = active_ray_num_ * kBufferSizeFactor;
      buffer_.Allocate(buffer_size_);
    }
    pool->AddStepMapJobs(active_ray_num_, [=](size_t idx0, size_t idx1, size_t step) {
      for (size_t i = idx0; i < idx1; i += step) {
        Optics::HitSurface(crystal, n, 1,                                                        //
                           buffer_.dir[0] + i * 3, buffer_.face_id[0] + i, buffer_.w[0] + i,     //
                           buffer_.dir[1] + i * 6, buffer_.w[1] + i * 2);                        //
        Optics::Propagate(crystal, 1 * 2, buffer_.pt[0] + i * 3,                                 //
                          buffer_.dir[1] + i * 6, buffer_.w[1] + i * 2, buffer_.face_id[0] + i,  //
                          buffer_.pt[1] + i * 6, buffer_.face_id[1] + i * 2);                    //
      }
    });
    pool->WaitFinish();
    StoreRaySegments(crystal_ctx, filter);
    RefreshBuffer();  // active_ray_num_ is updated.
  }
}


// Save rays
void Simulator::StoreRaySegments(const CrystalContext* crystal_ctx, AbstractRayPathFilter* filter) {
  auto crystal = crystal_ctx->GetCrystal();
  filter->ApplySymmetry(crystal_ctx);
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
    r->recorder = prev_ray_seg->recorder;
    r->recorder << crystal->FaceNumber(r->face_id);
    if (r->state == RaySegmentState::kFinished) {
      r->recorder << kInvalidFaceNumber;
    }
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
    if (buffer_.face_id[1][i] < 0 || buffer_.w[1][i] < ProjectContext::kPropMinW) {
      continue;
    }
    std::memcpy(buffer_.pt[0] + idx * 3, buffer_.pt[1] + i * 3, sizeof(float) * 3);
    std::memcpy(buffer_.dir[0] + idx * 3, buffer_.dir[1] + i * 3, sizeof(float) * 3);
    buffer_.w[0][idx] = buffer_.w[1][i];
    buffer_.face_id[0][idx] = buffer_.face_id[1][i];
    buffer_.ray_seg[0][idx] = buffer_.ray_seg[1][i];
    idx++;
  }
  active_ray_num_ = idx;
}


const SimulationData& Simulator::GetSimulationRayData() {
  return simulation_ray_data_;
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
      LOG_DEBUG("%zu,0,0,0,0,0,-1", s.size());
      while (!s.empty()) {
        p = s.top();
        s.pop();
        LOG_DEBUG("%+.4f,%+.4f,%+.4f,%+.4f,%+.4f,%+.4f,%+.4f",  //
                  p->pt.x(), p->pt.y(), p->pt.z(),              // point
                  p->dir.x(), p->dir.y(), p->dir.z(),           // direction
                  p->w);                                        // weight
      }
    }
  }
}
#endif

}  // namespace icehalo
