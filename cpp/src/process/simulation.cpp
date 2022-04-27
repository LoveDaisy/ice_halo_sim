#include "process/simulation.hpp"

#include <cstddef>
#include <cstdio>
#include <cstring>
#include <functional>
#include <memory>
#include <stack>
#include <utility>
#include <variant>

#include "core/buffer.hpp"
#include "core/crystal.hpp"
#include "core/def.hpp"
#include "core/math.hpp"
#include "core/optics.hpp"
#include "protocol/light_config.hpp"
#include "protocol/proj_config.hpp"
#include "protocol/sim_data.hpp"
#include "util/log.hpp"
#include "util/obj_pool.hpp"
#include "util/queue.hpp"
#include "util/threading_pool.hpp"

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

  uint64_t num = 0;
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


SimulationData::SimulationData() : wavelength_info_(), threading_pool_(ThreadingPool::CreatePool()) {}


void SimulationData::SetThreadingPool(ThreadingPoolPtr threading_pool) {
  threading_pool_ = std::move(threading_pool);
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


std::tuple<RayCollectionInfo, SimpleRayData> SimulationData::CollectFinalRayData() {
  size_t num = 0;
  for (const auto& n : exit_ray_seg_num_) {
    num += n;
  }

  SimpleRayData final_ray_data(num);
  RayCollectionInfo collection_info{};
  if (!rays_.empty()) {
    final_ray_data.init_ray_num = rays_[0].size();
  }
  final_ray_data.wavelength = wavelength_info_.wavelength;
  final_ray_data.wavelength_weight = wavelength_info_.weight;
  float* p = final_ray_data.buf.get();
  for (const auto& sr : exit_ray_segments_) {
    const auto* sr_data = sr.data();
    threading_pool_->CommitRangeStepJobsAndWait(0, sr.size(), [=](int /* thread_id */, int i) {
      const auto& r = sr_data[i];
      if (r->state != RaySegmentState::kFinished) {
        return;
      }
      const auto* axis = r->root_ctx->main_axis.val();
      RotateZBack(axis, r->dir.val(), p + i * 4);
      p[i * 4 + 3] = r->w;
    });
    for (const auto& r : sr) {
      if (r->state != RaySegmentState::kFinished) {
        continue;
      }
      collection_info.total_energy += r->w;
    }
    p += sr.size() * 4;
  }
  return std::make_pair(std::move(collection_info), std::move(final_ray_data));
}


std::tuple<RayCollectionInfoList, SimpleRayData> SimulationData::CollectSplitRayData(const ProjectContextPtr& ctx,
                                                                                     const RenderSplitter& splitter) {
  switch (splitter.type) {
    case RenderSplitterType::kNone:
      return std::make_tuple<RayCollectionInfoList, SimpleRayData>({}, SimpleRayData());
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

  MakeRayPathMap(ctx);
  auto idx_list = GenerateIdxList();

  auto threading_pool = ThreadingPool::CreatePool();
  size_t hash_table_init_size = num / 3;
  std::unordered_map<size_t, RayCollectionInfo> ray_collection_info_map;
  ray_collection_info_map.reserve(hash_table_init_size);

  for (size_t i = 0; i < exit_ray_segments_.size(); i++) {
    const auto& sr = exit_ray_segments_[i];
    const auto& curr_idx_list = idx_list[i];
    for (size_t j = 0; j < sr.size(); j++) {
      const auto& r = sr[j];
      if (r->state != RaySegmentState::kFinished) {
        continue;
      }

      // 1. Get hash for the whole path
      RayPathRecorder recorder;
      RaySegment* p = r;
      while (p) {
        p->recorder >> recorder;
        p = p->root_ctx->prev_ray_segment;
      }
      auto ray_path_hash = recorder.Hash();

      // 3. Fill collection data
      auto normalized_hash = ray_path_map_.at(ray_path_hash).second;
      if (!ray_collection_info_map.count(normalized_hash)) {
        RayCollectionInfo tmp_collection{};
        tmp_collection.identifier = normalized_hash;
        tmp_collection.is_partial_data = true;
        ray_collection_info_map.emplace(normalized_hash, std::move(tmp_collection));
      }
      auto& collection_info = ray_collection_info_map[normalized_hash];
      collection_info.idx.emplace_back(curr_idx_list[j]);
      collection_info.total_energy += r->w;
    }
  }

  // 4. Fill in result_ray_data
  SimpleRayData result_ray_data(num);
  result_ray_data.buf_ray_num = num;
  result_ray_data.wavelength = wavelength_info_.wavelength;
  result_ray_data.wavelength_weight = wavelength_info_.weight;
  if (!rays_.empty()) {
    result_ray_data.init_ray_num = rays_[0].size();
  }
  float* result_buf_p = result_ray_data.buf.get();

  for (size_t i = 0; i < exit_ray_segments_.size(); i++) {
    const auto& sr = exit_ray_segments_[i];
    threading_pool->CommitRangeStepJobsAndWait(0, sr.size(), [=, &idx_list, &sr](int /* thread_id */, int j) {
      const auto& r = sr[j];
      if (r->state != RaySegmentState::kFinished) {
        return;
      }

      auto* p = result_buf_p + idx_list[i][j] * 4;
      const auto* axis = r->root_ctx->main_axis.val();
      RotateZBack(axis, r->dir.val(), p);
      p[3] = r->w;
    });
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
    for (size_t i = 0; i < std::min(ms_num * 2, cf.size()); i += 2) {
      cp = ctx->GetCrystalContext(cf[i]);
      auto* fp = ctx->GetRayPathFilter(cf[i + 1]);
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
        bool pass = true;
        for (auto riter = cf.rbegin(); riter != cf.rend(); ++riter) {
          if (p->root_ctx->crystal_id != riter->first->GetId()) {
            pass = false;
            break;
          }
          const auto* curr_crystal = riter->first->GetCrystal();
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
        break;
      }

      // 7. Fill in result_ray_data
      const auto* axis = r->root_ctx->main_axis.val();
      RotateZBack(axis, r->dir.val(), result_buf_p);
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

  auto threading_pool = ThreadingPool::CreatePool();
  auto pool_size = threading_pool->GetPoolSize();
  std::vector<decltype(ray_path_map_)> tmp_ray_path_maps(pool_size);

  for (const auto& sr : exit_ray_segments_) {
    threading_pool->CommitRangeStepJobsAndWait(0, sr.size(), [=, &tmp_ray_path_maps, &sr](int pool_idx, int i) {
      auto& tmp_map = tmp_ray_path_maps.at(pool_idx);
      const auto& r = sr[i];
      if (r->state != RaySegmentState::kFinished) {
        return;
      }

      // 1. Get hash for the whole path
      RayPathRecorder recorder;
      RaySegment* p = r;
      while (p) {
        p->recorder >> recorder;
        p = p->root_ctx->prev_ray_segment;
      }
      auto ray_path_hash = recorder.Hash();
      if (tmp_map.count(ray_path_hash)) {
        return;
      }

      // 2. Normalize ray path
      auto curr_path = proj_ctx->GetRayPath(r);
      auto [normalized_path, normalized_hash] = NormalizeRayPath(curr_path, proj_ctx, RenderSplitter::kDefaultSymmetry);
      tmp_map.emplace(ray_path_hash, std::make_pair(std::move(curr_path), normalized_hash));
      tmp_map.emplace(normalized_hash, std::make_pair(std::move(normalized_path), normalized_hash));
    });
  }

  for (auto& map : tmp_ray_path_maps) {
    for (auto& kv : map) {
      ray_path_map_.emplace(std::move(kv));
    }
  }
}


std::vector<std::vector<size_t>> SimulationData::GenerateIdxList() const {
  size_t ray_seg_idx = 0;
  std::vector<std::vector<size_t>> idx_list;
  for (const auto& sr : exit_ray_segments_) {
    idx_list.emplace_back(std::vector<size_t>(sr.size()));
    for (size_t i = 0; i < sr.size(); i++) {
      idx_list.back()[i] = ray_seg_idx;
      const auto& r = sr[i];
      if (r->state != RaySegmentState::kFinished) {
        continue;
      }
      ray_seg_idx++;
    }
  }
  return idx_list;
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

  auto* ray_info_pool = RayInfoPool::GetInstance();
  auto* ray_seg_pool = RaySegmentPool::GetInstance();
  ray_info_pool->Serialize(file, false);
  ray_seg_pool->Serialize(file, false);

  uint32_t multi_scatters = rays_.size();
  file.Write(multi_scatters);
  for (const auto& sc : rays_) {
    uint32_t num = sc.size();
    file.Write(num);
    for (const auto& r : sc) {
      uint32_t chunk_id = 0;
      uint32_t obj_id = 0;
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
      uint32_t chunk_id = 0;
      uint32_t obj_id = 0;
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

  int32_t wl = 0;
  file.Read(&wl);
  if (need_swap) {
    endian::ByteSwap::Swap(&wl);
  }
  wavelength_info_.wavelength = wl;

  file.Read(&wavelength_info_.weight);
  if (need_swap) {
    endian::ByteSwap::Swap(&wavelength_info_.weight);
  }

  auto* ray_info_pool = RayInfoPool::GetInstance();
  auto* ray_seg_pool = RaySegmentPool::GetInstance();
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

  uint32_t multi_scatters = 0;
  file.Read(&multi_scatters);
  if (need_swap) {
    endian::ByteSwap::Swap(&multi_scatters);
  }
  for (size_t k = 0; k < multi_scatters; k++) {
    rays_.emplace_back();
    uint32_t num = 0;
    file.Read(&num);
    if (need_swap) {
      endian::ByteSwap::Swap(&num);
    }
    for (size_t i = 0; i < num; i++) {
      uint32_t chunk_id = 0;
      uint32_t obj_id = 0;
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
    uint32_t num = 0;
    file.Read(&num);
    if (need_swap) {
      endian::ByteSwap::Swap(&num);
    }
    for (size_t i = 0; i < num; i++) {
      uint32_t chunk_id = 0;
      uint32_t obj_id = 0;
      file.Read(&chunk_id);
      file.Read(&obj_id);
      if (need_swap) {
        endian::ByteSwap::Swap(&chunk_id);
        endian::ByteSwap::Swap(&obj_id);
      }
      auto* r = ray_seg_pool->GetPointerFromSerializeData(chunk_id, obj_id);
      exit_ray_segments_.back().emplace_back(r);
    }
  }
  for (size_t k = 0; k < multi_scatters; k++) {
    uint64_t n = 0;
    file.Read(&n);
    if (need_swap) {
      endian::ByteSwap::Swap(&n);
    }
    exit_ray_seg_num_.emplace_back(n);
  }
}


namespace v3 {

/**
 * @brief Sample on crystal & init origin (p & fid) of rays.
 *        NOTE: direction of rays should be given.
 */
void InitRayPFid(const Crystal& curr_crystal,  // input
                 RayBuffer* ray_buf_ptr) {     // output
  if (!ray_buf_ptr) {
    return;
  }

  RayBuffer& ray_buf = *ray_buf_ptr;
  // p & fid: sample on crystal faces
  std::unique_ptr<float[]> proj_prob{ new float[curr_crystal.TotalFaces()]{} };
  const auto* face_norm = curr_crystal.GetFaceNorm();
  const auto* face_area = curr_crystal.GetFaceArea();
  auto total_faces = curr_crystal.TotalFaces();
  for (size_t i = 0; i < ray_buf.size_; i++) {
    const auto* d = ray_buf[i].d_;
    for (size_t j = 0; j < total_faces; j++) {
      proj_prob[j] = std::max(-Dot3(d, face_norm + j * 3) * face_area[j], 0.0f);
    }
    // fid
    RandomSample(total_faces, proj_prob.get(), &ray_buf[i].fid_);
    auto fid = ray_buf[i].fid_;
    // p
    SampleTrianglePoint(curr_crystal.GetFaceVtx() + fid * 9, ray_buf[i].p_);
  }
}

/**
 * @brief Set initial value of d & w (direction and intensity) for rays.
 *
 */
void InitRayDW(const LightSourceConfig& light_config, size_t ray_num,  // input
               RayBuffer* ray_buf_ptr) {                               // output
  if (!ray_buf_ptr) {
    return;
  }
  const auto& ray_buf = *ray_buf_ptr;
  const auto& light_param = light_config.param_;
  float w0 = light_config.wl_param_[0].weight_;

  // w: set init weight.
  for (size_t i = 0; i < ray_num; i++) {
    ray_buf[i].w_ = w0;
  }

  // d: sample direction
  if (std::holds_alternative<SunParam>(light_param)) {
    const auto& param = std::get<SunParam>(light_param);
    SampleSphCapPoint(param.azimuth_ + 180.0f, -param.altitude_, param.diameter_ / 2.0f, ray_buf[0].d_, ray_num,
                      sizeof(RaySeg));
  } else if (std::holds_alternative<StreetLightParam>(light_param)) {
    LOG_WARNING("we will support street light later.");
    // TODO: current we do **NOT** support street light source
  } else {
    LOG_ERROR("unknown light source type!");
  }
}

std::vector<Crystal> SampleMsCrystal(const SceneConfig* config) {
  size_t total = 0;
  for (const auto& m : config->ms_) {
    total += m.setting_.size();
  }
  auto* rng = RandomNumberGenerator::GetInstance();
  std::vector<Crystal> crystals;
  crystals.reserve(total);

  for (const auto& m : config->ms_) {
    for (const auto& c : m.setting_) {
      // Sample current scattering crystals
      if (std::holds_alternative<PrismCrystalParam>(c.crystal_.param_)) {
        const auto& param = std::get<PrismCrystalParam>(c.crystal_.param_);
        float h = rng->Get(param.h_);
        crystals.emplace_back(Crystal::CreatePrism(h));
        // TODO: prism face distance
      } else if (std::holds_alternative<PyramidCrystalParam>(c.crystal_.param_)) {
        // TODO:
      }
      crystals.back().config_id_ = c.crystal_.id_;

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

      // TODO: Set origin
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

  for (size_t k = 0; k < curr_ray_num * 2; k++) {
    std::memcpy(buffer_data[0][k].d_, buffer_data[1][k].d_, 3 * sizeof(float));
    buffer_data[0][k].w_ = buffer_data[1][k].w_;
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

void CollectData(const MsInfo& ms_info, size_t ci,  //
                 RayBuffer* buffer_data, RayBuffer* init_data) {
  const auto& filter = ms_info.setting_[ci].filter_;  // TODO:
  auto* rng = RandomNumberGenerator::GetInstance();
  for (size_t j = 0; j < buffer_data[1].size_; j++) {
    auto& r = buffer_data[1][j];
    if (r.fid_ < 0) {
      //  a. Copy outgoing rays into initial data, with probability of p
      if (rng->GetUniform() < ms_info.prob_) {
        init_data[1].EmplaceBack(r);
      }
    } else {
      //  b. Squeeze (or better swap?) buffers.
      buffer_data[0].EmplaceBack(r);
    }
  }
}


Simulator::Simulator(QueuePtrS<SceneConfigPtrU> config_queue, QueuePtrS<SimDataPtrU> data_queue)
    : config_queue_(config_queue), data_queue_(data_queue), stop_(false) {}

void Simulator::Run() {
  while (true) {
    // Config in the config_queue is processed by frontend of simulator (NOT GUI) so that
    // it has small ray_num and contains only one wavelength parameter of light source.
    auto config = config_queue_->Get();  // Will block until get one
    if (stop_ || !config) {
      // If no data in the queue or recieve a terminal signal, abort simulation
      break;
    }

    auto ms_crystals = SampleMsCrystal(config.get());

    float wl = config->light_source_.wl_param_[0].wl_;  // Take first wl **ONLY**. Single wl in a single run.

    // For memory saving, it's better to keep ray_num small.
    // Actually, if ms_prob = 1.0, i.e. all outgoing rays will be sent to next crystal,
    // then the final ray number for init_data will be (num0 * (max_hits + 1)^ms_num),
    // which increase rapidily.
    size_t ray_num = config->ray_num_;

    // Calculate total rays (expected value) used in the whole simulation.
    // total = n * (2 * k + 1) * (1 + k * p1 + k^2 * p1 * p2 + k^3 * p1 * p2 * p3 + ...)
    // where k = max_hits
    size_t total_ray_num = ray_num * (2 * config->max_hits_ + 1);
    {
      float x = 1;
      float y = 1;
      float x_max = 1;
      float y_max = 1;
      for (size_t i = 0; i + 1 < config->ms_.size(); i++) {
        y *= config->max_hits_ * config->ms_[i].prob_;
        x += y;
        y_max *= config->max_hits_;
        x_max += y_max;
      }
      total_ray_num = static_cast<size_t>(total_ray_num * std::min(x * 1.5f, x_max));
    }

    RayBuffer all_data(total_ray_num);
    RayBuffer init_data[2]{ RayBuffer(), RayBuffer(ray_num * config->max_hits_) };
    RayBuffer buffer_data[2]{};

    bool first_ms = true;
    size_t ms_ci = 0;
    for (const auto& m : config->ms_) {
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
          if (first_ms) {
            InitRayDW(config->light_source_, curr_ray_num, buffer_data + 0);
            for (size_t i = 0; i < curr_ray_num; i++) {
              buffer_data[0][i].prev_ray_id_ = kInvalidId;
            }
            buffer_data[0].size_ = curr_ray_num;
          } else {
            buffer_data[0].EmplaceBack(init_data[0], init_ray_offset, curr_ray_num);
            init_ray_offset += curr_ray_num;
          }
          InitRayPFid(curr_crystal, buffer_data + 0);
          for (auto& r : buffer_data[0]) {
            const auto& c = ms_crystals[ms_ci + ci];
            r.crystal_id_ = ms_ci + ci;
            r.rp_ << r.crystal_id_ << c.GetFn(r.fid_);
          }
          all_data.EmplaceBack(buffer_data[0]);
        }

        // 2. Start tracing
        for (size_t i = 0; i < config->max_hits_; i++) {
          // 2.1 Trace rays.
          TraceRays(curr_crystal, refractive_index, curr_ray_num, buffer_data);
          // After tracing, ray data will be in buffer_data[1], and there are curr_ray_num * 2 rays.

          // 2.2 Fill some information
          {
            size_t ray_id_offset = all_data.size_;
            for (size_t j = 0; j < buffer_data[1].size_; j++) {
              auto& r = buffer_data[1][j];
              const auto& c = ms_crystals[ms_ci + ci];
              r.rp_ << c.GetFn(r.fid_);
              r.crystal_id_ = ms_ci + ci;

              if (i == 0) {
                r.prev_ray_id_ = j / 2 + ray_id_offset - curr_ray_num;
              } else if (i == 1) {
                r.prev_ray_id_ = j / 2 * 2 + 1 + ray_id_offset - curr_ray_num * 2;
              } else {
                r.prev_ray_id_ = j / 2 * 2 + 0 + ray_id_offset - curr_ray_num * 2;
              }
            }
          }

          // 2.3 Collect data.
          CollectData(m, ci, buffer_data, init_data);

          // 2.4 Copy to all_data
          all_data.EmplaceBack(buffer_data[1]);

          if (stop_) {
            break;
          }
        }  // hit loop

        if (stop_) {
          break;
        }
      }  // crystal loop

      if (stop_) {
        break;
      }

      ms_ci += ms_crystal_cnt;
      ray_num = init_data[1].size_;
      init_data[0].Reset(ray_num * (config->max_hits_ + 1));
      std::swap(init_data[0], init_data[1]);

      // Shuffle init_data
      auto* rng = RandomNumberGenerator::GetInstance();
      for (size_t i = 0; i < init_data[0].size_; i++) {
        size_t j = static_cast<size_t>(rng->GetUniform() * (init_data[0].size_ - i)) + i;
        std::swap(init_data[0][i], init_data[0][j]);
      }

      first_ms = false;
    }  // ms loop

    if (stop_) {
      break;
    }

    auto sim_data = std::make_unique<SimData>();
    sim_data->curr_wl_ = wl;
    sim_data->crystals_ = std::move(ms_crystals);
    sim_data->ray_buffer_ = std::move(all_data);
    data_queue_->Emplace(std::move(sim_data));

    if (stop_) {
      break;
    }
  }
}

void Simulator::Stop() {
  stop_ = true;
  config_queue_->Shutdown();
  data_queue_->Shutdown();
}

}  // namespace v3


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
    auto* tmp_pt = new float[ray_number * 3];
    auto* tmp_dir = new float[ray_number * 3];
    auto* tmp_w = new float[ray_number];
    auto* tmp_face_id = new int[ray_number];
    auto* tmp_ray_seg = new RaySegment*[ray_number];

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
    : context_(std::move(context)), threading_pool_(ThreadingPool::CreatePool()), current_wavelength_index_(-1),
      total_ray_num_(0), active_ray_num_(0), buffer_size_(0), entry_ray_offset_(0) {
  simulation_ray_data_.SetThreadingPool(threading_pool_);
}


void Simulator::SetCurrentWavelengthIndex(int index) {
  if (index < 0 || static_cast<size_t>(index) >= context_->wavelengths_.size()) {
    current_wavelength_index_ = -1;
    return;
  }

  current_wavelength_index_ = index;
}


void Simulator::SetThreadingPool(ThreadingPoolPtr threading_pool) {
  threading_pool_ = std::move(threading_pool);
  simulation_ray_data_.SetThreadingPool(threading_pool_);
}


// Start simulation
void Simulator::Run() {
#ifndef FOR_TEST
  if (context_->GetInitRayNum() < ProjectContext::kMinInitRayNum) {
    return;
  }
#endif

  simulation_ray_data_.Clear();
  RaySegmentPool::GetInstance()->Clear();
  RayInfoPool::GetInstance()->Clear();
  IdPool::GetInstance()->Clear();
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
        LOG_DEBUG("Allocate buffer in Run()");
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
  auto total_face = ctx->GetCrystal()->TotalFaces();

  auto* ray_seg_pool = RaySegmentPool::GetInstance();
  auto* ray_info_pool = RayInfoPool::GetInstance();

  std::unique_ptr<float[]> axis_rot{ new float[active_ray_num_ * 3] };
  auto* axis_rot_ptr = axis_rot.get();
  std::unique_ptr<float[]> face_prob_buf{ new float[total_face * active_ray_num_] };
  auto* face_prob_buf_ptr = face_prob_buf.get();
  threading_pool_->CommitRangeStepJobsAndWait(0, active_ray_num_, [=](int /* thread_id */, int i) {
    InitMainAxis(ctx, axis_rot_ptr + i * 3);
    RotateZ(axis_rot_ptr + i * 3, entry_ray_data_.ray_dir + (i + entry_ray_offset_) * 3, buffer_.dir[0] + i * 3);

    buffer_.face_id[0][i] = ctx->RandomSampleFace(buffer_.dir[0] + i * 3, face_prob_buf_ptr + i * total_face);
    RandomSampler::SampleTriangularPoints(face_vertex + buffer_.face_id[0][i] * 9, buffer_.pt[0] + i * 3);

    auto* prev_r = entry_ray_data_.ray_seg[entry_ray_offset_ + i];
    buffer_.w[0][i] = prev_r ? prev_r->w : 1.0f;
  });

  for (size_t i = 0; i < active_ray_num_; i++) {
    auto* prev_r = entry_ray_data_.ray_seg[entry_ray_offset_ + i];
    auto* r =
        ray_seg_pool->GetObject(buffer_.pt[0] + i * 3, buffer_.dir[0] + i * 3, buffer_.w[0][i], buffer_.face_id[0][i]);
    buffer_.ray_seg[0][i] = r;
    r->root_ctx = ray_info_pool->GetObject(r, crystal_id, axis_rot_ptr + i * 3);
    r->root_ctx->prev_ray_segment = prev_r;
    r->recorder << crystal_id;
    simulation_ray_data_.AddRay(r->root_ctx);
  }
}


// Init crystal main axis.
// Random sample points on a sphere with given parameters.
void Simulator::InitMainAxis(const CrystalContext* ctx, float* axis) {
  auto* rng = RandomNumberGenerator::GetInstance();

  auto axis_dist = ctx->GetAxisDistribution();
  if (axis_dist.latitude_dist.type == DistributionType::kUniform) {
    // Random sample on full sphere, ignore other parameters.
    RandomSampler::SampleSphericalPointsSph(axis);
  } else {
    RandomSampler::SampleSphericalPointsSph(axis_dist, axis);
  }

  if (axis_dist.roll_dist.type == DistributionType::kUniform) {
    // Random roll, ignore other parameters.
    axis[2] = rng->GetUniform() * 2 * math::kPi;
  } else {
    axis[2] = rng->Get(axis_dist.roll_dist) * math::kDegreeToRad;
  }
}


// Restore and shuffle resulted rays, and fill into dir[0].
void Simulator::PrepareMultiScatterRays(float prob) {
  auto last_exit_ray_seg_num = simulation_ray_data_.GetLastExitRaySegments().size();
  if (buffer_size_ < last_exit_ray_seg_num * 2) {
    LOG_DEBUG("Allocate buffer in PrepareMultiScatterRays()");
    buffer_size_ = last_exit_ray_seg_num * 2;
    buffer_.Allocate(buffer_size_);
  }
  if (entry_ray_data_.ray_num < last_exit_ray_seg_num) {
    entry_ray_data_.Allocate(last_exit_ray_seg_num);
  }

  auto* rng = RandomNumberGenerator::GetInstance();
  size_t idx = 0;
  for (const auto r : simulation_ray_data_.GetLastExitRaySegments()) {
    if (r->w < context_->kScatMinW) {
      r->state = RaySegmentState::kAirAbsorbed;
      continue;
    }
    if (rng->GetUniform() > prob) {
      continue;
    }
    r->state = RaySegmentState::kContinued;
    const auto* axis_rot = r->root_ctx->main_axis.val();
    RotateZBack(axis_rot, r->dir.val(), entry_ray_data_.ray_dir + idx * 3);
    entry_ray_data_.ray_seg[idx] = r;
    idx++;
  }
  total_ray_num_ = idx;

  // Shuffle
  for (size_t i = 0; i < total_ray_num_; i++) {
    int tmp_idx = RandomSampler::SampleInt(static_cast<int>(total_ray_num_ - i));

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
  const auto* crystal = crystal_ctx->GetCrystal();
  int max_recursion_num = context_->GetRayHitNum();
  auto n = static_cast<float>(IceRefractiveIndex::Get(simulation_ray_data_.wavelength_info_.wavelength));
  for (int i = 0; i < max_recursion_num; i++) {
    if (buffer_size_ < active_ray_num_ * 2) {
      LOG_DEBUG("Allocate buffer in TraceRays()");
      buffer_size_ = active_ray_num_ * kBufferSizeFactor;
      buffer_.Allocate(buffer_size_);
    }
    threading_pool_->CommitRangeStepJobsAndWait(0, active_ray_num_, [=](int /* thread_id */, int i) {
      Optics::HitSurface(crystal, n, 1,                                                        //
                         buffer_.dir[0] + i * 3, buffer_.face_id[0] + i, buffer_.w[0] + i,     //
                         buffer_.dir[1] + i * 6, buffer_.w[1] + i * 2);                        //
      Optics::Propagate(crystal, 1 * 2, buffer_.pt[0] + i * 3,                                 //
                        buffer_.dir[1] + i * 6, buffer_.w[1] + i * 2, buffer_.face_id[0] + i,  //
                        buffer_.pt[1] + i * 6, buffer_.face_id[1] + i * 2);                    //
    });
    StoreRaySegments(crystal_ctx, filter);
    RefreshBuffer();  // active_ray_num_ is updated.
  }
}


// Save rays
void Simulator::StoreRaySegments(const CrystalContext* crystal_ctx, AbstractRayPathFilter* filter) {
  const auto* crystal = crystal_ctx->GetCrystal();
  filter->ApplySymmetry(crystal_ctx);
  auto* ray_pool = RaySegmentPool::GetInstance();
  auto* r_array = ray_pool->AllocateObjectArray(active_ray_num_ * 2);
  for (size_t i = 0; i < active_ray_num_ * 2; i++) {
    if (buffer_.w[1][i] <= 0) {  // Refractive rays in total reflection case
      continue;
    }

    auto* r = new (r_array + i)
        RaySegment(buffer_.pt[0] + i / 2 * 3, buffer_.dir[1] + i * 3, buffer_.w[1][i], buffer_.face_id[0][i / 2]);
    if (buffer_.face_id[1][i] < 0) {
      r->state = RaySegmentState::kFinished;
    }
    if (r->w < ProjectContext::kPropMinW) {
      r->state = RaySegmentState::kCrystalAbsorbed;
    }

    auto* prev_ray_seg = buffer_.ray_seg[0][i / 2];
    if (i % 2 == 0) {
      prev_ray_seg->next_reflect = r;
    } else {
      prev_ray_seg->next_refract = r;
    }
    r->prev = prev_ray_seg;
    r->recorder = prev_ray_seg->recorder;
    r->recorder << crystal->FaceNumber(r->face_id);
    if (r->state == RaySegmentState::kFinished) {
      r->recorder << kInvalidId;
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
