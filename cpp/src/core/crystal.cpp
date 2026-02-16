#include "core/crystal.hpp"

#include <algorithm>
#include <cstddef>
#include <cstring>
#include <memory>
#include <utility>
#include <vector>

#include "config/filter_config.hpp"
#include "core/def.hpp"
#include "core/geo3d.hpp"
#include "core/math.hpp"
#include "core/optics.hpp"
#include "util/logger.hpp"

namespace lumice {

void FillHexFnMap(size_t face_cnt, const float* face_n, IdType* fn_map) {
  using math::kSqrt3_2;

  std::fill(fn_map, fn_map + face_cnt, kInvalidId);

  float ref_norms[9 * 3]{
    0.0f,  0.0f,      0.0f,   // placeholder
    0.0f,  0.0f,      1.0f,   // 1
    0.0f,  0.0f,      -1.0f,  // 2
    1.0f,  0.0f,      0.0f,   // 3
    0.5f,  kSqrt3_2,  0.0f,   // 4
    -0.5f, kSqrt3_2,  0.0f,   // 5
    -1.0f, 0.0f,      0.0f,   // 6
    -0.5,  -kSqrt3_2, 0.0f,   // 7
    0.5f,  -kSqrt3_2, 0.0f,   // 8
  };

  for (size_t i = 0; i < face_cnt; i++) {
    const float* curr_norm = face_n + i * 3;
    IdType pri = 0;
    float p_comp = -1;
    for (IdType j = 3; j < 9; j++) {
      if (auto p = Dot3(curr_norm, ref_norms + j * 3); p > math::kFloatEps && p > p_comp) {
        pri = j;
        p_comp = p;
      }
    }

    IdType pyr = 0;
    if (Dot3(curr_norm, ref_norms + 3) > math::kFloatEps) {
      pyr = 1;
    } else if (Dot3(curr_norm, ref_norms + 6) > math::kFloatEps) {
      pyr = 2;
    }

    if (pri > 0 || pyr > 0) {
      fn_map[i] = pyr * 10 + pri;
    } else {
      LOG_WARNING("Unrecognized face number!");
    }
  }
}

Crystal Crystal::CreatePrism(float h) {
  auto c = Crystal(CreatePrismMesh(h));
  c.fn_period_ = 6;
  for (int i = 0; i < 4; i++) {
    c.fn_map_[i] = 1;
    c.fn_map_[i + 16] = 2;
  }
  for (int i = 0; i < 6; i++) {
    c.fn_map_[i * 2 + 4] = i + 3;
    c.fn_map_[i * 2 + 5] = i + 3;
  }
  return c;
}

Crystal Crystal::CreatePrism(float h, const float* fd) {
  auto c = Crystal(CreatePrismMesh(h, fd));
  c.fn_period_ = 6;
  FillHexFnMap(c.TotalTriangles(), c.face_n_, c.fn_map_.get());
  return c;
}

Crystal Crystal::CreatePyramid(float h1, float h2, float h3) {
  auto c = Crystal(CreatePyramidMesh(h1, h2, h3));
  c.fn_period_ = 6;
  for (int i = 0; i < 4; i++) {
    c.fn_map_[i] = 1;
    c.fn_map_[i + 40] = 2;
  }
  for (int i = 0; i < 6; i++) {
    c.fn_map_[i * 2 + 4] = i + 13;
    c.fn_map_[i * 2 + 5] = i + 13;
    c.fn_map_[i * 2 + 16] = i + 3;
    c.fn_map_[i * 2 + 17] = i + 3;
    c.fn_map_[i * 2 + 28] = i + 23;
    c.fn_map_[i * 2 + 29] = i + 23;
  }
  return c;
}


Crystal Crystal::CreatePyramid(int upper_i1, int upper_i4, int lower_i1, int lower_i4,  // Miller index
                               float h1, float h2, float h3,                            // height
                               const float* dist) {                                     // face distance
  auto c = Crystal(CreatePyramidMesh(upper_i1, upper_i4, lower_i1, lower_i4, h1, h2, h3, dist));
  c.fn_period_ = 6;
  FillHexFnMap(c.TotalTriangles(), c.face_n_, c.fn_map_.get());
  return c;
}


enum CrystalCachOffset {
  kVtx = 0,
  kEdgeVec = 9,
  kNormal = 15,
  kArea = 18,
  kTf = 19,
  kTotal = 31,
};

Crystal::Crystal() {}

Crystal::Crystal(size_t vtx_cnt, std::unique_ptr<float[]> vtx, size_t triangle_cnt, std::unique_ptr<int[]> triangle_idx)
    : mesh_(vtx_cnt, std::move(vtx), triangle_cnt, std::move(triangle_idx)),
      cache_data_(std::make_unique<float[]>(triangle_cnt * CrystalCachOffset::kTotal)),
      face_v_(cache_data_.get() + triangle_cnt * CrystalCachOffset::kVtx),
      face_ev_(cache_data_.get() + triangle_cnt * CrystalCachOffset::kEdgeVec),
      face_n_(cache_data_.get() + triangle_cnt * CrystalCachOffset::kNormal),
      face_area_(cache_data_.get() + triangle_cnt * CrystalCachOffset::kArea),
      face_coord_tf_(cache_data_.get() + triangle_cnt * CrystalCachOffset::kTf),
      fn_map_(std::make_unique<IdType[]>(triangle_cnt)), fn_period_(-1) {
  // Initialize other pre-computed data
  ComputeCacheData();
}

Crystal::Crystal(Mesh mesh)
    : mesh_(std::move(mesh)),
      cache_data_(std::make_unique<float[]>(mesh_.GetTriangleCnt() * CrystalCachOffset::kTotal)),
      face_v_(cache_data_.get() + mesh_.GetTriangleCnt() * CrystalCachOffset::kVtx),
      face_ev_(cache_data_.get() + mesh_.GetTriangleCnt() * CrystalCachOffset::kEdgeVec),
      face_n_(cache_data_.get() + mesh_.GetTriangleCnt() * CrystalCachOffset::kNormal),
      face_area_(cache_data_.get() + mesh_.GetTriangleCnt() * CrystalCachOffset::kArea),
      face_coord_tf_(cache_data_.get() + mesh_.GetTriangleCnt() * CrystalCachOffset::kTf),
      fn_map_(std::make_unique<IdType[]>(mesh_.GetTriangleCnt())), fn_period_(-1) {
  ComputeCacheData();
}

Crystal::Crystal(const Crystal& other)
    : config_id_(other.config_id_), mesh_(other.mesh_),
      cache_data_(std::make_unique<float[]>(other.TotalTriangles() * CrystalCachOffset::kTotal)),
      face_v_(cache_data_.get() + other.TotalTriangles() * CrystalCachOffset::kVtx),
      face_ev_(cache_data_.get() + other.TotalTriangles() * CrystalCachOffset::kEdgeVec),
      face_n_(cache_data_.get() + other.TotalTriangles() * CrystalCachOffset::kNormal),
      face_area_(cache_data_.get() + other.TotalTriangles() * CrystalCachOffset::kArea),
      face_coord_tf_(cache_data_.get() + other.TotalTriangles() * CrystalCachOffset::kTf),
      fn_map_(std::make_unique<IdType[]>(other.TotalTriangles())), fn_period_(other.fn_period_) {
  auto n = other.TotalTriangles();
  std::memcpy(cache_data_.get(), other.cache_data_.get(), n * CrystalCachOffset::kTotal * sizeof(float));
  std::memcpy(fn_map_.get(), other.fn_map_.get(), n * sizeof(IdType));
}

Crystal::Crystal(Crystal&& other)
    : config_id_(other.config_id_), mesh_(std::move(other.mesh_)), cache_data_(std::move(other.cache_data_)),
      face_v_(cache_data_.get() + mesh_.GetTriangleCnt() * CrystalCachOffset::kVtx),
      face_ev_(cache_data_.get() + mesh_.GetTriangleCnt() * CrystalCachOffset::kEdgeVec),
      face_n_(cache_data_.get() + mesh_.GetTriangleCnt() * CrystalCachOffset::kNormal),
      face_area_(cache_data_.get() + mesh_.GetTriangleCnt() * CrystalCachOffset::kArea),
      face_coord_tf_(cache_data_.get() + mesh_.GetTriangleCnt() * CrystalCachOffset::kTf),
      fn_map_(std::move(other.fn_map_)), fn_period_(other.fn_period_) {
  other.face_v_ = nullptr;
  other.face_ev_ = nullptr;
  other.face_n_ = nullptr;
  other.face_area_ = nullptr;
  other.face_coord_tf_ = nullptr;
}

Crystal& Crystal::operator=(const Crystal& other) {
  if (&other == this) {
    return *this;
  }

  config_id_ = other.config_id_;
  mesh_ = other.mesh_;

  auto n = other.TotalTriangles();
  cache_data_ = std::make_unique<float[]>(n * CrystalCachOffset::kTotal);
  face_v_ = cache_data_.get() + n * CrystalCachOffset::kVtx;
  face_ev_ = cache_data_.get() + n * CrystalCachOffset::kEdgeVec;
  face_n_ = cache_data_.get() + n * CrystalCachOffset::kNormal;
  face_area_ = cache_data_.get() + n * CrystalCachOffset::kArea;
  face_coord_tf_ = cache_data_.get() + n * CrystalCachOffset::kTf;
  fn_map_ = std::make_unique<IdType[]>(n);

  std::memcpy(cache_data_.get(), other.cache_data_.get(), n * CrystalCachOffset::kTotal * sizeof(float));
  std::memcpy(fn_map_.get(), other.fn_map_.get(), n * sizeof(IdType));

  fn_period_ = other.fn_period_;
  return *this;
}

Crystal& Crystal::operator=(Crystal&& other) {
  if (&other == this) {
    return *this;
  }

  config_id_ = other.config_id_;
  mesh_ = std::move(other.mesh_);

  auto n = mesh_.GetTriangleCnt();
  cache_data_ = std::move(other.cache_data_);
  face_v_ = cache_data_.get() + n * CrystalCachOffset::kVtx;
  face_ev_ = cache_data_.get() + n * CrystalCachOffset::kEdgeVec;
  face_n_ = cache_data_.get() + n * CrystalCachOffset::kNormal;
  face_area_ = cache_data_.get() + n * CrystalCachOffset::kArea;
  face_coord_tf_ = cache_data_.get() + n * CrystalCachOffset::kTf;
  other.face_v_ = nullptr;
  other.face_ev_ = nullptr;
  other.face_n_ = nullptr;
  other.face_area_ = nullptr;
  other.face_coord_tf_ = nullptr;

  fn_map_ = std::move(other.fn_map_);
  fn_period_ = other.fn_period_;
  return *this;
}

void Crystal::ComputeCacheData() {
  auto triangle_cnt = mesh_.GetTriangleCnt();
  const auto* vtx = mesh_.GetVtxPtr(0);
  const auto* triangle_idx = mesh_.GetTrianglePtr(0);
  float m[3];
  for (size_t i = 0; i < triangle_cnt; i++) {
    // face_v_
    for (int j = 0; j < 3; j++) {
      std::memcpy(face_v_ + i * 9 + j * 3, vtx + triangle_idx[i * 3 + j] * 3, 3 * sizeof(float));
    }

    // face_ev_
    for (int j = 0; j < 3; j++) {
      face_ev_[i * 6 + j + 0] = face_v_[i * 9 + j + 3] - face_v_[i * 9 + j + 0];
      face_ev_[i * 6 + j + 3] = face_v_[i * 9 + j + 6] - face_v_[i * 9 + j + 0];
    }

    // face_n_ && face_area_
    Cross3(face_ev_ + i * 6 + 0, face_ev_ + i * 6 + 3, face_n_ + i * 3);
    face_area_[i] = Norm3(face_n_ + i * 3) / 2.0f;
    Normalize3(face_n_ + i * 3);

    // face_coord_tf_
    Cross3(face_ev_ + i * 6 + 0, face_ev_ + i * 6 + 3, m);
    auto a = Dot3(face_n_ + i * 3, m);
    face_coord_tf_[i * 12 + 0] =
        (face_ev_[i * 6 + 4] * face_n_[i * 3 + 2] - face_ev_[i * 6 + 5] * face_n_[i * 3 + 1]) / a;
    face_coord_tf_[i * 12 + 1] =
        (face_ev_[i * 6 + 5] * face_n_[i * 3 + 0] - face_ev_[i * 6 + 3] * face_n_[i * 3 + 2]) / a;
    face_coord_tf_[i * 12 + 2] =
        (face_ev_[i * 6 + 3] * face_n_[i * 3 + 1] - face_ev_[i * 6 + 4] * face_n_[i * 3 + 0]) / a;

    face_coord_tf_[i * 12 + 4] =
        (face_ev_[i * 6 + 2] * face_n_[i * 3 + 1] - face_ev_[i * 6 + 1] * face_n_[i * 3 + 2]) / a;
    face_coord_tf_[i * 12 + 5] =
        (face_ev_[i * 6 + 0] * face_n_[i * 3 + 2] - face_ev_[i * 6 + 2] * face_n_[i * 3 + 0]) / a;
    face_coord_tf_[i * 12 + 6] =
        (face_ev_[i * 6 + 1] * face_n_[i * 3 + 0] - face_ev_[i * 6 + 0] * face_n_[i * 3 + 1]) / a;

    face_coord_tf_[i * 12 + 8] =
        (face_ev_[i * 6 + 1] * face_ev_[i * 6 + 5] - face_ev_[i * 6 + 2] * face_ev_[i * 6 + 4]) / a;
    face_coord_tf_[i * 12 + 9] =
        (face_ev_[i * 6 + 2] * face_ev_[i * 6 + 3] - face_ev_[i * 6 + 0] * face_ev_[i * 6 + 5]) / a;
    face_coord_tf_[i * 12 + 10] =
        (face_ev_[i * 6 + 0] * face_ev_[i * 6 + 4] - face_ev_[i * 6 + 1] * face_ev_[i * 6 + 3]) / a;

    face_coord_tf_[i * 12 + 3] = -Dot3(face_coord_tf_ + i * 12 + 0, face_v_ + i * 9);
    face_coord_tf_[i * 12 + 7] = -Dot3(face_coord_tf_ + i * 12 + 4, face_v_ + i * 9);
    face_coord_tf_[i * 12 + 11] = -Dot3(face_coord_tf_ + i * 12 + 8, face_v_ + i * 9);
  }
}

size_t Crystal::TotalTriangles() const {
  return mesh_.GetTriangleCnt();
}

size_t Crystal::TotalVertices() const {
  return mesh_.GetVtxCnt();
}

const float* Crystal::GetTriangleVtx() const {
  return face_v_;
}

const float* Crystal::GetTriangleEdgeVec() const {
  return face_ev_;
}

const float* Crystal::GetTriangleNormal() const {
  return face_n_;
}

const float* Crystal::GetTirangleArea() const {
  return face_area_;
}

const float* Crystal::GetTriangleCoordTf() const {
  return face_coord_tf_;
}

IdType Crystal::GetFn(int fid) const {
  if (fid < 0 || fid >= static_cast<int>(mesh_.GetTriangleCnt())) {
    return kInvalidId;
  } else {
    return fn_map_[fid];
  }
}

Crystal& Crystal::Rotate(const Rotation& r) {
  auto vtx_cnt = mesh_.GetVtxCnt();
  auto* vtx = mesh_.GetVtxPtr(0);
  r.Apply(vtx, vtx_cnt);
  ComputeCacheData();
  return *this;
}

std::vector<IdType> Crystal::ReduceRaypath(const std::vector<IdType>& rp, uint8_t symmetry) const {
  if (symmetry == FilterConfig::kSymNone || fn_period_ < 0) {
    return rp;
  }

  std::vector<IdType> reduced_rp = rp;
  if (symmetry | FilterConfig::kSymP) {
    IdType first_pri = kInvalidId;
    for (auto& x : reduced_rp) {
      if (x < 3) {
        continue;
      }
      IdType pyr = x / 10;
      IdType pri = x % 10;
      if (first_pri == kInvalidId) {
        first_pri = pri;
      }
      pri += fn_period_ - first_pri;
      pri %= fn_period_;
      pri += 3;
      x = pyr * 10 + pri;
    }
  }

  if (symmetry | FilterConfig::kSymD) {
    IdType pri1 = kInvalidId;
    IdType pri2 = kInvalidId;
    for (auto& x : reduced_rp) {
      if (x < 3) {
        continue;
      }
      IdType pyr = x / 10;
      IdType pri = x % 10 - 3;
      if (pri1 == kInvalidId) {
        pri1 = pri;
        continue;
      }

      if (pri2 == kInvalidId) {
        if ((2 * pri1 - pri + fn_period_) % fn_period_ == pri) {
          continue;
        } else if ((2 * pri1 - pri + fn_period_) % fn_period_ > pri) {
          break;  // Do nothing. It is already reduced.
        } else {
          pri2 = pri;
        }
      }

      pri = (2 * pri1 - pri + fn_period_) % fn_period_;
      pri += 3;
      x = pyr * 10 + pri;
    }
  }

  if (symmetry | FilterConfig::kSymB) {
    IdType b1 = kInvalidId;
    for (auto& x : reduced_rp) {
      if (x > 2) {
        continue;
      }
      if (b1 == kInvalidId) {
        b1 = x;
      }
      if (b1 == 1) {
        break;  // Do nothing. It is alread reduced.
      }

      x = 3 - x;
    }
  }

  return reduced_rp;
}

std::vector<std::vector<IdType>> Crystal::ExpandRaypath(const std::vector<IdType>& rp, uint8_t symmetry) const {
  std::vector<std::vector<IdType>> result;
  result.emplace_back(rp);
  if (symmetry == FilterConfig::kSymNone || fn_period_ < 0) {
    return result;
  }

  if (symmetry & FilterConfig::kSymP) {
    for (int i = 1; i < fn_period_; i++) {
      std::vector<IdType> curr_rp{ rp };
      bool changed = false;
      for (auto& x : curr_rp) {
        if (x < 3) {
          continue;
        }

        IdType pyr = x / 10;
        IdType pri = x % 10;
        pri += fn_period_ - 3;
        pri += i;
        pri %= fn_period_;
        pri += 3;
        x = pyr * 10 + pri;
        changed = true;
      }
      if (changed) {
        result.emplace_back(curr_rp);
      }
    }
  }

  if (symmetry & FilterConfig::kSymD) {
    auto size = result.size();
    for (size_t i = 0; i < size; i++) {
      auto curr_rp = result[i];
      IdType pri0 = kInvalidId;
      bool changed = false;
      for (auto& x : curr_rp) {
        if (x < 3) {
          continue;
        }

        IdType pyr = x / 10;
        IdType pri = x % 10;
        if (pri0 == kInvalidId) {
          pri0 = pri;
          continue;
        }

        pri -= 3;
        pri = 2 * pri0 - pri + fn_period_;
        pri %= fn_period_;
        pri += 3;
        x = pyr * 10 + pri;
        changed = true;
      }
      if (changed) {
        result.emplace_back(curr_rp);
      }
    }
  }

  if (symmetry & FilterConfig::kSymB) {
    auto size = result.size();
    for (size_t i = 0; i < size; i++) {
      auto curr_rp = result[i];
      bool changed = false;
      for (auto& x : curr_rp) {
        if (x > 2) {
          continue;
        }

        x = 3 - x;
        changed = true;
      }
      if (changed) {
        result.emplace_back(curr_rp);
      }
    }
  }

  return result;
}

float Crystal::GetRefractiveIndex(float wl) const {
  return IceRefractiveIndex::Get(wl);
}

}  // namespace lumice
