#include "core/crystal.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
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

// Legal face-number sets mirror FillHexFnMap below:
//   basal:         1, 2
//   prism lateral: 3..8
//   upper pyramid: 13..18
//   lower pyramid: 23..28
// See crystal_kind.hpp for rationale on why this GUI-facing coarse enum is
// intentionally distinct from the core CrystalType.
bool IsLegalFace(CrystalKind kind, int face) {
  auto is_basal = [](int f) { return f == 1 || f == 2; };
  auto is_prism_lateral = [](int f) { return f >= 3 && f <= 8; };
  auto is_upper_pyramid = [](int f) { return f >= 13 && f <= 18; };
  auto is_lower_pyramid = [](int f) { return f >= 23 && f <= 28; };

  switch (kind) {
    case CrystalKind::kPrism:
      return is_basal(face) || is_prism_lateral(face);
    case CrystalKind::kPyramid:
      return is_basal(face) || is_prism_lateral(face) || is_upper_pyramid(face) || is_lower_pyramid(face);
  }
  // Unhandled CrystalKind — new enum values must extend the switch above.
  assert(false && "IsLegalFace: unhandled CrystalKind");
  return false;
}

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

    if (pri == 0 && pyr > 0) {
      fn_map[i] = pyr;  // basal: 1 or 2
    } else if (pri > 0) {
      fn_map[i] = pyr * 10 + pri;  // prism/pyramidal: 3-8, 13-18, 23-28
    } else {
      LOG_WARNING("Unrecognized face number!");
    }
  }
}

Crystal Crystal::CreatePrism(float h) {
  float dist[6]{ 1, 1, 1, 1, 1, 1 };
  float coef[kMaxHexCrystalPlanes * 4];
  auto plane_cnt = FillHexCrystalCoef(0, 0, 0, h, 0, dist, coef);
  auto c = Crystal(CreateConvexPolyhedronMesh(static_cast<int>(plane_cnt), coef));
  c.fn_period_ = 6;
  FillHexFnMap(c.TotalTriangles(), c.face_n_, c.fn_map_.get());
  c.BuildPolygonFaceData(coef, plane_cnt);
  return c;
}

Crystal Crystal::CreatePrism(float h, const float* fd) {
  float coef[kMaxHexCrystalPlanes * 4];
  auto plane_cnt = FillHexCrystalCoef(0, 0, 0, h, 0, fd, coef);
  auto c = Crystal(CreateConvexPolyhedronMesh(static_cast<int>(plane_cnt), coef));
  c.fn_period_ = 6;
  FillHexFnMap(c.TotalTriangles(), c.face_n_, c.fn_map_.get());
  c.BuildPolygonFaceData(coef, plane_cnt);
  return c;
}

Crystal Crystal::CreatePyramid(float h1, float h2, float h3) {
  float alpha = std::atan(math::kSqrt3_2 / kIceCrystalC) * math::kRadToDegree;
  return CreatePyramid(alpha, alpha, h1, h2, h3);
}

Crystal Crystal::CreatePyramid(float upper_alpha, float lower_alpha, float h1, float h2, float h3, const float* dist) {
  float coef[kMaxHexCrystalPlanes * 4];
  auto plane_cnt = FillHexCrystalCoef(upper_alpha, lower_alpha, h1, h2, h3, dist, coef);
  auto c = Crystal(CreateConvexPolyhedronMesh(static_cast<int>(plane_cnt), coef));
  c.fn_period_ = 6;
  FillHexFnMap(c.TotalTriangles(), c.face_n_, c.fn_map_.get());
  c.BuildPolygonFaceData(coef, plane_cnt);
  return c;
}

Crystal Crystal::CreatePyramid(float upper_alpha, float lower_alpha, float h1, float h2, float h3) {
  float dist[6]{ 1, 1, 1, 1, 1, 1 };
  return CreatePyramid(upper_alpha, lower_alpha, h1, h2, h3, dist);
}

Crystal Crystal::CreatePyramid(int upper_i1, int upper_i4, int lower_i1, int lower_i4,  // Miller index
                               float h1, float h2, float h3,                            // height
                               const float* dist) {                                     // face distance
  // Guard against i1=0: although float division yields inf (not UB), we prefer an explicit guard
  // over relying on atan(inf)=90° being filtered by the alpha range check in FillHexCrystalCoef.
  float upper_alpha =
      upper_i1 != 0 ? std::atan(math::kSqrt3_2 * upper_i4 / upper_i1 / kIceCrystalC) * math::kRadToDegree : 0.0f;
  float lower_alpha =
      lower_i1 != 0 ? std::atan(math::kSqrt3_2 * lower_i4 / lower_i1 / kIceCrystalC) * math::kRadToDegree : 0.0f;
  return CreatePyramid(upper_alpha, lower_alpha, h1, h2, h3, dist);
}


enum CrystalCachOffset {
  kVtx = 0,
  kNormal = 9,
  kArea = 12,
  kTf = 13,
  kTotal = 25,
};

Crystal::Crystal() {}

Crystal::Crystal(size_t vtx_cnt, std::unique_ptr<float[]> vtx, size_t triangle_cnt, std::unique_ptr<int[]> triangle_idx)
    : mesh_(vtx_cnt, std::move(vtx), triangle_cnt, std::move(triangle_idx)),
      cache_data_(std::make_unique<float[]>(triangle_cnt * CrystalCachOffset::kTotal)),
      face_v_(cache_data_.get() + triangle_cnt * CrystalCachOffset::kVtx),
      face_n_(cache_data_.get() + triangle_cnt * CrystalCachOffset::kNormal),
      face_area_(cache_data_.get() + triangle_cnt * CrystalCachOffset::kArea),
      face_coord_tf_(cache_data_.get() + triangle_cnt * CrystalCachOffset::kTf),
      fn_map_(std::make_unique<IdType[]>(triangle_cnt)), fn_period_(-1) {
  ComputeCacheData();
}

Crystal::Crystal(Mesh mesh)
    : mesh_(std::move(mesh)),
      cache_data_(std::make_unique<float[]>(mesh_.GetTriangleCnt() * CrystalCachOffset::kTotal)),
      face_v_(cache_data_.get() + mesh_.GetTriangleCnt() * CrystalCachOffset::kVtx),
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
      face_n_(cache_data_.get() + other.TotalTriangles() * CrystalCachOffset::kNormal),
      face_area_(cache_data_.get() + other.TotalTriangles() * CrystalCachOffset::kArea),
      face_coord_tf_(cache_data_.get() + other.TotalTriangles() * CrystalCachOffset::kTf),
      fn_map_(std::make_unique<IdType[]>(other.TotalTriangles())), fn_period_(other.fn_period_),
      poly_face_cnt_(other.poly_face_cnt_) {
  auto n = other.TotalTriangles();
  std::memcpy(cache_data_.get(), other.cache_data_.get(), n * CrystalCachOffset::kTotal * sizeof(float));
  std::memcpy(fn_map_.get(), other.fn_map_.get(), n * sizeof(IdType));

  if (poly_face_cnt_ > 0) {
    poly_face_data_ = std::make_unique<float[]>(poly_face_cnt_ * 5);
    poly_face_n_ = poly_face_data_.get();
    poly_face_d_ = poly_face_data_.get() + poly_face_cnt_ * 3;
    poly_face_tri_id_ = reinterpret_cast<int*>(poly_face_data_.get() + poly_face_cnt_ * 4);
    std::memcpy(poly_face_data_.get(), other.poly_face_data_.get(), poly_face_cnt_ * 5 * sizeof(float));
  }
}

Crystal::Crystal(Crystal&& other) noexcept
    : config_id_(other.config_id_), mesh_(std::move(other.mesh_)), cache_data_(std::move(other.cache_data_)),
      face_v_(cache_data_.get() + mesh_.GetTriangleCnt() * CrystalCachOffset::kVtx),
      face_n_(cache_data_.get() + mesh_.GetTriangleCnt() * CrystalCachOffset::kNormal),
      face_area_(cache_data_.get() + mesh_.GetTriangleCnt() * CrystalCachOffset::kArea),
      face_coord_tf_(cache_data_.get() + mesh_.GetTriangleCnt() * CrystalCachOffset::kTf),
      fn_map_(std::move(other.fn_map_)), fn_period_(other.fn_period_), poly_face_cnt_(other.poly_face_cnt_),
      poly_face_data_(std::move(other.poly_face_data_)) {
  other.face_v_ = nullptr;
  other.face_n_ = nullptr;
  other.face_area_ = nullptr;
  other.face_coord_tf_ = nullptr;

  if (poly_face_cnt_ > 0) {
    poly_face_n_ = poly_face_data_.get();
    poly_face_d_ = poly_face_data_.get() + poly_face_cnt_ * 3;
    poly_face_tri_id_ = reinterpret_cast<int*>(poly_face_data_.get() + poly_face_cnt_ * 4);
  }
  other.poly_face_cnt_ = 0;
  other.poly_face_n_ = nullptr;
  other.poly_face_d_ = nullptr;
  other.poly_face_tri_id_ = nullptr;
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
  face_n_ = cache_data_.get() + n * CrystalCachOffset::kNormal;
  face_area_ = cache_data_.get() + n * CrystalCachOffset::kArea;
  face_coord_tf_ = cache_data_.get() + n * CrystalCachOffset::kTf;
  fn_map_ = std::make_unique<IdType[]>(n);

  std::memcpy(cache_data_.get(), other.cache_data_.get(), n * CrystalCachOffset::kTotal * sizeof(float));
  std::memcpy(fn_map_.get(), other.fn_map_.get(), n * sizeof(IdType));

  fn_period_ = other.fn_period_;

  poly_face_cnt_ = other.poly_face_cnt_;
  if (poly_face_cnt_ > 0) {
    poly_face_data_ = std::make_unique<float[]>(poly_face_cnt_ * 5);
    poly_face_n_ = poly_face_data_.get();
    poly_face_d_ = poly_face_data_.get() + poly_face_cnt_ * 3;
    poly_face_tri_id_ = reinterpret_cast<int*>(poly_face_data_.get() + poly_face_cnt_ * 4);
    std::memcpy(poly_face_data_.get(), other.poly_face_data_.get(), poly_face_cnt_ * 5 * sizeof(float));
  } else {
    poly_face_data_.reset();
    poly_face_n_ = nullptr;
    poly_face_d_ = nullptr;
    poly_face_tri_id_ = nullptr;
  }

  return *this;
}

Crystal& Crystal::operator=(Crystal&& other) noexcept {
  if (&other == this) {
    return *this;
  }

  config_id_ = other.config_id_;
  mesh_ = std::move(other.mesh_);

  auto n = mesh_.GetTriangleCnt();
  cache_data_ = std::move(other.cache_data_);
  face_v_ = cache_data_.get() + n * CrystalCachOffset::kVtx;
  face_n_ = cache_data_.get() + n * CrystalCachOffset::kNormal;
  face_area_ = cache_data_.get() + n * CrystalCachOffset::kArea;
  face_coord_tf_ = cache_data_.get() + n * CrystalCachOffset::kTf;
  other.face_v_ = nullptr;
  other.face_n_ = nullptr;
  other.face_area_ = nullptr;
  other.face_coord_tf_ = nullptr;

  fn_map_ = std::move(other.fn_map_);
  fn_period_ = other.fn_period_;

  poly_face_cnt_ = other.poly_face_cnt_;
  poly_face_data_ = std::move(other.poly_face_data_);
  if (poly_face_cnt_ > 0) {
    poly_face_n_ = poly_face_data_.get();
    poly_face_d_ = poly_face_data_.get() + poly_face_cnt_ * 3;
    poly_face_tri_id_ = reinterpret_cast<int*>(poly_face_data_.get() + poly_face_cnt_ * 4);
  } else {
    poly_face_n_ = nullptr;
    poly_face_d_ = nullptr;
    poly_face_tri_id_ = nullptr;
  }
  other.poly_face_cnt_ = 0;
  other.poly_face_n_ = nullptr;
  other.poly_face_d_ = nullptr;
  other.poly_face_tri_id_ = nullptr;

  return *this;
}

void Crystal::ComputeCacheData() {
  auto triangle_cnt = mesh_.GetTriangleCnt();
  const auto* vtx = mesh_.GetVtxPtr(0);
  const auto* triangle_idx = mesh_.GetTrianglePtr(0);
  float ev[6];
  float m[3];
  for (size_t i = 0; i < triangle_cnt; i++) {
    // face_v_
    for (int j = 0; j < 3; j++) {
      std::memcpy(face_v_ + i * 9 + j * 3, vtx + triangle_idx[i * 3 + j] * 3, 3 * sizeof(float));
    }

    // edge vectors (local)
    for (int j = 0; j < 3; j++) {
      ev[j + 0] = face_v_[i * 9 + j + 3] - face_v_[i * 9 + j + 0];
      ev[j + 3] = face_v_[i * 9 + j + 6] - face_v_[i * 9 + j + 0];
    }

    // face_n_ && face_area_
    Cross3(ev + 0, ev + 3, face_n_ + i * 3);
    face_area_[i] = Norm3(face_n_ + i * 3) / 2.0f;
    Normalize3(face_n_ + i * 3);

    // face_coord_tf_
    Cross3(ev + 0, ev + 3, m);
    auto a = Dot3(face_n_ + i * 3, m);
    face_coord_tf_[i * 12 + 0] = (ev[4] * face_n_[i * 3 + 2] - ev[5] * face_n_[i * 3 + 1]) / a;
    face_coord_tf_[i * 12 + 1] = (ev[5] * face_n_[i * 3 + 0] - ev[3] * face_n_[i * 3 + 2]) / a;
    face_coord_tf_[i * 12 + 2] = (ev[3] * face_n_[i * 3 + 1] - ev[4] * face_n_[i * 3 + 0]) / a;

    face_coord_tf_[i * 12 + 4] = (ev[2] * face_n_[i * 3 + 1] - ev[1] * face_n_[i * 3 + 2]) / a;
    face_coord_tf_[i * 12 + 5] = (ev[0] * face_n_[i * 3 + 2] - ev[2] * face_n_[i * 3 + 0]) / a;
    face_coord_tf_[i * 12 + 6] = (ev[1] * face_n_[i * 3 + 0] - ev[0] * face_n_[i * 3 + 1]) / a;

    face_coord_tf_[i * 12 + 8] = (ev[1] * ev[5] - ev[2] * ev[4]) / a;
    face_coord_tf_[i * 12 + 9] = (ev[2] * ev[3] - ev[0] * ev[5]) / a;
    face_coord_tf_[i * 12 + 10] = (ev[0] * ev[4] - ev[1] * ev[3]) / a;

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
  // Rotate polygon face normals (d unchanged — rotation preserves origin distance)
  if (poly_face_cnt_ > 0) {
    r.Apply(poly_face_n_, poly_face_cnt_);
  }
  return *this;
}

std::vector<IdType> Crystal::ReduceRaypath(const std::vector<IdType>& rp, uint8_t symmetry) const {
  return ReduceRaypath(rp, symmetry, 0, false);
}

// NOLINTNEXTLINE(readability-function-cognitive-complexity)
std::vector<IdType> Crystal::ReduceRaypath(const std::vector<IdType>& rp, uint8_t symmetry, int sigma_a,
                                           bool d_applicable) const {
  if (symmetry == FilterConfig::kSymNone || fn_period_ < 0) {
    return rp;
  }

  std::vector<IdType> reduced_rp = rp;
  if (symmetry & FilterConfig::kSymP) {
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

  if ((symmetry & FilterConfig::kSymD) && d_applicable) {
    // σ-clean: reflect every prism face using formula face_id_new = (sigma_a - (face_id-3) + fn_period_) % fn_period_ +
    // 3
    std::vector<IdType> rp_reflected = reduced_rp;
    for (auto& x : rp_reflected) {
      if (x < 3 || (x >= 13 && x <= 28)) {
        continue;  // skip basal and pyramid faces
      }
      IdType pyr = x / 10;
      IdType pri = x % 10 - 3;
      pri = (sigma_a - pri + fn_period_) % fn_period_;
      x = pyr * 10 + pri + 3;
    }
    if (rp_reflected < reduced_rp) {
      reduced_rp = rp_reflected;
    }
  }

  if (symmetry & FilterConfig::kSymB) {
    // B reflection: basal 1↔2 and pyramid upper[13..18]↔lower[23..28], applied together.
    // Generate the B-reflected candidate and keep the lexicographically smaller one.
    std::vector<IdType> rp_b_reflected = reduced_rp;
    bool changed = false;
    for (auto& x : rp_b_reflected) {
      if (x <= 2) {
        x = 3 - x;
        changed = true;
      } else if (x >= 13 && x <= 18) {
        x += 10;
        changed = true;
      } else if (x >= 23 && x <= 28) {
        x -= 10;
        changed = true;
      }
    }
    if (changed && rp_b_reflected < reduced_rp) {
      reduced_rp = rp_b_reflected;
    }
  }

  return reduced_rp;
}

std::vector<std::vector<IdType>> Crystal::ExpandRaypath(const std::vector<IdType>& rp, uint8_t symmetry) const {
  return ExpandRaypath(rp, symmetry, 0, false);
}

// NOLINTNEXTLINE(readability-function-cognitive-complexity)
std::vector<std::vector<IdType>> Crystal::ExpandRaypath(const std::vector<IdType>& rp, uint8_t symmetry, int sigma_a,
                                                        bool d_applicable) const {
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

  if ((symmetry & FilterConfig::kSymD) && d_applicable) {
    // σ-clean: for each existing variant, generate σ-reflected copy
    auto size = result.size();
    for (size_t i = 0; i < size; i++) {
      auto curr_rp = result[i];
      bool changed = false;
      for (auto& x : curr_rp) {
        if (x < 3 || (x >= 13 && x <= 28)) {
          continue;  // skip basal and pyramid faces
        }
        IdType pyr = x / 10;
        IdType pri = x % 10 - 3;
        pri = (sigma_a - pri + fn_period_) % fn_period_;
        x = pyr * 10 + pri + 3;
        changed = true;
      }
      if (changed) {
        result.emplace_back(curr_rp);
      }
    }
  }

  if (symmetry & FilterConfig::kSymB) {
    // B reflection: basal 1↔2, pyramid upper[13..18]↔lower[23..28], prism unchanged.
    // Both swaps are part of the same transformation and applied together.
    auto size = result.size();
    for (size_t i = 0; i < size; i++) {
      auto curr_rp = result[i];
      bool changed = false;
      for (auto& x : curr_rp) {
        if (x <= 2) {
          x = 3 - x;
          changed = true;
        } else if (x >= 13 && x <= 18) {
          x += 10;
          changed = true;
        } else if (x >= 23 && x <= 28) {
          x -= 10;
          changed = true;
        }
      }
      if (changed) {
        result.emplace_back(curr_rp);
      }
    }
  }

  return result;
}

size_t Crystal::PolygonFaceCount() const {
  return poly_face_cnt_;
}

const float* Crystal::GetPolygonFaceNormal() const {
  return poly_face_n_;
}

const float* Crystal::GetPolygonFaceDist() const {
  return poly_face_d_;
}

const int* Crystal::GetPolygonFaceTriId() const {
  return poly_face_tri_id_;
}

// Safety note: this function matches plane equations to triangle normals via dot product (> 1.0 - 1e-3).
// Planes that are geometrically absent (e.g., faces eliminated by non-uniform face distances) will have no
// matching triangles and are naturally skipped. The safety of handling "phantom planes" depends on this
// normal-matching mechanism.
void Crystal::BuildPolygonFaceData(const float* plane_coef, size_t plane_cnt) {
  static_assert(sizeof(int) == sizeof(float), "BuildPolygonFaceData requires sizeof(int)==sizeof(float)");

  auto tri_cnt = TotalTriangles();

  // First pass: count valid faces (planes that match at least one triangle normal)
  size_t valid_cnt = 0;
  for (size_t p = 0; p < plane_cnt; p++) {
    const float* coef = plane_coef + p * 4;
    float norm_len = Norm3(coef);
    if (norm_len < math::kFloatEps) {
      continue;
    }

    float n[3]{ coef[0] / norm_len, coef[1] / norm_len, coef[2] / norm_len };
    for (size_t t = 0; t < tri_cnt; t++) {
      if (Dot3(n, face_n_ + t * 3) > 1.0f - 1e-3f) {
        valid_cnt++;
        break;
      }
    }
  }

  poly_face_cnt_ = valid_cnt;
  if (valid_cnt == 0) {
    poly_face_data_.reset();
    poly_face_n_ = nullptr;
    poly_face_d_ = nullptr;
    poly_face_tri_id_ = nullptr;
    return;
  }

  // Allocate: normals(3*cnt) + dist(cnt) + tri_id as int(cnt) = 5*cnt floats
  poly_face_data_ = std::make_unique<float[]>(valid_cnt * 5);
  poly_face_n_ = poly_face_data_.get();
  poly_face_d_ = poly_face_data_.get() + valid_cnt * 3;
  poly_face_tri_id_ = reinterpret_cast<int*>(poly_face_data_.get() + valid_cnt * 4);

  // Second pass: fill data
  size_t idx = 0;
  for (size_t p = 0; p < plane_cnt; p++) {
    const float* coef = plane_coef + p * 4;
    float norm_len = Norm3(coef);
    if (norm_len < math::kFloatEps) {
      continue;
    }

    float n[3]{ coef[0] / norm_len, coef[1] / norm_len, coef[2] / norm_len };
    float d = coef[3] / norm_len;

    int best_tri = -1;
    float best_dot = -1.0f;
    for (size_t t = 0; t < tri_cnt; t++) {
      float dot = Dot3(n, face_n_ + t * 3);
      if (dot > best_dot) {
        best_dot = dot;
        best_tri = static_cast<int>(t);
      }
    }

    if (best_tri < 0 || best_dot < 1.0f - 1e-3f) {
      continue;
    }

    poly_face_n_[idx * 3 + 0] = n[0];
    poly_face_n_[idx * 3 + 1] = n[1];
    poly_face_n_[idx * 3 + 2] = n[2];
    poly_face_d_[idx] = d;
    poly_face_tri_id_[idx] = best_tri;
    idx++;
  }
}

float Crystal::GetRefractiveIndex(float wl) const {
  return IceRefractiveIndex::Get(wl);
}


namespace detail {

bool IsRollMeanAtMultipleOf30(const AxisDistribution& d) {
  float mean = d.roll_dist.mean;
  float remainder = std::fmod(std::fmod(mean, 30.0f) + 30.0f, 30.0f);
  return FloatEqual(remainder, 0.0f) || FloatEqual(remainder, 30.0f);
}

int ComputeSigmaA(float roll_mean_deg) {
  int n = (static_cast<int>(std::round(roll_mean_deg / 30.0f)) % 6 + 6) % 6;
  return (6 - n) % 6;
}

bool IsDApplicable(const AxisDistribution& d) {
  return d.IsAzRotationallySymmetric() && IsRollMeanAtMultipleOf30(d);
}

}  // namespace detail


}  // namespace lumice
