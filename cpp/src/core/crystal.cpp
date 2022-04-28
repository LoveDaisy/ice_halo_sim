#include "core/crystal.hpp"

#include <algorithm>
#include <cstddef>
#include <cstring>
#include <memory>
#include <queue>
#include <set>
#include <tuple>
#include <utility>
#include <vector>

#include "context/context.hpp"
#include "core/def.hpp"
#include "core/math.hpp"
#include "core/optics.hpp"
#include "protocol/filter_config.hpp"
#include "util/log.hpp"
#include "util/obj_pool.hpp"

namespace icehalo {

using CrystalPrimitiveNormTalbe = std::vector<std::pair<Vec3f, int>>;

const CrystalPrimitiveNormTalbe& GetHexFaceNormToNumberList() {
  static CrystalPrimitiveNormTalbe face_norm_to_number_list{
    { { 0, 0, 1 }, 1 },                      // top face
    { { 0, 0, -1 }, 2 },                     // bottom face
    { { 1, 0, 0 }, 3 },                      // prism face
    { { 0.5f, math::kSqrt3 / 2, 0 }, 4 },    // prism face
    { { -0.5f, math::kSqrt3 / 2, 0 }, 5 },   // prism face
    { { -1, 0, 0 }, 6 },                     // prism face
    { { -0.5f, -math::kSqrt3 / 2, 0 }, 7 },  // prism face
    { { 0.5f, -math::kSqrt3 / 2, 0 }, 8 },   // prism face
  };
  return face_norm_to_number_list;
}


const CrystalPrimitiveNormTalbe& GetCubicFaceNormToNumberList() {
  static CrystalPrimitiveNormTalbe face_norm_to_number_list{
    { { 0, 0, 1 }, 1 },   // top face
    { { 0, 0, -1 }, 2 },  // bottom face
    { { 1, 0, 0 }, 3 },   // pyramidal face
    { { 0, 1, 0 }, 4 },   // pyramidal face
    { { -1, 0, 0 }, 5 },  // pyramidal face
    { { 0, -1, 0 }, 6 },  // pyramidal face
  };
  return face_norm_to_number_list;
}


Crystal::Crystal(std::vector<Vec3f> vertexes,     // vertex
                 std::vector<TriangleIdx> faces,  // face indices
                 CrystalType type)                // crystal type
    : type_(type), vertexes_(std::move(vertexes)), faces_(std::move(faces)), face_number_period_(-1),
      face_bases_(nullptr), face_vertexes_(nullptr), face_norm_(nullptr), face_area_(nullptr) {
  InitBasicData();
  InitPrimaryFaceNumber();
  PruneRedundantFaces();
  RefineFaceNumber();
  MergeFaces();
}


Crystal::Crystal(std::vector<Vec3f> vertexes,                 // vertex
                 std::vector<TriangleIdx> faces,              // face indices
                 std::vector<ShortIdType> face_number_table,  // face to face number
                 CrystalType type)                            // crystal type
    : type_(type), vertexes_(std::move(vertexes)), faces_(std::move(faces)),
      face_number_table_(std::move(face_number_table)), face_number_period_(-1), face_bases_(nullptr),
      face_vertexes_(nullptr), face_norm_(nullptr), face_area_(nullptr) {
  InitBasicData();
  MergeFaces();
}


const std::vector<Vec3f>& Crystal::GetVertexes() const {
  return vertexes_;
}


const std::vector<TriangleIdx>& Crystal::GetFaces() const {
  return faces_;
}


const std::vector<ShortIdType>& Crystal::GetFaceNumberTable() const {
  return face_number_table_;
}


const std::map<ShortIdType, PolygonIdx>& Crystal::GetMergedFaces() const {
  return merged_faces_;
}


const float* Crystal::GetFaceVertex() const {
  return face_vertexes_.get();
}


const float* Crystal::GetFaceBaseVector() const {
  return face_bases_.get();
}


const float* Crystal::GetFaceNorm() const {
  return face_norm_.get();
}


const float* Crystal::GetFaceArea() const {
  return face_area_.get();
}


int Crystal::GetFaceNumberPeriod() const {
  return face_number_period_;
}


CrystalType Crystal::GetType() const {
  return type_;
}


int Crystal::TotalVertexes() const {
  return static_cast<int>(vertexes_.size());
}

int Crystal::TotalFaces() const {
  return static_cast<int>(faces_.size());
}

ShortIdType Crystal::FaceNumber(int idx) const {
  if (face_number_table_.empty() || idx < 0 || static_cast<size_t>(idx) >= face_number_table_.size()) {
    return kInvalidId;
  } else {
    return face_number_table_[idx];
  }
}


void Crystal::InitBasicData() {
  auto face_num = faces_.size();
  face_bases_.reset(new float[face_num * 6]);
  face_vertexes_.reset(new float[face_num * 9]);
  face_norm_.reset(new float[face_num * 3]);
  face_area_.reset(new float[face_num]);

  auto* face_bases_ptr = face_bases_.get();
  auto* face_vertexes_ptr = face_vertexes_.get();
  auto* face_norm_ptr = face_norm_.get();

  for (size_t i = 0; i < faces_.size(); i++) {
    const auto& f = faces_[i];
    const auto* idx = f.idx();
    Vec3FromTo(vertexes_[idx[0]].val(), vertexes_[idx[1]].val(), face_bases_ptr + i * 6 + 0);
    Vec3FromTo(vertexes_[idx[0]].val(), vertexes_[idx[2]].val(), face_bases_ptr + i * 6 + 3);
    Cross3(face_bases_ptr + i * 6 + 0, face_bases_ptr + i * 6 + 3, face_norm_ptr + i * 3);

    face_area_[i] = Norm3(face_norm_ptr + i * 3) / 2;
    Normalize3(face_norm_ptr + i * 3);

    std::memcpy(face_vertexes_ptr + i * 9 + 0, vertexes_[idx[0]].val(), 3 * sizeof(float));
    std::memcpy(face_vertexes_ptr + i * 9 + 3, vertexes_[idx[1]].val(), 3 * sizeof(float));
    std::memcpy(face_vertexes_ptr + i * 9 + 6, vertexes_[idx[2]].val(), 3 * sizeof(float));
  }

  switch (type_) {
    case CrystalType::kPrism:
    case CrystalType::kPyramid_H3:
    case CrystalType::kPyramid_I2H3:
    case CrystalType::kPyramid_I4H3:
    case CrystalType::kPyramid_A2H3:
    case CrystalType::kPyramidStackHalf:
    case CrystalType::kIrregularPyramid:
    case CrystalType::kIrregularPrism:
      face_number_period_ = 6;
      break;
    case CrystalType::kCubicPyramid:
      face_number_period_ = 4;
      break;
    case CrystalType::kCustom:
    case CrystalType::kUnknown:
      break;
  }
}

void Crystal::InitPrimaryFaceNumber() {
  size_t face_num = faces_.size();

  face_number_table_.clear();
  face_number_table_.resize(face_num);
  std::fill(face_number_table_.begin(), face_number_table_.end(), 0);

  ShortIdType curr_id = std::numeric_limits<ShortIdType>::max() - 1;
  for (size_t i = 0; i < face_num; i++) {
    if (face_number_table_[i] != 0) {
      continue;
    }
    // Propagate this face number to other faces
    std::queue<size_t> tmp_queue;
    tmp_queue.emplace(i);
    while (!tmp_queue.empty()) {
      auto curr_i = tmp_queue.front();
      tmp_queue.pop();
      face_number_table_[curr_i] = curr_id;
      for (size_t j = 0; j < face_num; j++) {
        if (face_number_table_[j] == 0 && IsAdjacent(curr_i, j) && IsCoplanar(curr_i, j)) {
          tmp_queue.emplace(j);
        }
      }
    }
    curr_id--;
  }
}


void Crystal::PruneRedundantFaces() {
  size_t faces_num = faces_.size();
  std::vector<bool> face_remove(faces_num, false);

  // 1. remove area = 0
  for (size_t i = 0; i < faces_num; i++) {
    if (face_area_[i] < math::kFloatEps) {
      face_remove[i] = true;
    }
  }

  // 2. remove those thickness = 0 parts
  for (size_t i = 0; i < faces_num; i++) {
    if (face_remove[i]) {
      continue;
    }
    for (size_t j = i + 1; j < faces_num; j++) {
      if (face_remove[j] || face_number_table_[i] == face_number_table_[j] || !IsCounterCoplanar(i, j)) {
        continue;
      }
      // 1. Find all vertexes of the two surfaces
      std::set<int> surface1_v;
      std::set<int> surface2_v;
      for (size_t k = 0; k < faces_num; k++) {
        if (face_number_table_[i] == face_number_table_[k]) {
          surface1_v.emplace(faces_[i].idx()[0]);
          surface1_v.emplace(faces_[i].idx()[1]);
          surface1_v.emplace(faces_[i].idx()[2]);
        } else if (face_number_table_[j] == face_number_table_[k]) {
          surface2_v.emplace(faces_[j].idx()[0]);
          surface2_v.emplace(faces_[j].idx()[1]);
          surface2_v.emplace(faces_[j].idx()[2]);
        }
      }
      // 2. Check if every vertex are matched
      bool all_matched = true;
      for (auto v1 : surface1_v) {
        float min_d = std::numeric_limits<float>::max();
        for (auto v2 : surface2_v) {
          auto tmp_d = DiffNorm3(vertexes_[v1].val(), vertexes_[v2].val());
          if (tmp_d < min_d) {
            min_d = tmp_d;
          }
        }
        if (min_d > math::kFloatEps) {
          all_matched = false;
          break;
        }
      }
      if (all_matched) {
        // 3. Remove faces
        for (auto v1 : surface1_v) {
          face_remove[v1] = true;
        }
        for (auto v2 : surface2_v) {
          face_remove[v2] = true;
        }
      }
    }
  }

  // 3. Collect remained faces
  size_t new_faces_num = 0;
  for (auto r : face_remove) {
    new_faces_num += !r;
  }
  if (new_faces_num == faces_num) {
    return;
  }

  decltype(faces_) new_faces;
  decltype(face_number_table_) new_face_number_table;
  std::unique_ptr<float[]> new_face_bases{ new float[new_faces_num * 6]{} };
  std::unique_ptr<float[]> new_face_vertexes{ new float[new_faces_num * 9]{} };
  std::unique_ptr<float[]> new_face_norm{ new float[new_faces_num * 3]{} };
  std::unique_ptr<float[]> new_face_area{ new float[new_faces_num * 1]{} };

  size_t new_idx = 0;
  for (size_t i = 0; i < faces_num; i++) {
    if (face_remove[i]) {
      continue;
    }
    new_faces.emplace_back(faces_[i]);
    new_face_number_table.emplace_back(face_number_table_[i]);
    std::memcpy(new_face_bases.get() + new_idx * 6, face_bases_.get() + i * 6, 6 * sizeof(float));
    std::memcpy(new_face_vertexes.get() + new_idx * 9, face_vertexes_.get() + i * 9, 9 * sizeof(float));
    std::memcpy(new_face_norm.get() + new_idx * 3, face_norm_.get() + i * 3, 3 * sizeof(float));
    new_face_area[new_idx] = face_area_[i];
    new_idx++;
  }

  faces_.swap(new_faces);
  face_number_table_.swap(new_face_number_table);
  face_bases_ = std::move(new_face_bases);
  face_vertexes_ = std::move(new_face_vertexes);
  face_norm_ = std::move(new_face_norm);
  face_area_ = std::move(new_face_area);
}

void Crystal::RefineFaceNumber() {
  size_t face_num = faces_.size();

  // Find all pyramidal faces z component
  using PyrIdxInfo = std::pair<float, int>;  // {z-comp, pyr idx}
  std::vector<PyrIdxInfo> norm_z_pyr;
  for (size_t i = 0; i < face_num; i++) {
    auto curr_z = face_norm_[i * 3 + 2];
    if (std::abs(curr_z) > 1 - math::kFloatEps || std::abs(curr_z) < math::kFloatEps) {
      continue;
    }
    auto iter = std::find_if(norm_z_pyr.begin(), norm_z_pyr.end(),
                             [=](const PyrIdxInfo& a) { return std::abs(a.first - curr_z) < math::kFloatEps; });
    if (iter == norm_z_pyr.end()) {
      norm_z_pyr.emplace_back(curr_z, 0);
    }
  }

  if (norm_z_pyr.size() > 1) {
    // Sort them
    std::sort(norm_z_pyr.begin(), norm_z_pyr.end(),
              [=](const PyrIdxInfo& a, const PyrIdxInfo& b) { return a.first < b.first; });
    // Assign pyramidal index
    for (int lower_iter = 0, upper_iter = static_cast<int>(norm_z_pyr.size()) - 1, lower_idx = 2, upper_idx = 1;
         lower_iter < upper_iter; lower_iter++, upper_iter--) {
      norm_z_pyr[lower_iter].second = lower_idx++;
      norm_z_pyr[upper_iter].second = upper_idx++;
    }
  } else if (norm_z_pyr.size() == 1) {
    float min_z = std::numeric_limits<float>::max();
    for (size_t i = 0; i < face_num; i++) {
      for (int j = 0; j < 3; j++) {
        if (face_vertexes_[i * 9 + j * 3 + 2] < min_z) {
          min_z = face_vertexes_[i * 9 + j * 3 + 2];
        }
      }
    }
    norm_z_pyr[0].second = norm_z_pyr[0].first > min_z + math::kFloatEps ? 1 : 2;
  }

  const CrystalPrimitiveNormTalbe& primitive_fn_table =
      face_number_period_ == 6 ? GetHexFaceNormToNumberList() : GetCubicFaceNormToNumberList();
  ShortIdType fn_th = std::numeric_limits<ShortIdType>::max() / 2;
  // Assign primitive face number
  for (size_t i = 0; i < face_num; i++) {
    if (face_number_table_[i] < fn_th) {
      continue;
    }
    const auto* curr_norm = face_norm_.get() + i * 3;
    for (const auto& [p_norm, fn] : primitive_fn_table) {
      if (std::abs(Dot3(p_norm.val(), curr_norm) - 1) < math::kFloatEps) {
        face_number_table_[i] = fn;
        break;
      }
    }
  }

  // Assign those pyramidal faces
  for (size_t i = 0; i < face_num; i++) {
    if (face_number_table_[i] < fn_th) {
      continue;
    }
    auto curr_fn = face_number_table_[i];
    const auto* curr_norm = face_norm_.get() + i * 3;
    auto cmp = [=](const std::pair<Vec3f, int>& a, const std::pair<Vec3f, int>& b) {
      float hor_n[3]{ curr_norm[0], curr_norm[1], 0.0f };
      return Dot3(hor_n, a.first.val()) < Dot3(hor_n, b.first.val());
    };
    auto [min_iter, max_iter] = std::minmax_element(primitive_fn_table.begin() + 2, primitive_fn_table.end(), cmp);
    float face_center[3]{};
    for (int j = 0; j < 3; j++) {
      face_center[0] += face_vertexes_[i * 9 + j * 3 + 0] / 3.0f;
      face_center[1] += face_vertexes_[i * 9 + j * 3 + 1] / 3.0f;
    }
    auto max_dot3 = Dot3(face_center, max_iter->first.val());
    auto min_dot3 = Dot3(face_center, min_iter->first.val());
    int fn = max_dot3 > min_dot3 ? max_iter->second : min_iter->second;

    auto z_iter = std::find_if(norm_z_pyr.begin(), norm_z_pyr.end(),
                               [=](const PyrIdxInfo& x) { return std::abs(x.first - curr_norm[2]) < math::kFloatEps; });
    int pyr = z_iter == norm_z_pyr.end() ? 0 : z_iter->second;

    for (size_t j = 0; j < face_num; j++) {
      if (face_number_table_[j] == curr_fn) {
        face_number_table_[j] = pyr * 10 + fn;
      }
    }
  }
}

void Crystal::MergeFaces() {
  size_t face_num = faces_.size();
  size_t vertex_num = vertexes_.size();
  std::unique_ptr<int[]> tmp_mat{ new int[vertex_num * vertex_num] };
  for (size_t fid = 0; fid < face_num; fid++) {
    auto curr_face_num = face_number_table_[fid];
    if (merged_faces_.count(curr_face_num)) {
      continue;
    }

    std::memset(tmp_mat.get(), 0, sizeof(int) * vertex_num * vertex_num);
    for (size_t tmp_fid = fid; tmp_fid < face_num; tmp_fid++) {
      if (face_number_table_[tmp_fid] != curr_face_num) {
        continue;
      }
      const auto* tmp_tri_idx = faces_[tmp_fid].idx();
      for (int i = 0; i < 3; i++) {
        ShortIdType id1 = tmp_tri_idx[i % 3];
        ShortIdType id2 = tmp_tri_idx[(i + 1) % 3];
        tmp_mat[id1 * vertex_num + id2] = 1;
      }
    }
    for (size_t r = 0; r < vertex_num; r++) {
      for (size_t c = r + 1; c < vertex_num; c++) {
        if (tmp_mat[r * vertex_num + c] > 0 && tmp_mat[c * vertex_num + r] > 0) {
          tmp_mat[r * vertex_num + c] = 0;
          tmp_mat[c * vertex_num + r] = 0;
        } else if (tmp_mat[r * vertex_num + c] > 0) {
          tmp_mat[c * vertex_num + r] -= 1;
        } else if (tmp_mat[c * vertex_num + r] > 0) {
          tmp_mat[r * vertex_num + c] -= 1;
        }
      }
    }

    std::vector<ShortIdType> curr_face;
    {
      bool found = false;
      size_t r = 0;
      size_t c = 0;
      for (; r < vertex_num; r++) {
        for (c = 0; c < vertex_num; c++) {
          if (tmp_mat[r * vertex_num + c] > 0) {
            found = true;
            break;
          }
        }
        if (found) {
          break;
        }
      }

      bool finished = !found;
      curr_face.emplace_back(r + 1);
      while (!finished) {
        curr_face.emplace_back(c + 1);
        tmp_mat[r * vertex_num + c] = 0;
        r = c;
        finished = true;
        for (c = 0; c < vertex_num; c++) {
          if (tmp_mat[r * vertex_num + c] > 0) {
            finished = false;
            break;
          }
        }
      }

      curr_face.pop_back();
    }

    if (!curr_face.empty()) {
      merged_faces_.emplace(curr_face_num, PolygonIdx(curr_face));
    }
  }
}

bool Crystal::IsCoplanar(size_t idx1, size_t idx2) const {
  const auto* face_norm_ptr = face_norm_.get();
  return Dot3(face_norm_ptr + idx1 * 3, face_norm_ptr + idx2 * 3) > 1 - math::kFloatEps;
}

bool Crystal::IsCounterCoplanar(size_t idx1, size_t idx2) const {
  const auto* face_norm_ptr = face_norm_.get();
  return Dot3(face_norm_ptr + idx1 * 3, face_norm_ptr + idx2 * 3) < -1 + math::kFloatEps;
}

bool Crystal::IsAdjacent(size_t idx1, size_t idx2) const {
  return MatchedVertexes(idx1, idx2) == 2;
}

bool Crystal::IsConnected(size_t idx1, size_t idx2) const {
  return MatchedVertexes(idx1, idx2) > 0;
}

int Crystal::MatchedVertexes(size_t idx1, size_t idx2) const {
  int matched_v = 0;
  for (int i = 0; i < 3; i++) {
    auto v1 = faces_[idx1].idx()[i];
    for (int j = 0; j < 3; j++) {
      auto v2 = faces_[idx2].idx()[j];
      matched_v += (v1 == v2);
    }
  }
  return matched_v;
}


CrystalPtrU Crystal::CreateHexPrism(float h) {
  using math::kPi_6;

  std::vector<Vec3f> vertexes;
  std::vector<TriangleIdx> faces;
  vertexes.reserve(12);
  faces.reserve(20);

  for (int i = 0; i < 6; ++i) {
    vertexes.emplace_back(cos(static_cast<float>(2 * i - 1) * kPi_6),  // x
                          sin(static_cast<float>(2 * i - 1) * kPi_6),  // y
                          h);                                          // z
  }
  for (int i = 0; i < 6; ++i) {
    vertexes.emplace_back(cos(static_cast<float>(2 * i - 1) * kPi_6),  // x
                          sin(static_cast<float>(2 * i - 1) * kPi_6),  // y
                          -h);                                         // z
  }

  faces.emplace_back(0, 1, 2);
  faces.emplace_back(0, 2, 3);
  faces.emplace_back(3, 4, 5);
  faces.emplace_back(3, 5, 0);
  for (int i = 0; i < 6; ++i) {
    faces.emplace_back(i, i + 6, (i + 1) % 6);
    faces.emplace_back(i + 6, (i + 1) % 6 + 6, (i + 1) % 6);
  }
  faces.emplace_back(6, 8, 7);
  faces.emplace_back(6, 9, 8);
  faces.emplace_back(9, 11, 10);
  faces.emplace_back(9, 6, 11);

  return std::unique_ptr<Crystal>(new Crystal(vertexes, faces, CrystalType::kPrism));
}


CrystalPtrU Crystal::CreateHexPyramid(float h1, float h2, float h3) {
  auto crystal = CreateHexPyramid(1, 1, 1, 1, h1, h2, h3);
  crystal->type_ = CrystalType::kPyramid_H3;
  return crystal;
}


CrystalPtrU Crystal::CreateCubicPyramid(float h1, float h2) {
  using math::kPi_2;
  using math::kPi_4;

  h1 = std::min(h1, 1.f);
  h2 = std::min(h2, 1.f);

  std::vector<Vec3f> vertexes;
  std::vector<TriangleIdx> faces;
  vertexes.reserve(12);

  for (int i = 0; i < 4; i++) {
    vertexes.emplace_back(cos(kPi_4 + kPi_2 * static_cast<float>(i)) * (1 - h1),  // x
                          sin(kPi_4 + kPi_2 * static_cast<float>(i)) * (1 - h1),  // y
                          h1);                                                    // z
  }
  for (int i = 0; i < 4; i++) {
    vertexes.emplace_back(cos(kPi_4 + kPi_2 * static_cast<float>(i)),  // x
                          sin(kPi_4 + kPi_2 * static_cast<float>(i)),  // y
                          0);                                          // z
  }
  for (int i = 0; i < 4; i++) {
    vertexes.emplace_back(cos(kPi_4 + kPi_2 * static_cast<float>(i)) * (1 - h2),  // x
                          sin(kPi_4 + kPi_2 * static_cast<float>(i)) * (1 - h2),  // y
                          -h2);                                                   // z
  }

  faces.emplace_back(0, 1, 2);
  faces.emplace_back(0, 2, 3);
  for (int i = 0; i < 4; ++i) {
    faces.emplace_back(i, i + 4, (i + 1) % 4);
    faces.emplace_back(i + 4, (i + 1) % 4 + 4, (i + 1) % 4);
  }
  for (int i = 4; i < 8; ++i) {
    faces.emplace_back(i, i + 4, (i + 1) % 4 + 4);
    faces.emplace_back(i + 4, (i + 1) % 4 + 8, (i + 1) % 4 + 4);
  }
  faces.emplace_back(9, 8, 10);
  faces.emplace_back(10, 8, 11);

  return std::unique_ptr<Crystal>(new Crystal(vertexes, faces, CrystalType::kCubicPyramid));
}


CrystalPtrU Crystal::CreateHexPyramid(int i1, int i4,                  // Miller index
                                      float h1, float h2, float h3) {  // heights
  auto crystal = CreateHexPyramid(i1, i4, i1, i4, h1, h2, h3);
  crystal->type_ = CrystalType::kPyramid_I2H3;
  return crystal;
}


// NOLINTBEGIN(readability-magic-numbers)
CrystalPtrU Crystal::CreateHexPyramid(int upper_idx1, int upper_idx4,  // upper Miller index
                                      int lower_idx1, int lower_idx4,  // lower Miller index
                                      float h1, float h2, float h3) {  // heights
  using math::kPi_6;

  float H1 = kC * static_cast<float>(upper_idx1 * 1.0 / upper_idx4);
  float H3 = kC * static_cast<float>(lower_idx1 * 1.0 / lower_idx4);
  h1 = std::max(std::min(h1, 1.0f), 0.0f);
  h3 = std::max(std::min(h3, 1.0f), 0.0f);

  std::vector<Vec3f> vertexes;
  std::vector<TriangleIdx> faces;
  vertexes.reserve(24);
  faces.reserve(44);

  for (int i = 0; i < 6; i++) {
    vertexes.emplace_back(cos(static_cast<float>(2 * i - 1) * kPi_6) * (1 - h1),  // x
                          sin(static_cast<float>(2 * i - 1) * kPi_6) * (1 - h1),  // y
                          h2 + h1 * H1);                                          // z
  }
  for (int i = 0; i < 6; i++) {
    vertexes.emplace_back(cos(static_cast<float>(2 * i - 1) * kPi_6),  // x
                          sin(static_cast<float>(2 * i - 1) * kPi_6),  // y
                          h2);                                         // z
  }
  for (int i = 0; i < 6; i++) {
    vertexes.emplace_back(cos(static_cast<float>(2 * i - 1) * kPi_6),  // x
                          sin(static_cast<float>(2 * i - 1) * kPi_6),  // y
                          -h2);                                        // z
  }
  for (int i = 0; i < 6; i++) {
    vertexes.emplace_back(cos(static_cast<float>(2 * i - 1) * kPi_6) * (1 - h3),  // x
                          sin(static_cast<float>(2 * i - 1) * kPi_6) * (1 - h3),  // y
                          -h2 - h3 * H3);                                         // z
  }

  faces.emplace_back(0, 1, 2);
  faces.emplace_back(0, 2, 3);
  faces.emplace_back(3, 4, 5);
  faces.emplace_back(3, 5, 0);
  for (int i = 0; i < 6; ++i) {
    faces.emplace_back(i, i + 6, (i + 1) % 6);
    faces.emplace_back(i + 6, (i + 1) % 6 + 6, (i + 1) % 6);
  }
  for (int i = 6; i < 12; ++i) {
    faces.emplace_back(i, i + 6, (i + 1) % 6 + 6);
    faces.emplace_back(i + 6, (i + 1) % 6 + 12, (i + 1) % 6 + 6);
  }
  for (int i = 12; i < 18; ++i) {
    faces.emplace_back(i, i + 6, (i + 1) % 6 + 12);
    faces.emplace_back(i + 6, (i + 1) % 6 + 18, (i + 1) % 6 + 12);
  }
  faces.emplace_back(18, 20, 19);
  faces.emplace_back(18, 21, 20);
  faces.emplace_back(21, 23, 22);
  faces.emplace_back(21, 18, 23);

  return std::unique_ptr<Crystal>(new Crystal(vertexes, faces, CrystalType::kPyramid_I4H3));
}

CrystalPtrU Crystal::CreateHexPyramid(float angle1, float angle2,      // wedge angles
                                      float h1, float h2, float h3) {  // heights
  using math::kDegreeToRad;
  using math::kPi_6;
  using math::kSqrt3;

  angle1 *= kDegreeToRad;
  angle2 *= kDegreeToRad;
  float tan_a1 = std::tan(angle1 / 2.0f);
  float tan_a2 = std::tan(angle2 / 2.0f);
  float H1 = kSqrt3 / 2.0f / tan_a1;
  float H3 = kSqrt3 / 2.0f / tan_a2;
  h1 = std::max(std::min(h1, 1.0f), 0.0f);
  h3 = std::max(std::min(h3, 1.0f), 0.0f);
  if (h1 * H1 + h3 * H3 + h2 * 2 < 0) {
    auto real_h1 = -h2 * 2.0f * tan_a2 / (tan_a1 + tan_a2);
    auto real_h3 = -h2 * 2.0f * tan_a1 / (tan_a1 + tan_a2);
    h1 = real_h1 / H1;
    h3 = real_h3 / H3;
  }

  std::vector<Vec3f> vertexes;
  std::vector<TriangleIdx> faces;
  vertexes.reserve(24);
  faces.reserve(44);

  for (int i = 0; i < 6; i++) {
    vertexes.emplace_back(cos(static_cast<float>(2 * i - 1) * kPi_6) * (1 - h1),  // x
                          sin(static_cast<float>(2 * i - 1) * kPi_6) * (1 - h1),  // y
                          h2 + h1 * H1);                                          // z
  }
  for (int i = 0; i < 6; i++) {
    vertexes.emplace_back(cos(static_cast<float>(2 * i - 1) * kPi_6),  // x
                          sin(static_cast<float>(2 * i - 1) * kPi_6),  // y
                          h2);                                         // z
  }
  for (int i = 0; i < 6; i++) {
    vertexes.emplace_back(cos(static_cast<float>(2 * i - 1) * kPi_6),  // x
                          sin(static_cast<float>(2 * i - 1) * kPi_6),  // y
                          -h2);                                        // z
  }
  for (int i = 0; i < 6; i++) {
    vertexes.emplace_back(cos(static_cast<float>(2 * i - 1) * kPi_6) * (1 - h3),  // x
                          sin(static_cast<float>(2 * i - 1) * kPi_6) * (1 - h3),  // y
                          -h2 - h3 * H3);                                         // z
  }

  faces.emplace_back(0, 1, 2);
  faces.emplace_back(0, 2, 3);
  faces.emplace_back(3, 4, 5);
  faces.emplace_back(3, 5, 0);
  for (int i = 0; i < 6; ++i) {
    faces.emplace_back(i, i + 6, (i + 1) % 6);
    faces.emplace_back(i + 6, (i + 1) % 6 + 6, (i + 1) % 6);
  }
  for (int i = 6; i < 12; ++i) {
    faces.emplace_back(i, i + 6, (i + 1) % 6 + 6);
    faces.emplace_back(i + 6, (i + 1) % 6 + 12, (i + 1) % 6 + 6);
  }
  for (int i = 12; i < 18; ++i) {
    faces.emplace_back(i, i + 6, (i + 1) % 6 + 12);
    faces.emplace_back(i + 6, (i + 1) % 6 + 18, (i + 1) % 6 + 12);
  }
  faces.emplace_back(18, 20, 19);
  faces.emplace_back(18, 21, 20);
  faces.emplace_back(21, 23, 22);
  faces.emplace_back(21, 18, 23);

  return std::unique_ptr<Crystal>(new Crystal(vertexes, faces, CrystalType::kPyramid_A2H3));
}
// NOLINTEND(readability-magic-numbers)


CrystalPtrU Crystal::CreateHexPyramidStackHalf(int upper_idx1, int upper_idx4,  // upper Miller index
                                               int lower_idx1, int lower_idx4,  // lower Miller index
                                               float h1, float h2, float h3) {  // heights
  using math::kPi;

  float H1 = kC * static_cast<float>(upper_idx1 * 1.0 / upper_idx4);
  float H2 = kC * static_cast<float>(lower_idx1 * 1.0 / lower_idx4);
  h1 = std::max(std::min(h1, 1.0f), 0.0f);
  h2 = std::max(std::min(h2, 1.0f), 0.0f);

  std::vector<Vec3f> vertexes;
  std::vector<TriangleIdx> faces;
  vertexes.reserve(24);
  faces.reserve(44);

  float r = (1.0f - h2) * (1.0f - h1);
  for (int i = 0; i < 6; i++) {
    vertexes.emplace_back(cos(kPi * static_cast<float>(2 * i - 1) / 6) * r,  // x
                          sin(kPi * static_cast<float>(2 * i - 1) / 6) * r,  // y
                          h1 * H1 * (1.0f - h2) + h2 * H2 + h3 * 2);         // z
  }
  r = 1.0f - h2;
  for (int i = 0; i < 6; i++) {
    vertexes.emplace_back(cos(kPi * static_cast<float>(2 * i - 1) / 6) * r,  // x
                          sin(kPi * static_cast<float>(2 * i - 1) / 6) * r,  // y
                          h2 * H2 + h3 * 2);                                 // z
  }
  for (int i = 0; i < 6; i++) {
    vertexes.emplace_back(cos(kPi * static_cast<float>(2 * i - 1) / 6),  // x
                          sin(kPi * static_cast<float>(2 * i - 1) / 6),  // y
                          h3 * 2);                                       // z
  }
  for (int i = 0; i < 6; i++) {
    vertexes.emplace_back(cos(kPi * static_cast<float>(2 * i - 1) / 6),  // x
                          sin(kPi * static_cast<float>(2 * i - 1) / 6),  // y
                          0);                                            // z
  }

  faces.emplace_back(0, 1, 2);
  faces.emplace_back(0, 2, 3);
  faces.emplace_back(3, 4, 5);
  faces.emplace_back(3, 5, 0);
  for (int i = 0; i < 6; ++i) {
    faces.emplace_back(i, i + 6, (i + 1) % 6);
    faces.emplace_back(i + 6, (i + 1) % 6 + 6, (i + 1) % 6);
  }
  for (int i = 6; i < 12; ++i) {
    faces.emplace_back(i, i + 6, (i + 1) % 6 + 6);
    faces.emplace_back(i + 6, (i + 1) % 6 + 12, (i + 1) % 6 + 6);
  }
  for (int i = 12; i < 18; ++i) {
    faces.emplace_back(i, i + 6, (i + 1) % 6 + 12);
    faces.emplace_back(i + 6, (i + 1) % 6 + 18, (i + 1) % 6 + 12);
  }
  faces.emplace_back(18, 20, 19);
  faces.emplace_back(18, 21, 20);
  faces.emplace_back(21, 23, 22);
  faces.emplace_back(21, 18, 23);

  return std::unique_ptr<Crystal>(new Crystal(vertexes, faces, CrystalType::kPyramidStackHalf));
}


CrystalPtrU Crystal::CreateIrregularHexPrism(const float* dist, float h) {
  /* Use a naive algorithm to determine the profile of basal face
   * 1. For each line pair L1 and L2, get their intersection point p12;
   * 2. For all half planes, check if p12 is in the plane;
   *  2.1 If p12 is in all half planes, put it into a set P;
   *  2.2 Else drop this point;
   * 3. Construct a convex hull from point set P.
   */
  using math::kSqrt3;

  constexpr int kConstraintNum = 8;
  constexpr int kFaceNum = 6;

  float prism_dist[kFaceNum];
  for (int i = 0; i < kFaceNum; i++) {
    prism_dist[i] = dist[i] * kSqrt3 / 2;
  }

  /* Half plane is expressed as: a*x + b*y + c*z + d <= 0 */
  /* clang-format off */
  float a[kConstraintNum] = {
    1.0f, 1.0f, -1.0f, -1.0f, -1.0f, 1.0f,  // prism faces
    0.0f, 0.0f,                             // top and bottom faces
  };
  float b[kConstraintNum] = {
    0.0f, kSqrt3, kSqrt3, 0.0f, -kSqrt3, -kSqrt3,  // prism
    0.0f, 0.0f,                                    // top and bottom
  };
  float c[kConstraintNum] = {
    0.0f, 0.0f,  0.0f, 0.0f, 0.0f, 0.0f,  // prism
    1.0f, -1.0f,                          // top and bottom
  };
  float d[kConstraintNum] = {
    -prism_dist[0], -2 * prism_dist[1], -2 * prism_dist[2],  // prism
    -prism_dist[3], -2 * prism_dist[4], -2 * prism_dist[5],  // prism
    -h,       -h,                                            // top and bottom
  };
  /* clang-format on */
  HalfSpaceSet hss(kConstraintNum, a, b, c, d);

  std::vector<Vec3f> pts = FindInnerPoints(hss);
  SortAndRemoveDuplicate(&pts);

  std::vector<TriangleIdx> faces;
  BuildPolyhedronFaces(hss, pts, faces);

  return std::unique_ptr<Crystal>(new Crystal(pts, faces, CrystalType::kIrregularPrism));
}


/* Irregular hexagon pyramid
 * parameter: dist, defines the distance from origin of each face. Must contains 6 numbers. The distance of a
 *      normal hexagon is defined as 1.
 * parameter: idx, defines the Miller index of upper and lower pyramidal segments. Must contains 4 numbers.
 * parameter: h, defines the height of each segment.
 *      h[0] and h[2] are the heights of upper and lower pyramidal segments, defined as height / H, where
 *      H is the maximum possible height.
 *      h[1] are the heights of middle cylindrical segment, defined as height / a, where a is the
 *      diameter of original basal face.
 */
CrystalPtrU Crystal::CreateIrregularHexPyramid(const float* dist, const int* idx, const float* h) {
  /* There are 20 faces. The crystal is the intersection of all these half-spaces.
   * 1. Find all inner point as vertexes.
   * 2. Find all co-planner points.
   * 3. For points in each face, construct a triangular division.
   */
  using math::kSqrt3;

  constexpr int kConstraintNum = 20;
  constexpr int kFaceNum = 6;
  constexpr int kHeightNum = 3;

  auto alpha0 = static_cast<float>(idx[1] * 1.0 / kC / idx[0] * kSqrt3);
  auto alpha1 = static_cast<float>(idx[3] * 1.0 / kC / idx[2] * kSqrt3);
  float beta0 = alpha0 * h[1];
  float beta1 = alpha1 * h[1];

  float prism_dist[kFaceNum];
  float heights[kHeightNum];
  for (int i = 0; i < kFaceNum; i++) {
    prism_dist[i] = dist[i] * kSqrt3 / 2;
  }
  for (int i = 0; i < kHeightNum; i++) {
    heights[i] = std::max(h[i], 0.0f);
  }

  float a[kConstraintNum] = {
    1.0f, 1.0f, -1.0f, -1.0f, -1.0f, 1.0f,  // prism faces
    2.0f, 1.0f, -1.0f, -2.0f, -1.0f, 1.0f,  // upper pyramid faces
    2.0f, 1.0f, -1.0f, -2.0f, -1.0f, 1.0f,  // lower pyramid faces
    0.0f, 0.0f,                             // top and bottom faces
  };
  float b[kConstraintNum] = {
    0.0f, kSqrt3, kSqrt3, 0.0f, -kSqrt3, -kSqrt3,  // prism faces
    0.0f, kSqrt3, kSqrt3, 0.0f, -kSqrt3, -kSqrt3,  // upper pyramid faces
    0.0f, kSqrt3, kSqrt3, 0.0f, -kSqrt3, -kSqrt3,  // lower pyramid faces
    0.0f, 0.0f,                                    // top and bottom faces
  };
  float c[kConstraintNum] = {
    0.0f,    0.0f,    0.0f,    0.0f,    0.0f,    0.0f,     // prism faces
    alpha0,  alpha0,  alpha0,  alpha0,  alpha0,  alpha0,   // upper pyramid faces
    -alpha1, -alpha1, -alpha1, -alpha1, -alpha1, -alpha1,  // lower pyramid faces
    1.0f,    -1.0f,                                        // top and bottom faces
  };
  float d[kConstraintNum] = {
    -prism_dist[0],
    -2 * prism_dist[1],
    -2 * prism_dist[2],
    -prism_dist[3],
    -2 * prism_dist[4],
    -2 * prism_dist[5],
    -2 * prism_dist[0] - beta0,
    -2 * prism_dist[1] - beta0,
    -2 * prism_dist[2] - beta0,
    -2 * prism_dist[3] - beta0,
    -2 * prism_dist[4] - beta0,
    -2 * prism_dist[5] - beta0,
    -2 * prism_dist[0] - beta1,
    -2 * prism_dist[1] - beta1,
    -2 * prism_dist[2] - beta1,
    -2 * prism_dist[3] - beta1,
    -2 * prism_dist[4] - beta1,
    -2 * prism_dist[5] - beta1,
    0.0f,
    0.0f,
  };
  HalfSpaceSet hss(kConstraintNum - 2, a, b, c, d);

  /* Step 1. Find all inner points */
  std::vector<Vec3f> pts = FindInnerPoints(hss);
  SortAndRemoveDuplicate(&pts);

  /* Find max and min height, then determine the height of pyramid segment */
  float maxZ = pts[0].z();
  float minZ = pts[0].z();
  for (const auto& p : pts) {
    if (p.z() > maxZ) {
      maxZ = p.z();
    }
    if (p.z() < minZ) {
      minZ = p.z();
    }
  }
  d[kConstraintNum - 2] = -(maxZ - heights[1]) * heights[0] - heights[1];
  d[kConstraintNum - 1] = (minZ + heights[1]) * heights[2] - heights[1];

  hss.n = kConstraintNum;
  pts = FindInnerPoints(hss);
  SortAndRemoveDuplicate(&pts);

  /* Step 2. Build convex hull with vertexes */
  std::vector<TriangleIdx> faces;
  BuildPolyhedronFaces(hss, pts, faces);

  return std::unique_ptr<Crystal>(new Crystal(pts, faces, CrystalType::kIrregularPyramid));
}


CrystalPtrU Crystal::CreateCustomCrystal(const std::vector<Vec3f>& pts,            // vertex points
                                         const std::vector<TriangleIdx>& faces) {  // face indices
  return std::unique_ptr<Crystal>(new Crystal(pts, faces, CrystalType::kCustom));
}


CrystalPtrU Crystal::CreateCustomCrystal(const std::vector<Vec3f>& pts,                        // vertex points
                                         const std::vector<TriangleIdx>& faces,                // face indices
                                         const std::vector<ShortIdType>& face_number_table) {  // face to face number
  return std::unique_ptr<Crystal>(new Crystal(pts, faces, face_number_table, CrystalType::kCustom));
}


namespace v3 {

Crystal Crystal::CreatePrism(float h) {
  using math::kSqrt3;
  std::unique_ptr<float[]> vtx{ new float[12 * 3]{
      kSqrt3 / 4.0f,  -1.0f / 4.0f, h / 2,   // upper: vtx1
      kSqrt3 / 4.0f,  1.0f / 4.0f,  h / 2,   // upper: vtx2
      0.0f,           1.0f / 2.0f,  h / 2,   // upper: vtx3
      -kSqrt3 / 4.0f, 1.0f / 4.0f,  h / 2,   // upper: vtx4
      -kSqrt3 / 4.0f, -1.0f / 4.0f, h / 2,   // upper: vtx5
      0.0f,           -1.0f / 2.0f, h / 2,   // upper: vtx6
      kSqrt3 / 4.0f,  -1.0f / 4.0f, -h / 2,  // lower: vtx1
      kSqrt3 / 4.0f,  1.0f / 4.0f,  -h / 2,  // lower: vtx2
      0.0f,           1.0f / 2.0f,  -h / 2,  // lower: vtx3
      -kSqrt3 / 4.0f, 1.0f / 4.0f,  -h / 2,  // lower: vtx4
      -kSqrt3 / 4.0f, -1.0f / 4.0f, -h / 2,  // lower: vtx5
      0.0f,           -1.0f / 2.0f, -h / 2,  // lower: vtx6
  } };
  std::unique_ptr<int[]> triangle_idx{ new int[20 * 3]{
      0,  1,  2,   // upper: fn1
      0,  2,  3,   // upper: fn1
      3,  4,  5,   // upper: fn1
      3,  5,  0,   // upper: fn1
      0,  6,  1,   // side: fn3
      6,  7,  1,   // side: fn3
      1,  7,  2,   // side: fn4
      7,  8,  2,   // side: fn4
      2,  8,  3,   // side: fn5
      8,  9,  3,   // side: fn5
      3,  9,  4,   // side: fn6
      9,  10, 4,   // side: fn6
      4,  10, 5,   // side: fn7
      10, 11, 5,   // side: fn7
      5,  11, 0,   // side: fn8
      11, 6,  0,   // side: fn8
      6,  8,  7,   // lower: fn2
      6,  9,  8,   // lower: fn2
      9,  11, 10,  // lower: fn2
      9,  6,  11,  // lower: fn2
  } };
  auto c = Crystal(12, vtx.get(), 20, triangle_idx.get());
  c.fn_period_ = 6;
  for (int i = 0; i < 4; i++) {
    c.fn_map_.emplace(i, 1);
    c.fn_map_.emplace(i + 16, 2);
  }
  for (int i = 0; i < 6; i++) {
    c.fn_map_.emplace(i * 2 + 4, i + 3);
    c.fn_map_.emplace(i * 2 + 5, i + 3);
  }
  return c;
}

Crystal::Crystal() : origin_{} {}

Crystal::Crystal(size_t vtx_cnt, const float* vtx, size_t triangle_cnt, const int* triangle_idx)
    : mesh_(vtx_cnt, triangle_cnt), origin_{}, face_v_(new float[triangle_cnt * 9]{}),
      face_ev_(new float[triangle_cnt * 6]{}), face_n_(new float[triangle_cnt * 3]{}),
      face_area_(new float[triangle_cnt]{}), face_coord_tf_(new float[triangle_cnt * 12]{}), fn_period_(-1) {
  mesh_.SetVtx(vtx);
  mesh_.SetTriangle(triangle_idx);

  // Initialize other pre-computed data
  ComputeCacheData();
}

Crystal::Crystal(const Crystal& other)
    : mesh_(other.mesh_), origin_{ other.origin_[0], other.origin_[1], other.origin_[2] },
      face_v_(new float[other.TotalFaces() * 9]{}), face_ev_(new float[other.TotalFaces() * 6]{}),
      face_n_(new float[other.TotalFaces() * 3]{}), face_area_(new float[other.TotalFaces()]{}),
      face_coord_tf_(new float[other.TotalFaces() * 12]), fn_map_(other.fn_map_), fn_period_(other.fn_period_) {
  auto n = other.TotalFaces();
  std::memcpy(face_v_.get(), other.face_v_.get(), n * 9 * sizeof(float));
  std::memcpy(face_ev_.get(), other.face_ev_.get(), n * 6 * sizeof(float));
  std::memcpy(face_n_.get(), other.face_n_.get(), n * 3 * sizeof(float));
  std::memcpy(face_area_.get(), other.face_area_.get(), n * sizeof(float));
  std::memcpy(face_coord_tf_.get(), other.face_coord_tf_.get(), n * 12 * sizeof(float));
}

Crystal::Crystal(Crystal&& other)
    : mesh_(std::move(other.mesh_)), origin_{ other.origin_[0], other.origin_[1], other.origin_[2] },
      face_v_(std::move(other.face_v_)), face_ev_(std::move(other.face_ev_)), face_n_(std::move(other.face_n_)),
      face_area_(std::move(other.face_area_)), face_coord_tf_(std::move(other.face_coord_tf_)),
      fn_map_(std::move(other.fn_map_)), fn_period_(other.fn_period_) {}

Crystal& Crystal::operator=(const Crystal& other) {
  if (&other == this) {
    return *this;
  }

  mesh_ = other.mesh_;
  std::memcpy(origin_, other.origin_, 3 * sizeof(float));

  auto n = other.TotalFaces();
  face_v_.reset(new float[n * 9]);
  face_ev_.reset(new float[n * 6]);
  face_n_.reset(new float[n * 3]);
  face_area_.reset(new float[n]);
  face_coord_tf_.reset(new float[n * 12]);

  std::memcpy(face_v_.get(), other.face_v_.get(), n * 9 * sizeof(float));
  std::memcpy(face_ev_.get(), other.face_ev_.get(), n * 6 * sizeof(float));
  std::memcpy(face_n_.get(), other.face_n_.get(), n * 3 * sizeof(float));
  std::memcpy(face_area_.get(), other.face_area_.get(), n * 1 * sizeof(float));
  std::memcpy(face_coord_tf_.get(), other.face_coord_tf_.get(), n * 12 * sizeof(float));

  fn_map_ = other.fn_map_;
  fn_period_ = other.fn_period_;
  return *this;
}

Crystal& Crystal::operator=(Crystal&& other) {
  if (&other == this) {
    return *this;
  }

  mesh_ = std::move(other.mesh_);
  std::memcpy(origin_, other.origin_, 3 * sizeof(float));

  face_v_ = std::move(other.face_v_);
  face_ev_ = std::move(other.face_ev_);
  face_n_ = std::move(other.face_n_);
  face_area_ = std::move(other.face_area_);
  face_coord_tf_ = std::move(other.face_coord_tf_);

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
      std::memcpy(face_v_.get() + i * 9 + j * 3, vtx + triangle_idx[i * 3 + j] * 3, 3 * sizeof(float));
    }

    // face_ev_
    for (int j = 0; j < 3; j++) {
      face_ev_[i * 6 + j + 0] = face_v_[i * 9 + j + 3] - face_v_[i * 9 + j + 0];
      face_ev_[i * 6 + j + 3] = face_v_[i * 9 + j + 6] - face_v_[i * 9 + j + 0];
    }

    // face_n_ && face_area_
    Cross3(face_ev_.get() + i * 6 + 0, face_ev_.get() + i * 6 + 3, face_n_.get() + i * 3);
    face_area_[i] = Norm3(face_n_.get() + i * 3) / 2.0f;
    Normalize3(face_n_.get() + i * 3);

    // face_coord_tf_
    Cross3(face_ev_.get() + i * 6 + 0, face_ev_.get() + i * 6 + 3, m);
    auto a = Dot3(face_n_.get() + i * 3, m);
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

    face_coord_tf_[i * 12 + 3] = -Dot3(face_coord_tf_.get() + i * 12 + 0, face_v_.get() + i * 9);
    face_coord_tf_[i * 12 + 7] = -Dot3(face_coord_tf_.get() + i * 12 + 4, face_v_.get() + i * 9);
    face_coord_tf_[i * 12 + 11] = -Dot3(face_coord_tf_.get() + i * 12 + 8, face_v_.get() + i * 9);
  }
}

size_t Crystal::TotalFaces() const {
  return mesh_.GetTriangleCnt();
}

const float* Crystal::GetFaceVtx() const {
  return face_v_.get();
}

const float* Crystal::GetFaceEdgeVec() const {
  return face_ev_.get();
}

const float* Crystal::GetFaceNorm() const {
  return face_n_.get();
}

const float* Crystal::GetFaceArea() const {
  return face_area_.get();
}

const float* Crystal::GetFaceCoordTf() const {
  return face_coord_tf_.get();
}

const float* Crystal::GetOrigin() const {
  return origin_;
}

IdType Crystal::GetFn(int fid) const {
  if (fn_map_.count(fid)) {
    return fn_map_.at(fid);
  } else {
    return kInvalidId;
  }
}

Crystal& Crystal::Rotate(const Rotation& r) {
  auto vtx_cnt = mesh_.GetVtxCnt();
  auto* vtx = mesh_.GetVtxPtr(0);
  r.Apply(vtx, vtx_cnt);
  ComputeCacheData();
  return *this;
}

Crystal& Crystal::Translate(float dx, float dy, float dz) {
  origin_[0] += dx;
  origin_[1] += dy;
  origin_[2] += dz;
  return *this;
}

std::vector<IdType> Crystal::ReduceRaypath(const std::vector<IdType>& rp, uint8_t symmetry) const {
  if (symmetry == FilterConfig::kSymNone) {
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
      pri -= first_pri;
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
        if ((2 * pri1 - pri) % fn_period_ == pri) {
          continue;
        } else if ((2 * pri1 - pri) % fn_period_ > pri) {
          break;  // Do nothing. It is already reduced.
        } else {
          pri2 = pri;
        }
      }

      pri = (2 * pri1 - pri) % fn_period_;
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
  if (symmetry == FilterConfig::kSymNone) {
    return result;
  }

  if (symmetry | FilterConfig::kSymP) {
    for (int i = 1; i < fn_period_; i++) {
      std::vector<IdType> curr_rp{ rp };
      bool changed = false;
      for (auto& x : curr_rp) {
        if (x < 3) {
          continue;
        }

        IdType pyr = x / 10;
        IdType pri = x % 10;
        pri -= 3;
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

  if (symmetry | FilterConfig::kSymD) {
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
        pri = 2 * pri0 - pri;
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

  if (symmetry | FilterConfig::kSymB) {
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

}  // namespace v3


void MakeSymmetryExtensionHelper(const RayPath& curr_ray_path, const CrystalContext* crystal_ctx, uint8_t symmetry_flag,
                                 std::vector<RayPath>& ray_path_list) {
  std::vector<RayPath> ray_path_extension{};
  ray_path_extension.emplace_back(curr_ray_path);

  // Add symmetry P.
  auto period = crystal_ctx->GetCrystal()->GetFaceNumberPeriod();
  RayPath tmp_ray_path;
  if (period > 0 && (symmetry_flag & kSymmetryPrism)) {
    decltype(ray_path_extension) ray_paths_copy(ray_path_extension);
    for (const auto& rp : ray_paths_copy) {
      for (int i = 0; i < period; i++) {
        tmp_ray_path.Clear();
        bool crystal_flag = true;
        for (auto fn : rp) {
          if (!crystal_flag && fn != kInvalidId && fn != 1 && fn != 2) {
            fn = static_cast<ShortIdType>((fn + period + i - 3) % period + 3);
          }
          crystal_flag = (fn == kInvalidId);
          tmp_ray_path << fn;
        }
        ray_path_extension.emplace_back(tmp_ray_path);
      }
    }
  }

  // Add symmetry B.
  if (symmetry_flag & kSymmetryBasal) {
    decltype(ray_path_extension) ray_paths_copy(ray_path_extension);
    for (const auto& rp : ray_paths_copy) {
      tmp_ray_path.Clear();
      bool crystal_flag = true;
      for (auto fn : rp) {
        if (!crystal_flag && fn != kInvalidId && (fn == 1 || fn == 2)) {
          fn = static_cast<ShortIdType>(fn % 2 + 1);
        }
        crystal_flag = (fn == kInvalidId);
        tmp_ray_path << fn;
      }
      ray_path_extension.emplace_back(tmp_ray_path);
    }
  }

  // Add symmetry D.
  if (period > 0 && (symmetry_flag & kSymmetryDirection)) {
    decltype(ray_path_extension) ray_paths_copy(ray_path_extension);
    for (const auto& rp : ray_paths_copy) {
      tmp_ray_path.Clear();
      bool crystal_flag = true;
      for (auto fn : rp) {
        if (!crystal_flag && fn != kInvalidId && fn != 1 && fn != 2) {
          fn = static_cast<ShortIdType>(5 + period - fn);
        }
        crystal_flag = (fn == kInvalidId);
        tmp_ray_path << fn;
      }
      ray_path_extension.emplace_back(tmp_ray_path);
    }
  }

  if (ray_path_list.empty()) {
    ray_path_list.swap(ray_path_extension);
  } else {
    std::vector<RayPath> result;
    for (const auto& rp : ray_path_list) {
      for (const auto& p : ray_path_extension) {
        result.emplace_back(rp);
        for (const auto& fn : p) {
          result.back() << fn;
        }
      }
    }
    ray_path_list.swap(result);
  }
}


RayPath::RayPath()
    : len(0), capacity(kDefaultCapacity), ids(IdPool::GetInstance()->AllocateObjectArray(capacity)),
      is_permanent(false) {}


RayPath::RayPath(size_t reserve_len)
    : len(0), capacity(reserve_len), ids(IdPool::GetInstance()->AllocateObjectArray(capacity)), is_permanent(false) {}


RayPath::RayPath(std::initializer_list<ShortIdType> ids)
    : len(ids.size()), capacity(ids.size()), ids(IdPool::GetInstance()->AllocateObjectArray(capacity)),
      is_permanent(false) {
  size_t i = 0;
  for (const auto& id : ids) {
    this->ids[i++] = id;
  }
}


RayPath::RayPath(const icehalo::RayPath& other)
    : len(other.len), capacity(other.capacity), ids(nullptr), is_permanent(other.is_permanent) {
  if (is_permanent) {
    ids = new ShortIdType[capacity];
  } else {
    ids = IdPool::GetInstance()->AllocateObjectArray(capacity);
  }
  std::memcpy(ids, other.ids, sizeof(ShortIdType) * len);
}


RayPath::RayPath(RayPath&& other) noexcept
    : len(other.len), capacity(other.capacity), ids(other.ids), is_permanent(other.is_permanent) {
  other.len = 0;
  other.capacity = 0;
  other.ids = nullptr;
}


RayPath::~RayPath() {
  if (is_permanent) {
    delete[] ids;
  }
}


RayPath& RayPath::operator=(const RayPath& other) noexcept {
  if (&other == this) {
    return *this;
  }

  len = other.len;
  capacity = other.capacity;
  if (other.is_permanent) {
    ids = new ShortIdType[capacity];
  } else {
    ids = IdPool::GetInstance()->AllocateObjectArray(capacity);
  }
  std::memcpy(ids, other.ids, sizeof(ShortIdType) * len);
  if (ids[len - 1] != kInvalidId || len > capacity) {
    LOG_WARNING("RayPath copy ass. ids wrong!");
  }
  return *this;
}


RayPath& RayPath::operator=(RayPath&& other) noexcept {
  if (&other == this) {
    return *this;
  }

  len = other.len;
  capacity = other.capacity;
  ids = other.ids;
  is_permanent = other.is_permanent;
  other.len = 0;
  other.capacity = 0;
  other.ids = nullptr;
  return *this;
}


RayPath& RayPath::operator<<(ShortIdType id) {
  if (len >= capacity) {
    ShortIdType* new_ids = nullptr;
    if (is_permanent) {
      new_ids = new ShortIdType[capacity * 2];
    } else {
      new_ids = IdPool::GetInstance()->AllocateObjectArray(capacity * 2);
    }
    std::memcpy(new_ids, ids, sizeof(ShortIdType) * len);
    if (is_permanent) {
      delete[] ids;
    }
    ids = new_ids;
    capacity *= 2;
  }
  ids[len++] = id;
  return *this;
}


bool RayPath::operator==(const RayPath& other) const noexcept {
  if (len != other.len) {
    return false;
  }

  for (size_t i = 0; i < len; i++) {
    if (ids[i] != other.ids[i]) {
      return false;
    }
  }
  return true;
}


void RayPath::Clear() {
  len = 0;
}


void RayPath::PrependId(ShortIdType id) {
  if (len >= capacity) {
    ShortIdType* new_ids = nullptr;
    if (is_permanent) {
      new_ids = new ShortIdType[capacity * 2];
    } else {
      new_ids = IdPool ::GetInstance()->AllocateObjectArray(capacity * 2);
    }
    std::memcpy(new_ids + 1, ids, sizeof(ShortIdType) * len);
    if (is_permanent) {
      delete[] ids;
    }
    ids = new_ids;
    capacity *= 2;
  } else {
    for (size_t i = 0; i < len; i++) {
      ids[len - i] = ids[len - i - 1];
    }
  }
  len++;
  ids[0] = id;
}


RayPath RayPath::MakePermanentCopy() const {
  RayPath copy(0);
  copy.len = len;
  copy.capacity = capacity;
  copy.is_permanent = true;
  copy.ids = new ShortIdType[capacity];
  std::memcpy(copy.ids, ids, sizeof(ShortIdType) * len);
  return copy;
}


std::vector<RayPath> MakeSymmetryExtension(const RayPath& curr_ray_path, const CrystalContext* crystal_ctx,
                                           uint8_t symmetry_flag) {
  std::vector<RayPath> result{};

  bool crystal_flag = true;
  RayPath tmp_ray_path;
  for (const auto& fn : curr_ray_path) {
    if (crystal_flag) {
      crystal_flag = false;
      tmp_ray_path.Clear();
    } else if (fn == kInvalidId) {
      crystal_flag = true;
    }
    tmp_ray_path << fn;
    if (crystal_flag) {
      MakeSymmetryExtensionHelper(tmp_ray_path, crystal_ctx, symmetry_flag, result);
    }
  }

  return result;
}


std::pair<RayPath, size_t> NormalizeRayPath(RayPath ray_path, const ProjectContextPtr& proj_ctx,
                                            uint8_t symmetry_flag) {
  int period = 0;
  bool crystal_flag = true;
  ShortIdType first_p_fn = kInvalidId;
  ShortIdType second_p_fn = kInvalidId;
  ShortIdType first_b_fn = kInvalidId;
  for (auto& fn : ray_path) {
    if (crystal_flag) {
      crystal_flag = false;
      const auto* crystal = proj_ctx->GetCrystal(fn);
      if (!crystal) {
        LOG_ERROR("NormalizeRayPath no crystal of ID: %u", fn);
      }
      period = crystal->GetFaceNumberPeriod();
    } else if (fn == kInvalidId) {
      crystal_flag = true;
      first_p_fn = kInvalidId;
      second_p_fn = kInvalidId;
      first_b_fn = kInvalidId;
    } else {
      bool is_basal = fn == 1 || fn == 2;
      auto pyr = fn / 10;
      fn %= 10;
      if (first_p_fn == kInvalidId && !is_basal) {
        first_p_fn = fn;
      } else if (second_p_fn == kInvalidId && !is_basal && fn != first_p_fn &&
                 ((period % 2 == 0 && (fn + period - first_p_fn) != period / 2) || period % 2 != 0)) {
        second_p_fn = fn;
      }
      if (first_b_fn == kInvalidId) {
        if (is_basal) {
          first_b_fn = fn;
        } else if (first_p_fn != kInvalidId && pyr > 0) {
          first_b_fn = pyr;
        }
      }

      if ((symmetry_flag & kSymmetryPrism) && !is_basal) {
        fn += period - first_p_fn;
        fn %= period;
      }
      if (first_p_fn != kInvalidId && second_p_fn != kInvalidId && (symmetry_flag & kSymmetryDirection) &&
          (second_p_fn + period - first_p_fn) % period > period / 2.0 && !is_basal) {
        fn = period - fn;
        fn %= period;
      }
      if ((symmetry_flag & kSymmetryBasal) && first_b_fn == 2) {
        if (is_basal) {
          fn = fn % 2 + 1;
        } else if (pyr != 0) {
          pyr = pyr % 2 + 1;
        }
      }
      if (!is_basal) {
        fn += 3 + pyr * 10;
      }
    }
  }

  return std::make_pair(std::move(ray_path), RayPathRecorder::Hash(ray_path));
}

}  // namespace icehalo
