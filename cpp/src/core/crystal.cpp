#include "core/crystal.hpp"

#include <algorithm>
#include <cstring>
#include <tuple>
#include <utility>

#include "context/context.hpp"
#include "core/optics.hpp"
#include "util/log.hpp"
#include "util/obj_pool.hpp"

namespace icehalo {

using CrystalPrimitiveNormTalbe = std::vector<std::pair<math::Vec3f, int>>;

const CrystalPrimitiveNormTalbe& GetHexFaceNormToNumberList() {
  static CrystalPrimitiveNormTalbe face_norm_to_number_list{
    { { 0, 0, 1 }, 1 },                      // top face
    { { 0, 0, -1 }, 2 },                     // bottom face
    { { 1, 0, 0 }, 3 },                      // prism face
    { { math::kSqrt3 / 2, 0.5f, 0 }, 4 },    // prism face
    { { -math::kSqrt3 / 2, 0.5f, 0 }, 5 },   // prism face
    { { -1, 0, 0 }, 6 },                     // prism face
    { { -math::kSqrt3 / 2, -0.5f, 0 }, 7 },  // prism face
    { { math::kSqrt3 / 2, -0.5f, 0 }, 8 },   // prism face
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


Crystal::Crystal(std::vector<math::Vec3f> vertexes,     // vertex
                 std::vector<math::TriangleIdx> faces,  // face indices
                 CrystalType type)                      // crystal type
    : type_(type), vertexes_(std::move(vertexes)), faces_(std::move(faces)), face_number_period_(-1),
      face_bases_(nullptr), face_vertexes_(nullptr), face_norm_(nullptr), face_area_(nullptr) {
  InitBasicData();
  InitCrystalTypeData();
  RefineFaces();
}


Crystal::Crystal(std::vector<math::Vec3f> vertexes,     // vertex
                 std::vector<math::TriangleIdx> faces,  // face indices
                 FaceNumberTable face_number_table,     // face to face number
                 CrystalType type)                      // crystal type
    : type_(type), vertexes_(std::move(vertexes)), faces_(std::move(faces)),
      face_number_table_(std::move(face_number_table)), face_number_period_(-1), face_bases_(nullptr),
      face_vertexes_(nullptr), face_norm_(nullptr), face_area_(nullptr) {
  InitBasicData();
}


const std::vector<math::Vec3f>& Crystal::GetVertexes() const {
  return vertexes_;
}


const std::vector<math::TriangleIdx>& Crystal::GetFaces() const {
  return faces_;
}


const Crystal::FaceNumberTable& Crystal::GetFaceNumberTable() const {
  return face_number_table_;
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
  using math::Vec3f;

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
    math::Vec3FromTo(vertexes_[idx[0]].val(), vertexes_[idx[1]].val(), face_bases_ptr + i * 6 + 0);
    math::Vec3FromTo(vertexes_[idx[0]].val(), vertexes_[idx[2]].val(), face_bases_ptr + i * 6 + 3);
    math::Cross3(face_bases_ptr + i * 6 + 0, face_bases_ptr + i * 6 + 3, face_norm_ptr + i * 3);

    face_area_[i] = math::Norm3(face_norm_ptr + i * 3) / 2;
    math::Normalize3(face_norm_ptr + i * 3);

    std::memcpy(face_vertexes_ptr + i * 9 + 0, vertexes_[idx[0]].val(), 3 * sizeof(float));
    std::memcpy(face_vertexes_ptr + i * 9 + 3, vertexes_[idx[1]].val(), 3 * sizeof(float));
    std::memcpy(face_vertexes_ptr + i * 9 + 6, vertexes_[idx[2]].val(), 3 * sizeof(float));
  }
}

void Crystal::InitCrystalTypeData() {
  switch (type_) {
    case CrystalType::kPrism:
      InitFaceNumberHexPrism();
      face_number_period_ = 6;
      break;
    case CrystalType::kPyramid_H3:
    case CrystalType::kPyramid_I2H3:
    case CrystalType::kPyramid_I4H3:
    case CrystalType::kPyramid_A2H3:
      InitFaceNumberHexPyramid();
      face_number_period_ = 6;
      break;
    case CrystalType::kIrregularPyramid:
    case CrystalType::kIrregularPrism:
      InitFaceNumberIrregularHex();
      face_number_period_ = 6;
      break;
    case CrystalType::kCubicPyramid:
      InitFaceNumberCubic();
      face_number_period_ = 4;
      break;
    case CrystalType::kPyramidStackHalf:
      InitFaceNumberStack();
      face_number_period_ = 6;
      break;
    case CrystalType::kCustom:
    case CrystalType::kUnknown:
      break;
  }
}

std::tuple<float, int> FindMatchedPrimitiveNorm(const float* norm,
                                                const CrystalPrimitiveNormTalbe& primitive_norm_table) {
  float max_val = -1;
  int max_face_number = -1;
  for (const auto& d : primitive_norm_table) {
    float curr_val = math::Dot3(norm, d.first.val());
    if (curr_val > max_val) {
      max_val = curr_val;
      max_face_number = d.second;
    }
  }
  return { max_val, max_face_number };
}

void Crystal::InitFaceNumberHexPrism() {
  face_number_table_.clear();
  for (int i = 0; i < 4; i++) {
    face_number_table_.emplace_back(1);
  }
  for (int i = 0; i < 6; i++) {
    face_number_table_.emplace_back(i + 3);
    face_number_table_.emplace_back(i + 3);
  }
  for (int i = 0; i < 4; i++) {
    face_number_table_.emplace_back(2);
  }
}


void Crystal::InitFaceNumberHexPyramid() {
  face_number_table_.clear();
  for (int i = 0; i < 4; i++) {
    face_number_table_.emplace_back(1);
  }
  for (int i = 0; i < 6; i++) {
    face_number_table_.emplace_back(i + 13);
    face_number_table_.emplace_back(i + 13);
  }
  for (int i = 0; i < 6; i++) {
    face_number_table_.emplace_back(i + 3);
    face_number_table_.emplace_back(i + 3);
  }
  for (int i = 0; i < 6; i++) {
    face_number_table_.emplace_back(i + 23);
    face_number_table_.emplace_back(i + 23);
  }
  for (int i = 0; i < 4; i++) {
    face_number_table_.emplace_back(2);
  }
}

void Crystal::InitFaceNumberIrregularHex() {
  for (size_t i = 0; i < faces_.size(); i++) {
    const auto* curr_face_norm = face_norm_.get() + i * 3;
    float max_val = -1;
    int max_face_number = -1;
    std::tie(max_val, max_face_number) = FindMatchedPrimitiveNorm(curr_face_norm, GetHexFaceNormToNumberList());

    if (max_val > 0) {
      if (std::abs(max_val - 1) > math::kFloatEps && curr_face_norm[2] > math::kFloatEps) {
        max_face_number += 10;
      } else if (std::abs(max_val - 1) > math::kFloatEps && curr_face_norm[2] < -math::kFloatEps) {
        max_face_number += 20;
      }
    }
    face_number_table_.push_back(max_face_number);
  }
}

void Crystal::InitFaceNumberCubic() {
  for (size_t i = 0; i < faces_.size(); i++) {
    const auto* curr_face_norm = face_norm_.get() + i * 3;
    float max_val = -1;
    int max_face_number = -1;
    std::tie(max_val, max_face_number) = FindMatchedPrimitiveNorm(curr_face_norm, GetCubicFaceNormToNumberList());

    if (max_val > 0) {
      if (std::abs(max_val - 1) > math::kFloatEps && curr_face_norm[2] > math::kFloatEps) {
        max_face_number += 10;
      } else if (std::abs(max_val - 1) > math::kFloatEps && curr_face_norm[2] < -math::kFloatEps) {
        max_face_number += 20;
      }
    }
    face_number_table_.push_back(max_face_number);
  }
}

void Crystal::InitFaceNumberStack() {
  float max_height = std::numeric_limits<float>::lowest();
  float min_height = std::numeric_limits<float>::max();
  for (size_t i = 0; i < faces_.size(); i++) {
    const auto* curr_face_norm = face_norm_.get() + i * 3;
    float max_val = -1;
    int max_face_number = -1;
    for (const auto& d : GetHexFaceNormToNumberList()) {
      float tmp_val = math::Dot3(curr_face_norm, d.first.val());
      if (tmp_val > max_val) {
        max_val = tmp_val;
        max_face_number = d.second;
      }
    }
    face_number_table_.push_back(max_face_number);

    const auto* idx = faces_[i].idx();
    for (int j = 0; j < 3; j++) {
      float h = vertexes_[idx[j]].z();
      if (h > max_height) {
        max_height = h;
      }
      if (h < min_height) {
        min_height = h;
      }
    }
  }

  for (size_t i = 0; i < faces_.size(); i++) {
    if (face_number_table_[i] < 3) {
      continue;
    }
    const auto* idx = faces_[i].idx();
    const auto* curr_face_norm = face_norm_.get() + i * 3;

    bool is_top = false;
    bool is_bottom = false;
    bool is_upper = true;
    bool is_lower = true;
    for (int j = 0; j < 3; j++) {
      is_top = is_top || std::abs(vertexes_[idx[j]].z() - max_height) < math::kFloatEps;
      is_bottom = is_bottom || std::abs(vertexes_[idx[j]].z() - min_height) < math::kFloatEps;
      is_upper = is_upper && vertexes_[idx[j]].z() > math::kFloatEps;
      is_lower = is_lower && vertexes_[idx[j]].z() < -math::kFloatEps;
    }

    bool is_prism = std::abs(curr_face_norm[2]) < math::kFloatEps;

    if (is_top && !is_prism) {
      face_number_table_[i] += 10;
    } else if (is_bottom && !is_prism) {
      face_number_table_[i] += 20;
    } else if (is_upper && !is_prism) {
      face_number_table_[i] += 30;
    } else if (is_lower && !is_prism) {
      face_number_table_[i] += 40;
    }
  }
}

void Crystal::RefineFaces() {
  using math::Vec3f;
  size_t faces = faces_.size();
  // Remove those parts whose thickness is 0
  std::vector<bool> face_remove(faces);
  std::fill(face_remove.begin(), face_remove.end(), false);

  for (size_t i = 0; i < faces; i++) {
    if (face_remove[i]) {
      continue;
    }
    if (face_area_[i] < math::kFloatEps) {
      face_remove[i] = true;
      continue;
    }
    const auto* n1 = face_norm_.get() + i * 3;
    Vec3f v1(face_vertexes_.get() + i * 9 + 0 * 3);
    for (size_t j = i + 1; j < faces; j++) {
      const auto* n2 = face_norm_.get() + j * 3;
      if (face_remove[j] || face_number_table_[i] == face_number_table_[j] ||
          std::abs(math::Dot3(n1, n2)) < 1 - math::kFloatEps) {
        continue;
      }
      if (face_area_[j] < math::kFloatEps) {
        face_remove[j] = true;
        continue;
      }
      float pd = -1, d = -1;
      int tmp_idx = 0;
      while (pd <= math::kFloatEps && tmp_idx < 3) {
        const auto* vp2 = face_vertexes_.get() + j * 9 + tmp_idx * 3;
        auto vd = Vec3f::FromTo(v1, Vec3f(vp2));
        pd = math::Norm3(vd.val());
        d = std::abs(math::Dot3(vd.val(), n1));
        tmp_idx++;
      }
      float min_pd = std::numeric_limits<float>::max();
      for (int i1 = 0; i1 < 3; i1++) {
        for (int i2 = 0; i2 < 3; i2++) {
          const auto* tmp_p1 = face_vertexes_.get() + i * 9 + i1 * 3;
          const auto* tmp_p2 = face_vertexes_.get() + j * 9 + i2 * 3;
          auto tmp_d = math::DiffNorm3(tmp_p1, tmp_p2);
          if (tmp_d < min_pd) {
            min_pd = tmp_d;
          }
        }
      }
      if (d < math::kFloatEps && min_pd < math::kFloatEps) {
        for (size_t i1 = 0; i1 < faces; i1++) {
          if (face_number_table_[i1] == face_number_table_[j] || face_number_table_[i1] == face_number_table_[i]) {
            face_remove[i1] = true;
          }
        }
      }
    }
  }

  size_t new_face_num = 0;
  for (auto r : face_remove) {
    if (!r) {
      new_face_num++;
    }
  }

  decltype(faces_) new_faces;
  decltype(face_number_table_) new_face_number_table;

  std::unique_ptr<float[]> new_face_bases{ new float[new_face_num * 6]{} };
  std::unique_ptr<float[]> new_face_vertexes{ new float[new_face_num * 9]{} };
  std::unique_ptr<float[]> new_face_norm{ new float[new_face_num * 3]{} };
  std::unique_ptr<float[]> new_face_area{ new float[new_face_num]{} };

  for (size_t i = 0, j = 0; i < faces; i++) {
    if (!face_remove[i]) {
      new_faces.emplace_back(faces_[i]);
      new_face_number_table.emplace_back(face_number_table_[i]);
      std::memcpy(new_face_bases.get() + j * 6, face_bases_.get() + i * 6, 6 * sizeof(float));
      std::memcpy(new_face_vertexes.get() + j * 9, face_vertexes_.get() + i * 9, 9 * sizeof(float));
      std::memcpy(new_face_norm.get() + j * 3, face_norm_.get() + i * 3, 3 * sizeof(float));
      new_face_area[j] = face_area_[i];
      j++;
    }
  }

  faces_.swap(new_faces);
  face_number_table_.swap(new_face_number_table);
  face_bases_ = std::move(new_face_bases);
  face_vertexes_ = std::move(new_face_vertexes);
  face_norm_ = std::move(new_face_norm);
  face_area_ = std::move(new_face_area);
}


CrystalPtrU Crystal::CreateHexPrism(float h) {
  using math::kPi;
  using math::TriangleIdx;
  using math::Vec3f;

  std::vector<Vec3f> vertexes;
  std::vector<TriangleIdx> faces;
  vertexes.reserve(12);
  faces.reserve(20);

  for (int i = 0; i < 6; ++i) {
    vertexes.emplace_back(cos(static_cast<float>(2 * i - 1) * kPi / 6),  // x
                          sin(static_cast<float>(2 * i - 1) * kPi / 6),  // y
                          h);                                            // z
  }
  for (int i = 0; i < 6; ++i) {
    vertexes.emplace_back(cos(static_cast<float>(2 * i - 1) * kPi / 6),  // x
                          sin(static_cast<float>(2 * i - 1) * kPi / 6),  // y
                          -h);                                           // z
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
  using math::kPi;
  using math::TriangleIdx;
  using math::Vec3f;

  h1 = std::min(h1, 1.f);
  h2 = std::min(h2, 1.f);

  std::vector<Vec3f> vertexes;
  std::vector<TriangleIdx> faces;
  vertexes.reserve(12);

  for (int i = 0; i < 4; i++) {
    vertexes.emplace_back(cos(kPi / 4 + kPi / 2 * static_cast<float>(i)) * (1 - h1),  // x
                          sin(kPi / 4 + kPi / 2 * static_cast<float>(i)) * (1 - h1),  // y
                          h1);                                                        // z
  }
  for (int i = 0; i < 4; i++) {
    vertexes.emplace_back(cos(kPi / 4 + kPi / 2 * static_cast<float>(i)),  // x
                          sin(kPi / 4 + kPi / 2 * static_cast<float>(i)),  // y
                          0);                                              // z
  }
  for (int i = 0; i < 4; i++) {
    vertexes.emplace_back(cos(kPi / 4 + kPi / 2 * static_cast<float>(i)) * (1 - h2),  // x
                          sin(kPi / 4 + kPi / 2 * static_cast<float>(i)) * (1 - h2),  // y
                          -h2);                                                       // z
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


CrystalPtrU Crystal::CreateHexPyramid(int upper_idx1, int upper_idx4,  // upper Miller index
                                      int lower_idx1, int lower_idx4,  // lower Miller index
                                      float h1, float h2, float h3) {  // heights
  using math::kPi;
  using math::TriangleIdx;
  using math::Vec3f;

  float H1 = kC * static_cast<float>(upper_idx1 * 1.0 / upper_idx4);
  float H3 = kC * static_cast<float>(lower_idx1 * 1.0 / lower_idx4);
  h1 = std::max(std::min(h1, 1.0f), 0.0f);
  h3 = std::max(std::min(h3, 1.0f), 0.0f);

  std::vector<Vec3f> vertexes;
  std::vector<TriangleIdx> faces;
  vertexes.reserve(24);
  faces.reserve(44);

  for (int i = 0; i < 6; i++) {
    vertexes.emplace_back(cos(static_cast<float>(2 * i - 1) * kPi / 6) * (1 - h1),  // x
                          sin(static_cast<float>(2 * i - 1) * kPi / 6) * (1 - h1),  // y
                          h2 + h1 * H1);                                            // z
  }
  for (int i = 0; i < 6; i++) {
    vertexes.emplace_back(cos(static_cast<float>(2 * i - 1) * kPi / 6),  // x
                          sin(static_cast<float>(2 * i - 1) * kPi / 6),  // y
                          h2);                                           // z
  }
  for (int i = 0; i < 6; i++) {
    vertexes.emplace_back(cos(static_cast<float>(2 * i - 1) * kPi / 6),  // x
                          sin(static_cast<float>(2 * i - 1) * kPi / 6),  // y
                          -h2);                                          // z
  }
  for (int i = 0; i < 6; i++) {
    vertexes.emplace_back(cos(static_cast<float>(2 * i - 1) * kPi / 6) * (1 - h3),  // x
                          sin(static_cast<float>(2 * i - 1) * kPi / 6) * (1 - h3),  // y
                          -h2 - h3 * H3);                                           // z
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
  using math::kPi;
  using math::kSqrt3;
  using math::TriangleIdx;
  using math::Vec3f;

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
    vertexes.emplace_back(cos(static_cast<float>(2 * i - 1) * kPi / 6) * (1 - h1),  // x
                          sin(static_cast<float>(2 * i - 1) * kPi / 6) * (1 - h1),  // y
                          h2 + h1 * H1);                                            // z
  }
  for (int i = 0; i < 6; i++) {
    vertexes.emplace_back(cos(static_cast<float>(2 * i - 1) * kPi / 6),  // x
                          sin(static_cast<float>(2 * i - 1) * kPi / 6),  // y
                          h2);                                           // z
  }
  for (int i = 0; i < 6; i++) {
    vertexes.emplace_back(cos(static_cast<float>(2 * i - 1) * kPi / 6),  // x
                          sin(static_cast<float>(2 * i - 1) * kPi / 6),  // y
                          -h2);                                          // z
  }
  for (int i = 0; i < 6; i++) {
    vertexes.emplace_back(cos(static_cast<float>(2 * i - 1) * kPi / 6) * (1 - h3),  // x
                          sin(static_cast<float>(2 * i - 1) * kPi / 6) * (1 - h3),  // y
                          -h2 - h3 * H3);                                           // z
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


CrystalPtrU Crystal::CreateHexPyramidStackHalf(int upper_idx1, int upper_idx4,  // upper Miller index
                                               int lower_idx1, int lower_idx4,  // lower Miller index
                                               float h1, float h2, float h3) {  // heights
  using math::kPi;
  using math::TriangleIdx;
  using math::Vec3f;

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
    vertexes.emplace_back(cos(2 * kPi * static_cast<float>(i) / 6) * r,  // x
                          sin(2 * kPi * static_cast<float>(i) / 6) * r,  // y
                          h1 * H1 * (1.0f - h2) + h2 * H2 + h3 * 2);     // z
  }
  r = 1.0f - h2;
  for (int i = 0; i < 6; i++) {
    vertexes.emplace_back(cos(2 * kPi * static_cast<float>(i) / 6) * r,  // x
                          sin(2 * kPi * static_cast<float>(i) / 6) * r,  // y
                          h2 * H2 + h3 * 2);                             // z
  }
  for (int i = 0; i < 6; i++) {
    vertexes.emplace_back(cos(2 * kPi * static_cast<float>(i) / 6),  // x
                          sin(2 * kPi * static_cast<float>(i) / 6),  // y
                          h3 * 2);                                   // z
  }
  for (int i = 0; i < 6; i++) {
    vertexes.emplace_back(cos(2 * kPi * static_cast<float>(i) / 6),  // x
                          sin(2 * kPi * static_cast<float>(i) / 6),  // y
                          0);                                        // z
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
  using math::HalfSpaceSet;
  using math::kSqrt3;
  using math::TriangleIdx;
  using math::Vec3f;

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
  using math::HalfSpaceSet;
  using math::kSqrt3;
  using math::TriangleIdx;
  using math::Vec3f;

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


CrystalPtrU Crystal::CreateCustomCrystal(const std::vector<math::Vec3f>& pts,            // vertex points
                                         const std::vector<math::TriangleIdx>& faces) {  // face indices
  return std::unique_ptr<Crystal>(new Crystal(pts, faces, CrystalType::kCustom));
}


CrystalPtrU Crystal::CreateCustomCrystal(const std::vector<math::Vec3f>& pts,          // vertex points
                                         const std::vector<math::TriangleIdx>& faces,  // face indices
                                         const FaceNumberTable& face_number_table) {   // face to face number
  return std::unique_ptr<Crystal>(new Crystal(pts, faces, face_number_table, CrystalType::kCustom));
}


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
  if (ids[len - 1] != kInvalidId || len > capacity) {
    LOG_WARNING("RayPath copy ctr. ids wrong!");
  }
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
  if (ids[len - 1] != kInvalidId || len > capacity) {
    LOG_WARNING("RayPath::PrependId() ids wrong!");
  }
}


RayPath RayPath::MakePermanentCopy() const {
  RayPath copy(0);
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
        } else if (first_p_fn != kInvalidId) {
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
