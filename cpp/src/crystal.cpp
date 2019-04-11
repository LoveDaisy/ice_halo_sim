#include "crystal.h"

#include <algorithm>
#include <cassert>
#include <cstring>
#include <utility>

namespace IceHalo {

Crystal::Crystal(std::vector<Math::Vec3f> vertexes,     // vertex
                 std::vector<Math::TriangleIdx> faces,  // face indices
                 CrystalType type)                      // crystal type
    : vertexes_(std::move(vertexes)), faces_(std::move(faces)), type_(type), face_number_period_(-1),
      face_bases_(nullptr), face_vertexes_(nullptr), face_norm_(nullptr), face_area_(nullptr) {
  InitNorm();
  InitFaceNumber();
  switch (type_) {
    case CrystalType::PRISM:
    case CrystalType::PYRAMID:
    case CrystalType::STACK_PYRAMID:
      face_number_period_ = 6;
      break;
    case CrystalType::CUBIC_PYRAMID:
      face_number_period_ = 4;
      break;
    case CrystalType::CUSTOM:
    case CrystalType::UNKNOWN:
    default:
      break;
  }
}


Crystal::Crystal(std::vector<IceHalo::Math::Vec3f> vertexes,     // vertex
                 std::vector<IceHalo::Math::TriangleIdx> faces,  // face indices
                 std::vector<int> face_number_map,               // face to face number
                 CrystalType type)                               // crystal type
    : vertexes_(std::move(vertexes)), faces_(std::move(faces)), face_number_map_(std::move(face_number_map)),
      type_(type), face_number_period_(-1), face_bases_(nullptr), face_vertexes_(nullptr), face_norm_(nullptr),
      face_area_(nullptr) {
  InitNorm();
}


Crystal::~Crystal() {
  delete[] face_bases_;
  delete[] face_vertexes_;
  delete[] face_norm_;
  delete[] face_area_;
}


const std::vector<Math::Vec3f>& Crystal::GetVertexes() {
  return vertexes_;
}


const std::vector<Math::TriangleIdx>& Crystal::GetFaces() {
  return faces_;
}


const std::vector<int>& Crystal::GetFaceNumberMap() {
  return face_number_map_;
}


const float* Crystal::GetFaceVertex() const {
  return face_vertexes_;
}


const float* Crystal::GetFaceBaseVector() const {
  return face_bases_;
}


const float* Crystal::GetFaceNorm() const {
  return face_norm_;
}


const float* Crystal::GetFaceArea() const {
  return face_area_;
}


int Crystal::GetFaceNumberPeriod() const {
  return face_number_period_;
}


int Crystal::TotalVertexes() const {
  return static_cast<int>(vertexes_.size());
}

int Crystal::TotalFaces() const {
  return static_cast<int>(faces_.size());
}

int Crystal::FaceNumber(int idx) const {
  if (face_number_map_.empty()) {
    return -1;
  } else if (idx < 0 || static_cast<size_t>(idx) >= face_number_map_.size()) {
    return -1;
  } else {
    return face_number_map_[idx];
  }
}


void Crystal::InitNorm() {
  using Math::Vec3f;

  auto face_num = faces_.size();
  face_bases_ = new float[face_num * 6];
  face_vertexes_ = new float[face_num * 9];
  face_norm_ = new float[face_num * 3];
  face_area_ = new float[face_num];

  for (decltype(faces_.size()) i = 0; i < faces_.size(); i++) {
    const auto& f = faces_[i];
    auto idx = f.idx();
    Math::Vec3FromTo(vertexes_[idx[0]].val(), vertexes_[idx[1]].val(), face_bases_ + i * 6 + 0);
    Math::Vec3FromTo(vertexes_[idx[0]].val(), vertexes_[idx[2]].val(), face_bases_ + i * 6 + 3);
    Math::Cross3(face_bases_ + i * 6 + 0, face_bases_ + i * 6 + 3, face_norm_ + i * 3);

    face_area_[i] = Math::Norm3(face_norm_ + i * 3) / 2;
    Math::Normalize3(face_norm_ + i * 3);

    std::memcpy(face_vertexes_ + i * 9 + 0, vertexes_[idx[0]].val(), 3 * sizeof(float));
    std::memcpy(face_vertexes_ + i * 9 + 3, vertexes_[idx[1]].val(), 3 * sizeof(float));
    std::memcpy(face_vertexes_ + i * 9 + 6, vertexes_[idx[2]].val(), 3 * sizeof(float));
  }
}

void Crystal::InitFaceNumber() {
  switch (type_) {
    case CrystalType::PRISM:
    case CrystalType::PYRAMID:
      InitFaceNumberHex();
      break;
    case CrystalType::CUBIC_PYRAMID:
      InitFaceNumberCubic();
      break;
    case CrystalType::STACK_PYRAMID:
      InitFaceNumberStack();
      break;
    case CrystalType::CUSTOM:
    case CrystalType::UNKNOWN:
      break;
  }
}

void Crystal::InitFaceNumberHex() {
  for (decltype(faces_.size()) i = 0; i < faces_.size(); i++) {
    const auto curr_face_norm = face_norm_ + i * 3;
    float max_val = -1;
    int max_face_number = -1;
    for (const auto& d : GetHexFaceNormToNumberList()) {
      float tmpVal = Math::Dot3(curr_face_norm, d.first.val());
      if (tmpVal > max_val) {
        max_val = tmpVal;
        max_face_number = d.second;
      }
    }

    if (max_val > 0) {
      if (std::abs(max_val - 1) > Math::kFloatEps && curr_face_norm[2] > Math::kFloatEps) {
        max_face_number += 10;
      } else if (std::abs(max_val - 1) > Math::kFloatEps && curr_face_norm[2] < -Math::kFloatEps) {
        max_face_number += 20;
      }
    }
    face_number_map_.push_back(max_face_number);
  }
}

void Crystal::InitFaceNumberCubic() {
  for (decltype(faces_.size()) i = 0; i < faces_.size(); i++) {
    const auto curr_face_norm = face_norm_ + i * 3;
    float max_val = -1;
    int max_face_number = -1;
    for (const auto& d : GetCubicFaceNormToNumberList()) {
      float tmpVal = Math::Dot3(curr_face_norm, d.first.val());
      if (tmpVal > max_val) {
        max_val = tmpVal;
        max_face_number = d.second;
      }
    }

    if (max_val > 0) {
      if (std::abs(max_val - 1) > Math::kFloatEps && curr_face_norm[2] > Math::kFloatEps) {
        max_face_number += 10;
      } else if (std::abs(max_val - 1) > Math::kFloatEps && curr_face_norm[2] < -Math::kFloatEps) {
        max_face_number += 20;
      }
    }
    face_number_map_.push_back(max_face_number);
  }
}

void Crystal::InitFaceNumberStack() {
  float max_height = std::numeric_limits<float>::lowest();
  float min_height = std::numeric_limits<float>::max();
  for (decltype(faces_.size()) i = 0; i < faces_.size(); i++) {
    const auto curr_face_norm = face_norm_ + i * 3;
    float max_val = -1;
    int max_face_number = -1;
    for (const auto& d : GetHexFaceNormToNumberList()) {
      float tmp_val = Math::Dot3(curr_face_norm, d.first.val());
      if (tmp_val > max_val) {
        max_val = tmp_val;
        max_face_number = d.second;
      }
    }
    face_number_map_.push_back(max_face_number);

    const auto idx = faces_[i].idx();
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

  for (decltype(faces_.size()) i = 0; i < faces_.size(); i++) {
    if (face_number_map_[i] < 3) {
      continue;
    }
    const auto* idx = faces_[i].idx();
    const auto curr_face_norm = face_norm_ + i * 3;

    bool is_top = false;
    bool is_bottom = false;
    bool is_upper = true;
    bool is_lower = true;
    for (int j = 0; j < 3; j++) {
      is_top = is_top || std::abs(vertexes_[idx[j]].z() - max_height) < Math::kFloatEps;
      is_bottom = is_bottom || std::abs(vertexes_[idx[j]].z() - min_height) < Math::kFloatEps;
      is_upper = is_upper && vertexes_[idx[j]].z() > Math::kFloatEps;
      is_lower = is_lower && vertexes_[idx[j]].z() < -Math::kFloatEps;
    }

    bool is_prism = std::abs(curr_face_norm[2]) < Math::kFloatEps;

    if (is_top && !is_prism) {
      face_number_map_[i] += 10;
    } else if (is_bottom && !is_prism) {
      face_number_map_[i] += 20;
    } else if (is_upper && !is_prism) {
      face_number_map_[i] += 30;
    } else if (is_lower && !is_prism) {
      face_number_map_[i] += 40;
    }
  }
}


const std::vector<std::pair<Math::Vec3f, int>>& Crystal::GetHexFaceNormToNumberList() {
  static std::vector<std::pair<Math::Vec3f, int>> face_norm_to_number_list{
    { { 0, 0, 1 }, 1 },                      // top face
    { { 0, 0, -1 }, 2 },                     // bottom face
    { { 1, 0, 0 }, 3 },                      // prism face
    { { Math::kSqrt3 / 2, 0.5f, 0 }, 4 },    // prism face
    { { -Math::kSqrt3 / 2, 0.5f, 0 }, 5 },   // prism face
    { { -1, 0, 0 }, 6 },                     // prism face
    { { -Math::kSqrt3 / 2, -0.5f, 0 }, 7 },  // prism face
    { { Math::kSqrt3 / 2, -0.5f, 0 }, 8 },   // prism face
  };
  return face_norm_to_number_list;
}


const std::vector<std::pair<Math::Vec3f, int>>& Crystal::GetCubicFaceNormToNumberList() {
  static std::vector<std::pair<Math::Vec3f, int>> face_norm_to_number_list{
    { { 0, 0, 1 }, 1 },   // top face
    { { 0, 0, -1 }, 2 },  // bottom face
    { { 1, 0, 0 }, 3 },   // pyramidal face
    { { 0, 1, 0 }, 4 },   // pyramidal face
    { { -1, 0, 0 }, 5 },  // pyramidal face
    { { 0, -1, 0 }, 6 },  // pyramidal face
  };
  return face_norm_to_number_list;
}


CrystalPtrU Crystal::CreateHexPrism(float h) {
  using Math::kPi;
  using Math::TriangleIdx;
  using Math::Vec3f;

  std::vector<Vec3f> vertexes;
  std::vector<TriangleIdx> faces;
  vertexes.reserve(12);
  faces.reserve(20);

  for (int i = 0; i < 6; ++i) {
    vertexes.emplace_back(cos((2 * i - 1) * kPi / 6),  // x
                          sin((2 * i - 1) * kPi / 6),  // y
                          h);                          // z
  }
  for (int i = 0; i < 6; ++i) {
    vertexes.emplace_back(cos((2 * i - 1) * kPi / 6),  // x
                          sin((2 * i - 1) * kPi / 6),  // y
                          -h);                         // z
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

  return std::unique_ptr<Crystal>(new Crystal(vertexes, faces, CrystalType::PRISM));
}


CrystalPtrU Crystal::CreateHexPyramid(float h1, float h2, float h3) {
  return CreateHexPyramid(1, 1, 1, 1, h1, h2, h3);
}


CrystalPtrU Crystal::CreateCubicPyramid(float h1, float h2) {
  using Math::kPi;
  using Math::TriangleIdx;
  using Math::Vec3f;

  h1 = std::min(h1, 1.f);
  h2 = std::min(h2, 1.f);

  std::vector<Vec3f> vertexes;
  std::vector<TriangleIdx> faces;
  vertexes.reserve(12);

  for (int i = 0; i < 4; i++) {
    vertexes.emplace_back(cos(kPi / 4 + kPi / 2 * i) * (1 - h1),  // x
                          sin(kPi / 4 + kPi / 2 * i) * (1 - h1),  // y
                          h1);                                    // z
  }
  for (int i = 0; i < 4; i++) {
    vertexes.emplace_back(cos(kPi / 4 + kPi / 2 * i),  // x
                          sin(kPi / 4 + kPi / 2 * i),  // y
                          0);                          // z
  }
  for (int i = 0; i < 4; i++) {
    vertexes.emplace_back(cos(kPi / 4 + kPi / 2 * i) * (1 - h2),  // x
                          sin(kPi / 4 + kPi / 2 * i) * (1 - h2),  // y
                          -h2);                                   // z
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

  return std::unique_ptr<Crystal>(new Crystal(vertexes, faces, CrystalType::CUBIC_PYRAMID));
}


CrystalPtrU Crystal::CreateHexPyramid(int i1, int i4,                  // Miller index
                                      float h1, float h2, float h3) {  // heights
  return CreateHexPyramid(i1, i4, i1, i4, h1, h2, h3);
}


CrystalPtrU Crystal::CreateHexPyramid(int upper_idx1, int upper_idx4,  // upper Miller index
                                      int lower_idx1, int lower_idx4,  // lower Miller index
                                      float h1, float h2, float h3) {  // heights
  using Math::kPi;
  using Math::TriangleIdx;
  using Math::Vec3f;

  float H1 = kC * upper_idx1 / upper_idx4;
  float H3 = kC * lower_idx1 / lower_idx4;
  h1 = std::max(std::min(h1, 1.0f), 0.0f);
  h3 = std::max(std::min(h3, 1.0f), 0.0f);

  std::vector<Vec3f> vertexes;
  std::vector<TriangleIdx> faces;
  vertexes.reserve(24);
  faces.reserve(44);

  for (int i = 0; i < 6; i++) {
    vertexes.emplace_back(cos((2 * i - 1) * kPi / 6) * (1 - h1),  // x
                          sin((2 * i - 1) * kPi / 6) * (1 - h1),  // y
                          h2 + h1 * H1);                          // z
  }
  for (int i = 0; i < 6; i++) {
    vertexes.emplace_back(cos((2 * i - 1) * kPi / 6),  // x
                          sin((2 * i - 1) * kPi / 6),  // y
                          h2);                         // z
  }
  for (int i = 0; i < 6; i++) {
    vertexes.emplace_back(cos((2 * i - 1) * kPi / 6),  // x
                          sin((2 * i - 1) * kPi / 6),  // y
                          -h2);                        // z
  }
  for (int i = 0; i < 6; i++) {
    vertexes.emplace_back(cos((2 * i - 1) * kPi / 6) * (1 - h3),  // x
                          sin((2 * i - 1) * kPi / 6) * (1 - h3),  // y
                          -h2 - h3 * H3);                         // z
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

  return std::unique_ptr<Crystal>(new Crystal(vertexes, faces, CrystalType::PYRAMID));
}


CrystalPtrU Crystal::CreateHexPyramidStackHalf(int upper_idx1, int upper_idx4,  // upper Miller index
                                               int lower_idx1, int lower_idx4,  // lower Miller index
                                               float h1, float h2, float h3) {  // heights
  using Math::kPi;
  using Math::TriangleIdx;
  using Math::Vec3f;

  float H1 = kC * upper_idx1 / upper_idx4;
  float H2 = kC * lower_idx1 / lower_idx4;
  h1 = std::max(std::min(h1, 1.0f), 0.0f);
  h2 = std::max(std::min(h2, 1.0f), 0.0f);

  std::vector<Vec3f> vertexes;
  std::vector<TriangleIdx> faces;
  vertexes.reserve(24);
  faces.reserve(44);

  float r = (1.0f - h2) * (1.0f - h1);
  for (int i = 0; i < 6; i++) {
    vertexes.emplace_back(cos(2 * kPi * i / 6) * r,                   // x
                          sin(2 * kPi * i / 6) * r,                   // y
                          h1 * H1 * (1.0f - h2) + h2 * H2 + h3 * 2);  // z
  }
  r = 1.0f - h2;
  for (int i = 0; i < 6; i++) {
    vertexes.emplace_back(cos(2 * kPi * i / 6) * r,  // x
                          sin(2 * kPi * i / 6) * r,  // y
                          h2 * H2 + h3 * 2);         // z
  }
  for (int i = 0; i < 6; i++) {
    vertexes.emplace_back(cos(2 * kPi * i / 6),  // x
                          sin(2 * kPi * i / 6),  // y
                          h3 * 2);               // z
  }
  for (int i = 0; i < 6; i++) {
    vertexes.emplace_back(cos(2 * kPi * i / 6),  // x
                          sin(2 * kPi * i / 6),  // y
                          0);                    // z
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

  return std::unique_ptr<Crystal>(new Crystal(vertexes, faces, CrystalType::STACK_PYRAMID));
}


CrystalPtrU Crystal::CreateIrregularHexPrism(float* dist, float h) {
  /* Use a naive algorithm to determine the profile of basal face
   * 1. For each line pair L1 and L2, get their intersection point p12;
   * 2. For all half planes, check if p12 is in the plane;
   *  2.1 If p12 is in all half planes, put it into a set P;
   *  2.2 Else drop this point;
   * 3. Construct a convex hull from point set P.
   */
  using Math::HalfSpaceSet;
  using Math::kSqrt3;
  using Math::TriangleIdx;
  using Math::Vec3f;

  constexpr int kConstraintNum = 8;

  for (int i = 0; i < 6; i++) {
    dist[i] *= kSqrt3 / 2;
  }

  /* Half plane is expressed as: a*x + b*y + c*z + d <= 0 */
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
    -dist[0], -2 * dist[1], -2 * dist[2], -dist[3], -2 * dist[4], -2 * dist[5],  // prism
    -h,       -h,                                                                // top and bottom
  };
  HalfSpaceSet hss(kConstraintNum, a, b, c, d);

  std::vector<Vec3f> pts = FindInnerPoints(hss);
  SortAndRemoveDuplicate(&pts);

  std::vector<TriangleIdx> faces;
  BuildPolyhedronFaces(hss, pts, faces);

  return std::unique_ptr<Crystal>(new Crystal(pts, faces, CrystalType::PRISM));
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
CrystalPtrU Crystal::CreateIrregularHexPyramid(float* dist, int* idx, float* h) {
  /* There are 20 faces. The crystal is the intersection of all these half-spaces.
   * 1. Find all inner point as vertexes.
   * 2. Find all co-planner points.
   * 3. For points in each face, construct a triangular division.
   */
  using Math::HalfSpaceSet;
  using Math::kSqrt3;
  using Math::TriangleIdx;
  using Math::Vec3f;

  constexpr int kConstraintNum = 20;
  constexpr int kDistNum = 6;
  constexpr int kHNum = 3;

  float alpha0 = idx[1] / kC / idx[0] * kSqrt3;
  float alpha1 = idx[3] / kC / idx[2] * kSqrt3;
  float beta0 = alpha0 * h[1];
  float beta1 = alpha1 * h[1];

  for (int i = 0; i < kDistNum; i++) {
    dist[i] *= kSqrt3 / 2;
  }
  for (int i = 0; i < kHNum; i++) {
    h[i] = std::max(h[i], 0.0f);
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
    -dist[0],
    -2 * dist[1],
    -2 * dist[2],
    -dist[3],
    -2 * dist[4],
    -2 * dist[5],
    -2 * dist[0] - beta0,
    -2 * dist[1] - beta0,
    -2 * dist[2] - beta0,
    -2 * dist[3] - beta0,
    -2 * dist[4] - beta0,
    -2 * dist[5] - beta0,
    -2 * dist[0] - beta1,
    -2 * dist[1] - beta1,
    -2 * dist[2] - beta1,
    -2 * dist[3] - beta1,
    -2 * dist[4] - beta1,
    -2 * dist[5] - beta1,
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
  d[kConstraintNum - 2] = -(maxZ - h[1]) * h[0] - h[1];
  d[kConstraintNum - 1] = (minZ + h[1]) * h[2] - h[1];

  hss.n = kConstraintNum;
  pts = FindInnerPoints(hss);
  SortAndRemoveDuplicate(&pts);

  /* Step 2. Build convex hull with verteces */
  std::vector<TriangleIdx> faces;
  BuildPolyhedronFaces(hss, pts, faces);

  return std::unique_ptr<Crystal>(new Crystal(pts, faces, CrystalType::PYRAMID));
}


CrystalPtrU Crystal::CreateCustomCrystal(const std::vector<IceHalo::Math::Vec3f>& pts,            // vertex points
                                         const std::vector<IceHalo::Math::TriangleIdx>& faces) {  // face indices
  return std::unique_ptr<Crystal>(new Crystal(pts, faces, CrystalType::CUSTOM));
}


CrystalPtrU Crystal::CreateCustomCrystal(const std::vector<IceHalo::Math::Vec3f>& pts,          // vertex points
                                         const std::vector<IceHalo::Math::TriangleIdx>& faces,  // face indices
                                         const std::vector<int>& face_number_map) {             // face to face number
  return std::unique_ptr<Crystal>(new Crystal(pts, faces, face_number_map, CrystalType::CUSTOM));
}

}  // namespace IceHalo
