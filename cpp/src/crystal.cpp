#include "crystal.h"

#include <algorithm>
#include <cstring>
#include <cassert>

namespace IceHalo {

Crystal::Crystal(const std::vector<Math::Vec3f>& vertexes, const std::vector<Math::TriangleIdx>& faces,
                 CrystalType type)
    : vertexes(vertexes), faces(faces), type(type) {
  initNorms();
  initFaceNumbers();
}

Crystal::Crystal(const std::vector<IceHalo::Math::Vec3f>& vertexes,
                 const std::vector<IceHalo::Math::TriangleIdx>& faces,
                 const std::vector<int>& faceId,
                 CrystalType type)
    : vertexes(vertexes), faces(faces), faceIdMap(faceId), type(type) {
  initNorms();
}

const std::vector<Math::Vec3f>& Crystal::getVertexes() {
  return vertexes;
}

const std::vector<Math::Vec3f>& Crystal::getNorms() {
  return norms;
}

const std::vector<Math::TriangleIdx>& Crystal::getFaces() {
  return faces;
}

const std::vector<int>& Crystal::getFaceNumber() {
  return faceIdMap;
}

int Crystal::vtxNum() const {
  return static_cast<int>(vertexes.size());
}

int Crystal::totalFaces() const {
  return static_cast<int>(faces.size());
}

int Crystal::faceNumber(int idx) const {
  if (faceIdMap.empty()) {
    return -1;
  } else if (idx < 0 || idx >= faceIdMap.size()) {
    return -1;
  } else {
    return faceIdMap[idx];
  }
}

void Crystal::copyVertexData(float* data) const {
  for (decltype(vertexes.size()) i = 0; i < vertexes.size(); ++i) {
    std::memcpy(data + i * 3, vertexes[i].val(), 3 * sizeof(float));
  }
}

void Crystal::copyFaceData(float* data) const {
  for (decltype(faces.size()) i = 0; i < faces.size(); i++) {
    auto idx = faces[i].idx();
    std::memcpy(data + i * 9 + 0, vertexes[idx[0]].val(), 3 * sizeof(float));
    std::memcpy(data + i * 9 + 3, vertexes[idx[1]].val(), 3 * sizeof(float));
    std::memcpy(data + i * 9 + 6, vertexes[idx[2]].val(), 3 * sizeof(float));
  }
}

void Crystal::copyFaceIdxData(int* data) const {
  for (decltype(faces.size()) i = 0; i < faces.size(); i++) {
    std::memcpy(data + i * 3, faces[i].idx(), 3 * sizeof(int));
  }
}

void Crystal::copyNormalData(int idx, float* data) const {
  if (idx >= static_cast<int>(faces.size()) || idx < 0) {
    return;
  }
  std::memcpy(data, norms[idx].val(), 3 * sizeof(float)); }

void Crystal::copyNormalData(float* data) const {
  for (decltype(norms.size()) i = 0; i < norms.size(); i++) {
    std::memcpy(data + i * 3, norms[i].val(), 3 * sizeof(float));
  }
}

void Crystal::initNorms() {
  using Math::Vec3f;
  norms.clear();
  for (const auto& f : faces) {
    auto idx = f.idx();
    Vec3f v1 = Vec3f::fromVec(vertexes[idx[0]], vertexes[idx[1]]);
    Vec3f v2 = Vec3f::fromVec(vertexes[idx[0]], vertexes[idx[2]]);
    norms.push_back(Vec3f::cross(v1, v2));
  }
  for (auto& v : norms) {
    v.normalize();
  }
}

void Crystal::initFaceNumbers() {
  switch (type) {
    case CrystalType::PRISM:
    case CrystalType::PYRAMID:
      initFaceNumbersHex();
      break;
    case CrystalType::CUBIC_PYRAMID:
      initFaceNumberCubic();
      break;
    case CrystalType::STACK_PYRAMID:
      initFaceNumberStack();
      break;
    case CrystalType::CUSTOM:
    case CrystalType::UNKNOWN:
      break;
  }
}

void Crystal::initFaceNumbersHex() {
  assert(faces.size() == norms.size());
  for (decltype(faces.size()) i = 0; i < faces.size(); i++) {
    float maxVal = -1;
    int maxFaceNumber = -1;
    for (const auto& d : hexFaceNormToNumberList) {
      float tmpVal = Math::Vec3f::dot(norms[i], d.first);
      if (tmpVal > maxVal) {
        maxVal = tmpVal;
        maxFaceNumber = d.second;
      }
    }

    if (maxVal > 0) {
      if (std::abs(maxVal - 1) > Math::kFloatEps && norms[i].z() > Math::kFloatEps) {
        maxFaceNumber += 10;
      } else if (std::abs(maxVal - 1) > Math::kFloatEps && norms[i].z() < -Math::kFloatEps) {
        maxFaceNumber += 20;
      }
    }
    faceIdMap.push_back(maxFaceNumber);
  }
}

void Crystal::initFaceNumberCubic() {
  assert(faces.size() == norms.size());
  for (decltype(faces.size()) i = 0; i < faces.size(); i++) {
    float maxVal = -1;
    int maxFaceNumber = -1;
    for (const auto& d : cubicFaceNormToNumberList) {
      float tmpVal = Math::Vec3f::dot(norms[i], d.first);
      if (tmpVal > maxVal) {
        maxVal = tmpVal;
        maxFaceNumber = d.second;
      }
    }

    if (maxVal > 0) {
      if (std::abs(maxVal - 1) > Math::kFloatEps && norms[i].z() > Math::kFloatEps) {
        maxFaceNumber += 10;
      } else if (std::abs(maxVal - 1) > Math::kFloatEps && norms[i].z() < -Math::kFloatEps) {
        maxFaceNumber += 20;
      }
    }
    faceIdMap.push_back(maxFaceNumber);
  }
}

void Crystal::initFaceNumberStack() {
  assert(faces.size() == norms.size());
  float maxHeight = std::numeric_limits<float>::lowest();
  float minHeight = std::numeric_limits<float>::max();
  for (decltype(faces.size()) i = 0; i < faces.size(); i++) {
    float maxVal = -1;
    int maxFaceNumber = -1;
    for (const auto& d : hexFaceNormToNumberList) {
      float tmpVal = Math::Vec3f::dot(norms[i], d.first);
      if (tmpVal > maxVal) {
        maxVal = tmpVal;
        maxFaceNumber = d.second;
      }
    }
    faceIdMap.push_back(maxFaceNumber);

    const auto* idx = faces[i].idx();
    for (int j = 0; j < 3; j++) {
      float h = vertexes[idx[j]].z();
      if (h > maxHeight) {
        maxHeight = h;
      }
      if (h < minHeight) {
        minHeight = h;
      }
    }
  }

  for (decltype(faces.size()) i = 0; i < faces.size(); i++) {
    if (faceIdMap[i] < 3) {
      continue;
    }
    const auto* idx = faces[i].idx();

    bool isTop = false;
    bool isBottom = false;
    bool isUpper = true;
    bool isLower = true;
    for (int j = 0; j < 3; j++) {
      isTop = isTop || std::abs(vertexes[idx[j]].z() - maxHeight) < Math::kFloatEps;
      isBottom = isBottom || std::abs(vertexes[idx[j]].z() - minHeight) < Math::kFloatEps;
      isUpper = isUpper && vertexes[idx[j]].z() > Math::kFloatEps;
      isLower = isLower && vertexes[idx[j]].z() < -Math::kFloatEps;
    }

    bool isPrism = std::abs(norms[i].z()) < Math::kFloatEps;

    if (isTop && !isPrism) {
      faceIdMap[i] += 10;
    } else if (isBottom && !isPrism) {
      faceIdMap[i] += 20;
    } else if (isUpper && !isPrism) {
      faceIdMap[i] += 30;
    } else if (isLower && !isPrism) {
      faceIdMap[i] += 40;
    }
  }
}


const std::vector<std::pair<Math::Vec3f, int> > Crystal::hexFaceNormToNumberList {
  { {                 0,     0,  1 }, 1 },
  { {                 0,     0, -1 }, 2 },
  { {                 1,     0,  0 }, 3 },
  { {  Math::kSqrt3 / 2,  0.5f,  0 }, 4 },
  { { -Math::kSqrt3 / 2,  0.5f,  0 }, 5 },
  { {                -1,     0,  0 }, 6 },
  { { -Math::kSqrt3 / 2, -0.5f,  0 }, 7 },
  { {  Math::kSqrt3 / 2, -0.5f,  0 }, 8 },
};


const std::vector<std::pair<Math::Vec3f, int> > Crystal::cubicFaceNormToNumberList {
  { {  0,  0,  1 }, 1 },
  { {  0,  0, -1 }, 2 },
  { {  1,  0,  0 }, 3 },
  { {  0,  1,  0 }, 4 },
  { { -1,  0,  0 }, 5 },
  { {  0, -1,  0 }, 6 },
};


/*
 * parameter: h, defined as height / diameter
 */
CrystalPtr Crystal::createHexCylinder(float h) {
  using Math::Vec3f;
  using Math::TriangleIdx;
  using Math::kPi;

  std::vector<Vec3f> vertexes;
  std::vector<TriangleIdx> faces;
  vertexes.reserve(12);
  faces.reserve(20);

  for (int i = 0; i < 6; ++i) {
    vertexes.emplace_back(cos((2 * i - 1) * kPi / 6), sin((2 * i - 1) * kPi / 6), h);
  }
  for (int i = 0; i < 6; ++i) {
    vertexes.emplace_back(cos((2 * i - 1) * kPi / 6), sin((2 * i - 1) * kPi / 6), -h);
  }

  faces.emplace_back(0, 1, 2);
  faces.emplace_back(0, 2, 3);
  faces.emplace_back(3, 4, 5);
  faces.emplace_back(3, 5, 0);
  for (int i = 0; i < 6; ++i) {
    faces.emplace_back(i, i+6, (i+1)%6);
    faces.emplace_back(i+6, (i+1)%6+6, (i+1)%6);
  }
  faces.emplace_back(6, 8, 7);
  faces.emplace_back(6, 9, 8);
  faces.emplace_back(9, 11, 10);
  faces.emplace_back(9, 6, 11);

  return std::shared_ptr<Crystal>(new Crystal(vertexes, faces, CrystalType::PRISM));
}

/*
 * parameter: h1, defined as h / H, where h is the actual height of first pyramid, H is the
 *      full height of first pyramid. It will be clamped to between 0.0 and 1.0.
 * parameter: h2, defined as h / a, where h is the actual height of middle cylinder, a is the
 *      diameter of base plate.
 * parameter: h3, defined as h / H, similar to h3.
 */
CrystalPtr Crystal::createHexPyramid(float h1, float h2, float h3) {
  return createHexPyramid(1, 1, 1, 1, h1, h2, h3);
}

/*
  top vertex:
    1     0
    2     3
  middle vertex:
    5     4
    6     7
  bottom vertex:
    9     8
    10    11
*/
CrystalPtr Crystal::createCubicPyramid(float ratio1, float ratio2) {
  using Math::Vec3f;
  using Math::TriangleIdx;
  using Math::kPi;

  ratio1 = std::min(ratio1, 1.f);
  ratio2 = std::min(ratio2, 1.f);

  std::vector<Vec3f> vertexes;
  std::vector<TriangleIdx> faces;
  vertexes.reserve(12);

  for (int i = 0; i < 4; i++) {
    vertexes.emplace_back(
      cos(kPi / 4 + kPi / 2 * i) * (1 - ratio1),
      sin(kPi / 4 + kPi / 2 * i) * (1 - ratio1),
      ratio1);
  }
  for (int i = 0; i < 4; i++) {
    vertexes.emplace_back(
      cos(kPi / 4 + kPi / 2 * i),
      sin(kPi / 4 + kPi / 2 * i),
      0);
  }
  for (int i = 0; i < 4; i++) {
    vertexes.emplace_back(
      cos(kPi / 4 + kPi / 2 * i) * (1 - ratio2),
      sin(kPi / 4 + kPi / 2 * i) * (1 - ratio2),
      -ratio2);
  }

  faces.emplace_back(0, 1, 2);
  faces.emplace_back(0, 2, 3);
  for (int i = 0; i < 4; ++i) {
    faces.emplace_back(i, i+4, (i+1)%4);
    faces.emplace_back(i+4, (i+1)%4+4, (i+1)%4);
  }
  for (int i = 4; i < 8; ++i) {
    faces.emplace_back(i, i+4, (i+1)%4+4);
    faces.emplace_back(i+4, (i+1)%4+8, (i+1)%4+4);
  }
  faces.emplace_back(9, 8, 10);
  faces.emplace_back(10, 8, 11);

  return std::shared_ptr<Crystal>(new Crystal(vertexes, faces, CrystalType::CUBIC_PYRAMID));
}

/*
 * parameter: i1, i4, the miller index to describe face 13. Index form is (i1, 0, -i1, i4)
 * parameter: h1, defined as h / H
 * parameter: h2, defined as h / a
 * parameter: h3, similar to h1
 */
CrystalPtr Crystal::createHexPyramid(int i1, int i4, float h1, float h2, float h3) {
  return createHexPyramid(i1, i4, i1, i4, h1, h2, h3);
}


CrystalPtr Crystal::createHexPyramid(int upperIdx1, int upperIdx4, int lowerIdx1, int lowerIdx4,
                                     float h1, float h2, float h3) {
  using Math::Vec3f;
  using Math::TriangleIdx;
  using Math::kPi;

  float H1 = kC * upperIdx1 / upperIdx4;
  float H3 = kC * lowerIdx1 / lowerIdx4;
  h1 = std::max(std::min(h1, 1.0f), 0.0f);
  h3 = std::max(std::min(h3, 1.0f), 0.0f);

  std::vector<Vec3f> vertexes;
  std::vector<TriangleIdx> faces;
  vertexes.reserve(24);
  faces.reserve(44);

  for (int i = 0; i < 6; i++) {
    vertexes.emplace_back(
      cos((2 * i - 1) * kPi / 6) * (1 - h1),
      sin((2 * i - 1) * kPi / 6) * (1 - h1),
      h2 + h1 * H1);
  }
  for (int i = 0; i < 6; i++) {
    vertexes.emplace_back(
      cos((2 * i - 1) * kPi / 6),
      sin((2 * i - 1) * kPi / 6),
      h2);
  }
  for (int i = 0; i < 6; i++) {
    vertexes.emplace_back(
      cos((2 * i - 1) * kPi / 6),
      sin((2 * i - 1) * kPi / 6),
      -h2);
  }
  for (int i = 0; i < 6; i++) {
    vertexes.emplace_back(
      cos((2 * i - 1) * kPi / 6) * (1 - h3),
      sin((2 * i - 1) * kPi / 6) * (1 - h3),
      -h2 - h3 * H3);
  }

  faces.emplace_back(0, 1, 2);
  faces.emplace_back(0, 2, 3);
  faces.emplace_back(3, 4, 5);
  faces.emplace_back(3, 5, 0);
  for (int i = 0; i < 6; ++i) {
    faces.emplace_back(i, i+6, (i+1)%6);
    faces.emplace_back(i+6, (i+1)%6+6, (i+1)%6);
  }
  for (int i = 6; i < 12; ++i) {
    faces.emplace_back(i, i+6, (i+1)%6+6);
    faces.emplace_back(i+6, (i+1)%6+12, (i+1)%6+6);
  }
  for (int i = 12; i < 18; ++i) {
    faces.emplace_back(i, i+6, (i+1)%6+12);
    faces.emplace_back(i+6, (i+1)%6+18, (i+1)%6+12);
  }
  faces.emplace_back(18, 20, 19);
  faces.emplace_back(18, 21, 20);
  faces.emplace_back(21, 23, 22);
  faces.emplace_back(21, 18, 23);

  return std::shared_ptr<Crystal>(new Crystal(vertexes, faces, CrystalType::PYRAMID));
}

CrystalPtr Crystal::createHexPyramidStackHalf(int upperIdx1, int upperIdx4, int lowerIdx1, int lowerIdx4,
                                              float h1, float h2, float h3) {
  using Math::Vec3f;
  using Math::TriangleIdx;
  using Math::kPi;

  float H1 = kC * upperIdx1 / upperIdx4;
  float H2 = kC * lowerIdx1 / lowerIdx4;
  h1 = std::max(std::min(h1, 1.0f), 0.0f);
  h2 = std::max(std::min(h2, 1.0f), 0.0f);

  std::vector<Vec3f> vertexes;
  std::vector<TriangleIdx> faces;
  vertexes.reserve(24);
  faces.reserve(44);

  float r = (1.0f - h2) * (1.0f - h1);
  for (int i = 0; i < 6; i++) {
    vertexes.emplace_back(
      cos(2 * kPi * i / 6) * r,
      sin(2 * kPi * i / 6) * r,
      h1 * H1 * (1.0f - h2) + h2 * H2 + h3 * 2);
  }
  r = 1.0f - h2;
  for (int i = 0; i < 6; i++) {
    vertexes.emplace_back(
      cos(2 * kPi * i / 6) * r,
      sin(2 * kPi * i / 6) * r,
      h2 * H2 + h3 * 2);
  }
  for (int i = 0; i < 6; i++) {
    vertexes.emplace_back(
      cos(2 * kPi * i / 6),
      sin(2 * kPi * i / 6),
      h3 * 2);
  }
  for (int i = 0; i < 6; i++) {
    vertexes.emplace_back(
      cos(2 * kPi * i / 6),
      sin(2 * kPi * i / 6),
      0);
  }

  faces.emplace_back(0, 1, 2);
  faces.emplace_back(0, 2, 3);
  faces.emplace_back(3, 4, 5);
  faces.emplace_back(3, 5, 0);
  for (int i = 0; i < 6; ++i) {
    faces.emplace_back(i, i+6, (i+1)%6);
    faces.emplace_back(i+6, (i+1)%6+6, (i+1)%6);
  }
  for (int i = 6; i < 12; ++i) {
    faces.emplace_back(i, i+6, (i+1)%6+6);
    faces.emplace_back(i+6, (i+1)%6+12, (i+1)%6+6);
  }
  for (int i = 12; i < 18; ++i) {
    faces.emplace_back(i, i+6, (i+1)%6+12);
    faces.emplace_back(i+6, (i+1)%6+18, (i+1)%6+12);
  }
  faces.emplace_back(18, 20, 19);
  faces.emplace_back(18, 21, 20);
  faces.emplace_back(21, 23, 22);
  faces.emplace_back(21, 18, 23);

  return std::shared_ptr<Crystal>(new Crystal(vertexes, faces, CrystalType::STACK_PYRAMID));
}


/*
 * parameter: dist, defines the distance from origin of each face. Must contains 6 numbers.
 *      Starts from face 3.
 * parameter: h, cylinder height, h = height / a
 */
CrystalPtr Crystal::createIrregularHexCylinder(float* dist, float h) {
  /* Use a naive algorithm to determine the profile of basal face
   * 1. For each line pair L1 and L2, get their intersection point p12;
   * 2. For all half planes, check if p12 is in the plane;
   *  2.1 If p12 is in all half planes, put it into a set P;
   *  2.2 Else drop this point;
   * 3. Construct a convex hull from point set P.
   */
  using Math::kSqrt3;
  using Math::Vec3f;
  using Math::TriangleIdx;
  using Math::HalfSpaceSet;

  constexpr int kConstraintNum = 8;

  for (int i = 0; i < 6; i++) {
    dist[i] *= kSqrt3 / 2;
  }

  /* Half plane is expressed as: a*x + b*y + c*z + d <= 0 */
  float a[kConstraintNum] = {
    1.0f, 1.0f, -1.0f, -1.0f, -1.0f, 1.0f,   // Prism faces
    0.0f, 0.0f,                              // Top and bottom faces
  };
  float b[kConstraintNum] = {
    0.0f, kSqrt3, kSqrt3, 0.0f, -kSqrt3, -kSqrt3,
    0.0f, 0.0f,
  };
  float c[kConstraintNum] = {
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    1.0f, -1.0f,
  };
  float d[kConstraintNum] = {
    -dist[0], -2 * dist[1], -2 * dist[2], -dist[3], -2 * dist[4], -2 * dist[5],
    -h, -h,
  };
  HalfSpaceSet hss(kConstraintNum, a, b, c, d);

  std::vector<Vec3f> pts = findInnerPoints(hss);
  sortAndRemoveDuplicate(&pts);

  std::vector<TriangleIdx> faces;
  buildPolyhedronFaces(hss, pts, faces);

  return std::shared_ptr<Crystal>(new Crystal(pts, faces, CrystalType::PRISM));
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
CrystalPtr Crystal::createIrregularHexPyramid(float* dist, int* idx, float* h) {
  /* There are 20 faces. The crystal is the intersection of all these half-spaces.
   * 1. Find all inner point as vertexes.
   * 2. Find all co-planner points.
   * 3. For points in each face, construct a triangular division.
   */
  using Math::kSqrt3;
  using Math::Vec3f;
  using Math::TriangleIdx;
  using Math::HalfSpaceSet;

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
    1.0f, 1.0f, -1.0f, -1.0f, -1.0f, 1.0f,   // Prism faces
    2.0f, 1.0f, -1.0f, -2.0f, -1.0f, 1.0f,   // Upper pyramid faces
    2.0f, 1.0f, -1.0f, -2.0f, -1.0f, 1.0f,   // Lower pyramid faces
    0.0f, 0.0f,                              // Top and bottom faces
  };
  float b[kConstraintNum] = {
    0.0f, kSqrt3, kSqrt3, 0.0f, -kSqrt3, -kSqrt3,
    0.0f, kSqrt3, kSqrt3, 0.0f, -kSqrt3, -kSqrt3,
    0.0f, kSqrt3, kSqrt3, 0.0f, -kSqrt3, -kSqrt3,
    0.0f, 0.0f,
  };
  float c[kConstraintNum] = {
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    alpha0, alpha0, alpha0, alpha0, alpha0, alpha0,
    -alpha1, -alpha1, -alpha1, -alpha1, -alpha1, -alpha1,
    1.0f, -1.0f,
  };
  float d[kConstraintNum] = {
    -dist[0], -2 * dist[1], -2 * dist[2], -dist[3], -2 * dist[4], -2 * dist[5],
    -2 * dist[0] - beta0, -2 * dist[1] - beta0, -2 * dist[2] - beta0, -2 * dist[3] - beta0, -2 * dist[4] - beta0, -2 * dist[5] - beta0,
    -2 * dist[0] - beta1, -2 * dist[1] - beta1, -2 * dist[2] - beta1, -2 * dist[3] - beta1, -2 * dist[4] - beta1, -2 * dist[5] - beta1,
    0.0f, 0.0f,
  };
  HalfSpaceSet hss(kConstraintNum - 2, a, b, c, d);

  /* Step 1. Find all inner points */
  std::vector<Vec3f> pts = findInnerPoints(hss);
  sortAndRemoveDuplicate(&pts);

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
  pts = findInnerPoints(hss);
  sortAndRemoveDuplicate(&pts);

  /* Step 2. Build convex hull with verteces */
  std::vector<TriangleIdx> faces;
  buildPolyhedronFaces(hss, pts, faces);

  return std::shared_ptr<Crystal>(new Crystal(pts, faces, CrystalType::PYRAMID));
}


CrystalPtr Crystal::createCustomCrystal(const std::vector<IceHalo::Math::Vec3f>& pts,
                                        const std::vector<IceHalo::Math::TriangleIdx>& faces) {
  return std::shared_ptr<Crystal>(new Crystal(pts, faces, CrystalType::CUSTOM));
}


CrystalPtr Crystal::createCustomCrystal(const std::vector<IceHalo::Math::Vec3f>& pts,
                                        const std::vector<IceHalo::Math::TriangleIdx>& faces,
                                        const std::vector<int>& faceIdMap) {
  return std::shared_ptr<Crystal>(new Crystal(pts, faces, faceIdMap, CrystalType::CUSTOM));
}

};  // namespace IceHalo
