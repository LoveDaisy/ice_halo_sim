#include "crystal.h"
#include "mymath.h"

#include "gtest/gtest.h"

#include <vector>
#include <algorithm>

namespace {

class CrystalTest : public ::testing::Test {
protected:
  void checkVertex(const std::vector<IceHalo::Math::Vec3f>& vtx1, const std::vector<IceHalo::Math::Vec3f>& vtx2) {
    ASSERT_EQ(vtx1.size(), vtx2.size());
    for (decltype(vtx1.size()) i = 0; i < vtx1.size(); i++) {
      const auto& p0 = vtx1[i];
      const auto& p = vtx2[i];

      EXPECT_TRUE(p0 == p);
    }
  }

  void checkFaceId(const std::vector<int>& ids1, const std::vector<int>& ids2) {
    ASSERT_EQ(ids1.size(), ids2.size());
    for (decltype(ids1.size()) i = 0; i < ids1.size(); i++) {
      EXPECT_EQ(ids1[i], ids2[i]);
    }
  }

  void checkCrystal(const IceHalo::CrystalPtr & c1, const IceHalo::CrystalPtr& c2) {
    auto n1 = c1->getNorms();
    auto n2 = c2->getNorms();
    auto fn1 = c1->getFaceNumber();
    auto fn2 = c2->getFaceNumber();

    ASSERT_EQ(n1.size(), n2.size());
    ASSERT_EQ(fn1.size(), fn2.size());
    for (decltype(n1.size()) i = 0; i < n1.size(); i++) {
      const auto& n = n1[i];
      int fn = fn1[i];

      bool findSameNorm = false;
      for (decltype(n2.size()) j = 0; j < n2.size(); j++) {
        const auto& tmpN = n2[j];
        int tmpFn = fn2[j];
        if (n == tmpN) {
          findSameNorm = true;
          EXPECT_EQ(tmpFn, fn);
        }
      }
      EXPECT_TRUE(findSameNorm);
    }

    for (decltype(n2.size()) i = 0; i < n2.size(); i++) {
      const auto& n = n2[i];
      int fn = fn2[i];

      bool findSameNorm = false;
      for (decltype(n1.size()) j = 0; j < n1.size(); j++) {
        const auto& tmpN = n1[j];
        int tmpFn = fn1[j];
        if (n == tmpN) {
          findSameNorm = true;
          EXPECT_EQ(tmpFn, fn);
        }
      }
      EXPECT_TRUE(findSameNorm);
    }

    auto v1 = c1->getVertexes();
    auto v2 = c2->getVertexes();
    for (const auto& v : v1) {
      auto findIter = std::find(v2.begin(), v2.end(), v);
      EXPECT_NE(findIter, v2.end());
    }
    for (const auto& v : v2) {
      auto findIter = std::find(v1.begin(), v1.end(), v);
      EXPECT_NE(findIter, v1.end());
    }
  }

};


TEST_F(CrystalTest, CreateNotNull) {
  /* Cylinder */
  auto c = IceHalo::Crystal::createHexCylinder(1.2f);
  EXPECT_NE(c, nullptr);

  /* Pyramid */
  c = IceHalo::Crystal::createHexPyramid(1.2f, 1.2f, 1.2f);
  EXPECT_NE(c, nullptr);

  c = IceHalo::Crystal::createHexPyramid(1, 1, 1.2f, 1.2f, 1.2f);
  EXPECT_NE(c, nullptr);

  c = IceHalo::Crystal::createHexPyramid(1, 1, 2, 3, 1.2f, 1.2f, 1.2f);
  EXPECT_NE(c, nullptr);

  /* IrregularHexCylinder */
  float dist[6] = { 1.0f, 1.0f, 1.5f, 1.0f, 2.5f, 1.0f };
  c = IceHalo::Crystal::createIrregularHexCylinder(dist, 1.2f);
  EXPECT_NE(c, nullptr);

  /* IrregularHexCylinder */
  int idx[4] = { 1, 1, 1, 1 };
  float h[3] = { 0.3f, 1.2f, 0.9f };
  c = IceHalo::Crystal::createIrregularHexPyramid(dist, idx, h);
  EXPECT_NE(c, nullptr);
}


TEST_F(CrystalTest, HexCylinderVertex) {
  using IceHalo::Math::kSqrt3;

  float h = 1.2f;
  float k = kSqrt3 / 2;

  std::vector<IceHalo::Math::Vec3f> pts0 = {
    {  k,    -0.5f,  h },
    {  k,     0.5f,  h },
    {  0.0f,  1.0f,  h },
    { -k,     0.5f,  h },
    { -k,    -0.5f,  h },
    {  0.0f, -1.0f,  h },
    {  k,    -0.5f, -h },
    {  k,     0.5f, -h },
    {  0.0f,  1.0f, -h },
    { -k,     0.5f, -h },
    { -k,    -0.5f, -h },
    {  0.0f, -1.0f, -h },
  };
  std::vector<int> faceId0 = {
    1, 1, 1, 1, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 8, 8, 2, 2, 2, 2,
  };

  auto c = IceHalo::Crystal::createHexCylinder(h);
  EXPECT_EQ(c->getVertexes().size(), 12ul);
  checkVertex(pts0, c->getVertexes());
  checkFaceId(faceId0, c->getFaceNumber());
}


TEST_F(CrystalTest, HexPyramidVertex0) {
  using IceHalo::Math::kSqrt3;

  float h1 = 0.3f;
  float h2 = 1.0f;
  float h3 = 0.9f;
  float H = IceHalo::Crystal::kC;

  float r1 = 1.0f - h1;
  float r3 = 1.0f - h3;
  float k = kSqrt3 / 2;

  std::vector<IceHalo::Math::Vec3f> pts0 = {
    {     k * r1, -0.5f * r1,  h2 + h1 * H },
    {     k * r1,  0.5f * r1,  h2 + h1 * H },
    {  0.0f * r1,  1.0f * r1,  h2 + h1 * H },
    {    -k * r1,  0.5f * r1,  h2 + h1 * H },
    {    -k * r1, -0.5f * r1,  h2 + h1 * H },
    {  0.0f * r1, -1.0f * r1,  h2 + h1 * H },
    {     k, -0.5f,  h2 },
    {     k,  0.5f,  h2 },
    {  0.0f,  1.0f,  h2 },
    {    -k,  0.5f,  h2 },
    {    -k, -0.5f,  h2 },
    {  0.0f, -1.0f,  h2 },
    {     k, -0.5f, -h2 },
    {     k,  0.5f, -h2 },
    {  0.0f,  1.0f, -h2 },
    {    -k,  0.5f, -h2 },
    {    -k, -0.5f, -h2 },
    {  0.0f, -1.0f, -h2 },
    {     k * r3, -0.5f * r3, -h2 - h3 * H },
    {     k * r3,  0.5f * r3, -h2 - h3 * H },
    {  0.0f * r3,  1.0f * r3, -h2 - h3 * H },
    {    -k * r3,  0.5f * r3, -h2 - h3 * H },
    {    -k * r3, -0.5f * r3, -h2 - h3 * H },
    {  0.0f * r3, -1.0f * r3, -h2 - h3 * H },
  };
  std::vector<int> faceId0 = {
    1, 1, 1, 1,
    13, 13, 14, 14, 15, 15, 16, 16, 17, 17, 18, 18,
    3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 8, 8,
    23, 23, 24, 24, 25, 25, 26, 26, 27, 27, 28, 28,
    2, 2, 2, 2,
  };

  auto c = IceHalo::Crystal::createHexPyramid(h1, h2, h3);
  EXPECT_EQ(c->getVertexes().size(), 24ul);
  checkVertex(pts0, c->getVertexes());
  checkFaceId(faceId0, c->getFaceNumber());

  c = IceHalo::Crystal::createHexPyramid(1, 1, h1, h2, h3);
  EXPECT_EQ(c->getVertexes().size(), 24ul);
  checkVertex(pts0, c->getVertexes());
  checkFaceId(faceId0, c->getFaceNumber());

  c = IceHalo::Crystal::createHexPyramid(1, 1, 1, 1, h1, h2, h3);
  EXPECT_EQ(c->getVertexes().size(), 24ul);
  checkVertex(pts0, c->getVertexes());
  checkFaceId(faceId0, c->getFaceNumber());
}


TEST_F(CrystalTest, IrregularHexPyramid) {
  float h1 = 0.3;
  float h2 = 0.5;
  float h3 = 0.85;

  float dist[6] = { 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f };
  int idx[4] = { 1, 1, 1, 1 };
  float hs[3] = { h1, h2, h3 };

  auto c1 = IceHalo::Crystal::createHexPyramid(h1, h2, h3);
  auto c2 = IceHalo::Crystal::createIrregularHexPyramid(dist, idx, hs);
  EXPECT_EQ(c1->totalFaces(), c2->totalFaces());
  EXPECT_EQ(c1->vtxNum(), c2->vtxNum());
  checkCrystal(c1, c2);
}


TEST_F(CrystalTest, IrregularHexCylinderVertex0) {
  using IceHalo::Math::kSqrt3;

  float h = 1.2f;
  float dist[6] = { 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f };

  auto c1 = IceHalo::Crystal::createHexCylinder(h);
  auto c2 = IceHalo::Crystal::createIrregularHexCylinder(dist, h);

  EXPECT_EQ(c1->totalFaces(), c2->totalFaces());
  EXPECT_EQ(c1->vtxNum(), c2->vtxNum());
  checkCrystal(c1, c2);
}


TEST_F(CrystalTest, IrregularHexCylinderVertex1) {
  using IceHalo::Math::kSqrt3;

  float h = 1.2f;
  float dist[6] = { 1.0f, 1.0f, 1.5f, 1.0f, 2.5f, 1.0f };

  std::vector<IceHalo::Math::Vec3f> pts0 {
    {  kSqrt3 / 2, -0.5f,  h },
    {  kSqrt3 / 2,  0.5f,  h },
    { -kSqrt3 / 4, 1.25f,  h },
    { -kSqrt3 / 2,  1.0f,  h },
    { -kSqrt3 / 2, -1.5f,  h },
    {  kSqrt3 / 2, -0.5f, -h },
    {  kSqrt3 / 2,  0.5f, -h },
    { -kSqrt3 / 4, 1.25f, -h },
    { -kSqrt3 / 2,  1.0f, -h },
    { -kSqrt3 / 2, -1.5f, -h },
  };
  std::vector<IceHalo::Math::TriangleIdx> faces0 {
    { 0, 1, 2 }, { 0, 2, 3 }, { 0, 3, 4 },
    { 0, 6, 1 }, { 0, 5, 6 },
    { 1, 7, 2 }, { 1, 6, 7 },
    { 2, 8, 3 }, { 2, 7, 8 },
    { 3, 9, 4 }, { 3, 8, 9 },
    { 4, 5, 0 }, { 4, 9, 5 },
    { 5, 7, 6 }, { 5, 8, 7 }, { 5, 9, 8 },
  };
  std::vector<int> faceIdMap0 {
    1, 1, 1,
    3, 3,
    4, 4,
    5, 5,
    6, 6,
    8, 8,
    2, 2, 2,
  };

  auto c1 = IceHalo::Crystal::createCustomCrystal(pts0, faces0, faceIdMap0);
  auto c2 = IceHalo::Crystal::createIrregularHexCylinder(dist, h);
  EXPECT_EQ(c1->vtxNum(), c2->vtxNum());
  EXPECT_EQ(c1->totalFaces(), c2->totalFaces());
  checkCrystal(c1, c2);
}

}  // namespace
