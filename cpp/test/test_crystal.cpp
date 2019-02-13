#include "crystal.h"
#include "mymath.h"

#include "gtest/gtest.h"

#include <vector>

namespace {

class CrystalTest : public ::testing::Test {
protected:
  // You can do set-up work for each test here.
  CrystalTest() = default;

  // You can do clean-up work that doesn't throw exceptions here.
  virtual ~CrystalTest() = default;

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
      ASSERT_EQ(ids1[i], ids2[i]);
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
    {  k,    -0.5f,  h / 2 },
    {  k,     0.5f,  h / 2 },
    {  0.0f,  1.0f,  h / 2 },
    { -k,     0.5f,  h / 2 },
    { -k,    -0.5f,  h / 2 },
    {  0.0f, -1.0f,  h / 2 },
    {  k,    -0.5f, -h / 2 },
    {  k,     0.5f, -h / 2 },
    {  0.0f,  1.0f, -h / 2 },
    { -k,     0.5f, -h / 2 },
    { -k,    -0.5f, -h / 2 },
    {  0.0f, -1.0f, -h / 2 },
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
    {     k * r1, -0.5f * r1,  h2 / 2 + h1 * H },
    {     k * r1,  0.5f * r1,  h2 / 2 + h1 * H },
    {  0.0f * r1,  1.0f * r1,  h2 / 2 + h1 * H },
    {    -k * r1,  0.5f * r1,  h2 / 2 + h1 * H },
    {    -k * r1, -0.5f * r1,  h2 / 2 + h1 * H },
    {  0.0f * r1, -1.0f * r1,  h2 / 2 + h1 * H },
    {     k, -0.5f,  h2 / 2 },
    {     k,  0.5f,  h2 / 2 },
    {  0.0f,  1.0f,  h2 / 2 },
    {    -k,  0.5f,  h2 / 2 },
    {    -k, -0.5f,  h2 / 2 },
    {  0.0f, -1.0f,  h2 / 2 },
    {     k, -0.5f, -h2 / 2 },
    {     k,  0.5f, -h2 / 2 },
    {  0.0f,  1.0f, -h2 / 2 },
    {    -k,  0.5f, -h2 / 2 },
    {    -k, -0.5f, -h2 / 2 },
    {  0.0f, -1.0f, -h2 / 2 },
    {     k * r3, -0.5f * r3, -h2 / 2 - h3 * H },
    {     k * r3,  0.5f * r3, -h2 / 2 - h3 * H },
    {  0.0f * r3,  1.0f * r3, -h2 / 2 - h3 * H },
    {    -k * r3,  0.5f * r3, -h2 / 2 - h3 * H },
    {    -k * r3, -0.5f * r3, -h2 / 2 - h3 * H },
    {  0.0f * r3, -1.0f * r3, -h2 / 2 - h3 * H },
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


TEST_F(CrystalTest, IrregularHexCylinderVertex1) {
  using IceHalo::Math::kSqrt3;

  float h = 1.2f;
  float dist[6] = { 1.0f, 1.0f, 1.5f, 1.0f, 2.5f, 1.0f };

  std::vector<IceHalo::Math::Vec3f> pts0;
  pts0.emplace_back(-1.25f, kSqrt3 / 2/2, -h);
  pts0.emplace_back(-1.25f, kSqrt3 / 2/2, h);
  pts0.emplace_back(-1.0f, kSqrt3 / 2, -h);
  pts0.emplace_back(-1.0f, kSqrt3 / 2, h);
  pts0.emplace_back(0.0f, -kSqrt3 / 2*2, -h);
  pts0.emplace_back(0.0f, -kSqrt3 / 2*2, h);
  pts0.emplace_back(0.5f, kSqrt3 / 2, -h);
  pts0.emplace_back(0.5f, kSqrt3 / 2, h);
  pts0.emplace_back(1.0f, 0.0f, -h);
  pts0.emplace_back(1.0f, 0.0f, h);

  auto c = IceHalo::Crystal::createIrregularHexCylinder(dist, h);
  const auto pts = c->getVertexes();

  EXPECT_EQ(pts.size(), 10ul);
  checkVertex(pts0, pts);
}

}  // namespace
