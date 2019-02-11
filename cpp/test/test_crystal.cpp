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

  std::vector<IceHalo::Math::Vec3f> pts0;
  pts0.emplace_back(1.0f, 0.0f, h);
  pts0.emplace_back(0.5f, kSqrt3 / 2, h);
  pts0.emplace_back(-0.5f, kSqrt3 / 2, h);
  pts0.emplace_back(-1.0f, 0.0f, h);
  pts0.emplace_back(-0.5f, -kSqrt3 / 2, h);
  pts0.emplace_back(0.5f, -kSqrt3 / 2, h);

  pts0.emplace_back(1.0f, 0.0f, -h);
  pts0.emplace_back(0.5f, kSqrt3 / 2, -h);
  pts0.emplace_back(-0.5f, kSqrt3 / 2, -h);
  pts0.emplace_back(-1.0f, 0.0f, -h);
  pts0.emplace_back(-0.5f, -kSqrt3 / 2, -h);
  pts0.emplace_back(0.5f, -kSqrt3 / 2, -h);

  auto c = IceHalo::Crystal::createHexCylinder(h);
  const auto pts = c->getVertexes();

  EXPECT_EQ(pts.size(), 12ul);
  checkVertex(pts0, pts);
}


TEST_F(CrystalTest, IrregularHexCylinderVertex) {
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
