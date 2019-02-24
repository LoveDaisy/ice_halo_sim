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

  void checkCrystal(const IceHalo::CrystalPtrU& c1, const IceHalo::CrystalPtrU& c2) {
    auto n1 = c1->GetFaceNorm();
    auto n2 = c2->GetFaceNorm();
    auto fn1 = c1->GetFaceNumberMap();
    auto fn2 = c2->GetFaceNumberMap();

    ASSERT_EQ(c1->TotalFaces(), c2->TotalFaces());
    ASSERT_EQ(fn1.size(), fn2.size());
    for (decltype(c1->TotalFaces()) i = 0; i < c1->TotalFaces(); i++) {
      const auto n = n1 + i * 3;
      int fn = fn1[i];

      bool find_same_norm = false;
      for (decltype(c2->TotalFaces()) j = 0; j < c2->TotalFaces(); j++) {
        const auto tmp_n = n2 + j * 3;
        int tmp_fn = fn2[j];
        if (IceHalo::Math::DiffNorm3(n, tmp_n) < IceHalo::Math::kFloatEps) {
          find_same_norm = true;
          EXPECT_EQ(tmp_fn, fn);
        }
      }
      EXPECT_TRUE(find_same_norm);
    }

    for (decltype(c2->TotalFaces()) i = 0; i < c2->TotalFaces(); i++) {
      const auto n = n2 + i * 3;
      int fn = fn2[i];

      bool find_same_norm = false;
      for (decltype(c1->TotalFaces()) j = 0; j < c1->TotalFaces(); j++) {
        const auto tmp_n = n1 + j * 3;
        int tmp_fn = fn1[j];
        if (IceHalo::Math::DiffNorm3(n, tmp_n) < IceHalo::Math::kFloatEps) {
          find_same_norm = true;
          EXPECT_EQ(tmp_fn, fn);
        }
      }
      EXPECT_TRUE(find_same_norm);
    }

    auto v1 = c1->GetVertexes();
    auto v2 = c2->GetVertexes();
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
  /* Prism */
  auto c = IceHalo::Crystal::CreateHexPrism(1.2f);
  EXPECT_NE(c, nullptr);

  /* Pyramid */
  c = IceHalo::Crystal::CreateHexPyramid(1.2f, 1.2f, 1.2f);
  EXPECT_NE(c, nullptr);

  c = IceHalo::Crystal::CreateHexPyramid(1, 1, 1.2f, 1.2f, 1.2f);
  EXPECT_NE(c, nullptr);

  c = IceHalo::Crystal::CreateHexPyramid(1, 1, 2, 3, 1.2f, 1.2f, 1.2f);
  EXPECT_NE(c, nullptr);

  /* IrregularHexPrism */
  float dist[6] = { 1.0f, 1.0f, 1.5f, 1.0f, 2.5f, 1.0f };
  c = IceHalo::Crystal::CreateIrregularHexPrism(dist, 1.2f);
  EXPECT_NE(c, nullptr);

  /* IrregularHexPyramid */
  int idx[4] = { 1, 1, 1, 1 };
  float h[3] = { 0.3f, 1.2f, 0.9f };
  c = IceHalo::Crystal::CreateIrregularHexPyramid(dist, idx, h);
  EXPECT_NE(c, nullptr);
}


TEST_F(CrystalTest, HexPrismVertex) {
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

  auto c = IceHalo::Crystal::CreateHexPrism(h);
  EXPECT_EQ(c->GetVertexes().size(), 12ul);
  checkVertex(pts0, c->GetVertexes());
  checkFaceId(faceId0, c->GetFaceNumberMap());
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

  auto c = IceHalo::Crystal::CreateHexPyramid(h1, h2, h3);
  EXPECT_EQ(c->GetVertexes().size(), 24ul);
  checkVertex(pts0, c->GetVertexes());
  checkFaceId(faceId0, c->GetFaceNumberMap());

  c = IceHalo::Crystal::CreateHexPyramid(1, 1, h1, h2, h3);
  EXPECT_EQ(c->GetVertexes().size(), 24ul);
  checkVertex(pts0, c->GetVertexes());
  checkFaceId(faceId0, c->GetFaceNumberMap());

  c = IceHalo::Crystal::CreateHexPyramid(1, 1, 1, 1, h1, h2, h3);
  EXPECT_EQ(c->GetVertexes().size(), 24ul);
  checkVertex(pts0, c->GetVertexes());
  checkFaceId(faceId0, c->GetFaceNumberMap());
}


TEST_F(CrystalTest, IrregularHexPyramid) {
  float h1 = 0.3;
  float h2 = 0.5;
  float h3 = 0.85;

  float dist[6] = { 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f };
  int idx[4] = { 1, 1, 1, 1 };
  float hs[3] = { h1, h2, h3 };

  auto c1 = IceHalo::Crystal::CreateHexPyramid(h1, h2, h3);
  auto c2 = IceHalo::Crystal::CreateIrregularHexPyramid(dist, idx, hs);
  EXPECT_EQ(c1->TotalFaces(), c2->TotalFaces());
  EXPECT_EQ(c1->TotalVertexes(), c2->TotalVertexes());
  checkCrystal(c1, c2);
}


TEST_F(CrystalTest, IrregularHexPrismVertex0) {
  using IceHalo::Math::kSqrt3;

  float h = 1.2f;
  float dist[6] = { 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f };

  auto c1 = IceHalo::Crystal::CreateHexPrism(h);
  auto c2 = IceHalo::Crystal::CreateIrregularHexPrism(dist, h);

  EXPECT_EQ(c1->TotalFaces(), c2->TotalFaces());
  EXPECT_EQ(c1->TotalVertexes(), c2->TotalVertexes());
  checkCrystal(c1, c2);
}


TEST_F(CrystalTest, IrregularHexPrismVertex1) {
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

  auto c1 = IceHalo::Crystal::CreateCustomCrystal(pts0, faces0, faceIdMap0);
  auto c2 = IceHalo::Crystal::CreateIrregularHexPrism(dist, h);
  EXPECT_EQ(c1->TotalVertexes(), c2->TotalVertexes());
  EXPECT_EQ(c1->TotalFaces(), c2->TotalFaces());
  checkCrystal(c1, c2);
}

}  // namespace
