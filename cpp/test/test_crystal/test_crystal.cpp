#include "test_crystal.h"

#include "crystal.h"
#include "mymath.h"

#include <vector>

using namespace IceHalo;

void CrystalTest::SetUp()
{ }

void CrystalTest::TearDown()
{ }

void CrystalTest::checkVertex(const std::vector<Math::Vec3f> &vtx1, const std::vector<Math::Vec3f> &vtx2)
{
    ASSERT_EQ(vtx1.size(), vtx2.size());
    for (decltype(vtx1.size()) i = 0; i < vtx1.size(); i++) {
        const auto &p0 = vtx1[i];
        const auto &p = vtx2[i];

        EXPECT_TRUE(p0 == p);
    }
}



TEST_F(CrystalTest, CreateNotNull) {
    /* Cylinder */
    Crystal *c = Crystal::createHexCylinder(1.2f);
    EXPECT_NE(c, nullptr);
    delete c;

    /* Pyramid */
    c = Crystal::createHexPyramid(1.2f, 1.2f, 1.2f);
    EXPECT_NE(c, nullptr);
    delete c;

    c = Crystal::createHexPyramid(1, 1, 1.2f, 1.2f, 1.2f);
    EXPECT_NE(c, nullptr);
    delete c;

    c = Crystal::createHexPyramid(1, 1, 2, 3, 1.2f, 1.2f, 1.2f);
    EXPECT_NE(c, nullptr);
    delete c;

    /* IrregularHexCylinder */
    float dist[6] = { 1.0f, 1.0f, 1.5f, 1.0f, 2.5f, 1.0f };
    c = Crystal::createIrregularHexCylinder(dist, 1.2f);
    EXPECT_NE(c, nullptr);
    delete c;
}


TEST_F(CrystalTest, HexCylinderVertex) {
    using namespace Math;

    float h = 1.2f;
    float SQRT3_2 = 0.866025403784439f;

    std::vector<Math::Vec3f> pts0;
    pts0.reserve(12);
    pts0.emplace_back(Vec3f(1.0f, 0.0f, h));
    pts0.emplace_back(Vec3f(0.5f, SQRT3_2, h));
    pts0.emplace_back(Vec3f(-0.5f, SQRT3_2, h));
    pts0.emplace_back(Vec3f(-1.0f, 0.0f, h));
    pts0.emplace_back(Vec3f(-0.5f, -SQRT3_2, h));
    pts0.emplace_back(Vec3f(0.5f, -SQRT3_2, h));

    pts0.emplace_back(Vec3f(1.0f, 0.0f, -h));
    pts0.emplace_back(Vec3f(0.5f, SQRT3_2, -h));
    pts0.emplace_back(Vec3f(-0.5f, SQRT3_2, -h));
    pts0.emplace_back(Vec3f(-1.0f, 0.0f, -h));
    pts0.emplace_back(Vec3f(-0.5f, -SQRT3_2, -h));
    pts0.emplace_back(Vec3f(0.5f, -SQRT3_2, -h));

    Crystal *c = Crystal::createHexCylinder(h);
    const auto pts = c->getVertexes();

    EXPECT_EQ(pts.size(), 12ul);
    checkVertex(pts0, pts);

    delete c;
}


TEST_F(CrystalTest, IrregularHexCylinderVertex) {
    using namespace Math;

    float h = 1.2f;
    float SQRT3_2 = 0.866025403784439f;
    float dist[6] = { 1.0f, 1.0f, 1.5f, 1.0f, 2.5f, 1.0f };

    std::vector<Math::Vec3f> pts0;
    pts0.emplace_back(Vec3f(1.0f, 0.0f, h));
    pts0.emplace_back(Vec3f(0.5f, SQRT3_2, h));
    pts0.emplace_back(Vec3f(-1.0f, SQRT3_2, h));
    pts0.emplace_back(Vec3f(-1.25f, SQRT3_2/2, h));
    pts0.emplace_back(Vec3f(0.0f, -SQRT3_2*2, h));

    pts0.emplace_back(Vec3f(1.0f, 0.0f, -h));
    pts0.emplace_back(Vec3f(0.5f, SQRT3_2, -h));
    pts0.emplace_back(Vec3f(-1.0f, SQRT3_2, -h));
    pts0.emplace_back(Vec3f(-1.25f, SQRT3_2/2, -h));
    pts0.emplace_back(Vec3f(0.0f, -SQRT3_2*2, -h));

    Crystal *c = Crystal::createIrregularHexCylinder(dist, h);
    const auto pts = c->getVertexes();

    EXPECT_EQ(pts.size(), 10ul);
    checkVertex(pts0, pts);

    delete c;
}