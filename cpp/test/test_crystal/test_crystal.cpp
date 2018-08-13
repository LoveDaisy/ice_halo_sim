#include "test_crystal.h"

#include "crystal.h"

using namespace IceHalo;

void CrystalTest::SetUp()
{ }

void CrystalTest::TearDown()
{ }

TEST_F(CrystalTest, HexCylinderNotNull) {
    Crystal *c = Crystal::createHexCylinder(1.2f);
    EXPECT_NE(c, nullptr);
}

// TEST_F(CrystalTest, ByDefaultBazFalseIsFalse) {
//     Foo foo(m_bar);
//     EXPECT_EQ(foo.baz(false), false);
// }

// TEST_F(CrystalTest, SometimesBazFalseIsTrue) {
//     Foo foo(m_bar);
//     // Have norf return true for once
//     EXPECT_CALL(m_bar,norf()).WillOnce(Return(true));
//     EXPECT_EQ(foo.baz(false), false);
// }