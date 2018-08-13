#include "gtest/gtest.h"
// #include "mockbar.h"

// The fixture for testing class Foo.
class CrystalTest : public ::testing::Test {

protected:

    // You can do set-up work for each test here.
    CrystalTest() = default;

    // You can do clean-up work that doesn't throw exceptions here.
    virtual ~CrystalTest() = default;

    // If the constructor and destructor are not enough for setting up
    // and cleaning up each test, you can define the following methods:

    // Code here will be called immediately after the constructor (right
    // before each test).
    void SetUp() override;

    // Code here will be called immediately after each test (right
    // before the destructor).
    void TearDown() override;

};