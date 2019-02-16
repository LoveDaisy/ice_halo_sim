#include "context.h"
#include "mymath.h"

#include "gtest/gtest.h"

#include <string>

std::string config_file_name;

namespace {

class ContextTest : public ::testing::Test {
};


TEST_F(ContextTest, CreateNotNull) {
  auto context = IceHalo::SimulationContext::createFromFile(config_file_name.c_str());
  EXPECT_NE(context, nullptr);
}

}  // namespace
