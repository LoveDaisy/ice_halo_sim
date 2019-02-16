#include "gtest/gtest.h"

#include <string>

std::string config_file_name;

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  if (argc > 1) {
    config_file_name = std::string(argv[1]);
  } else {
    config_file_name = "";
  }

  return RUN_ALL_TESTS();
}