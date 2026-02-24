#include <string>

#include "gtest/gtest.h"

std::string config_file_name;
std::string working_dir;

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);

  if (argc == 3) {
    config_file_name = std::string(argv[1]);
    working_dir = std::string(argv[2]);
  } else {
    config_file_name = "";
    working_dir = "";
  }

  return RUN_ALL_TESTS();
}