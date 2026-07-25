#include <string>

#include "gtest/gtest.h"

std::string config_file_name;
std::string working_dir;
// Repository root, passed as an optional 3rd positional argv by the CMake add_test() COMMAND.
// Tests that need to enumerate on-disk config corpora (rather than a single fixture file) read
// it. Empty when the binary is run without it — such tests must skip rather than fail, so the
// binary stays runnable by hand.
std::string repo_root_dir;

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);

  if (argc >= 3) {
    config_file_name = std::string(argv[1]);
    working_dir = std::string(argv[2]);
  } else {
    config_file_name = "";
    working_dir = "";
  }
  if (argc >= 4) {
    repo_root_dir = std::string(argv[3]);
  } else {
    repo_root_dir = "";
  }

  return RUN_ALL_TESTS();
}