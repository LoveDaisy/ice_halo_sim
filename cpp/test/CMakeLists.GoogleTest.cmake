# Prepare for google test framework

cmake_minimum_required(VERSION 2.8.2)

project(googletest-prepare NONE)

include(ExternalProject)
ExternalProject_Add(googletest
  SOURCE_DIR        "${GoogleTest_DIR}"
  BINARY_DIR        "${CMAKE_CURRENT_BINARY_DIR}/googletest-build"
  CONFIGURE_COMMAND ""
  BUILD_COMMAND     ""
  INSTALL_COMMAND   ""
  TEST_COMMAND      ""
)