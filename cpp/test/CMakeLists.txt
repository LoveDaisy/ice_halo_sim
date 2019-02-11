# enable_testing()

# Google test need thread support
find_package(Threads REQUIRED)

add_executable(test
  ${PROJ_SRC_DIR}/mymath.cpp
  ${PROJ_SRC_DIR}/crystal.cpp
  test_crystal.cpp)
target_include_directories(test
  PUBLIC ${PROJ_SRC_DIR})
target_link_libraries(test
  gtest_main)

add_test(NAME "IceHaloTest" COMMAND test)
