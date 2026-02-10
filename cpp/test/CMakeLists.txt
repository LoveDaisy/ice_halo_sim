# ==================================================================================================
# Google Test
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
CPMAddPackage(
  NAME GTest
  GITHUB_REPOSITORY google/googletest
  VERSION 1.15.2
  OPTIONS "INSTALL_GTEST OFF"
  EXCLUDE_FROM_ALL YES
)

# ==================================================================================================
add_executable(unit_test
  "${PROJ_TEST_DIR}/test_crystal.cpp"
  "${PROJ_TEST_DIR}/test_context.cpp"
  "${PROJ_TEST_DIR}/test_optics.cpp"
  "${PROJ_TEST_DIR}/test_rng.cpp"
  "${PROJ_TEST_DIR}/test_serialize.cpp"
  "${PROJ_TEST_DIR}/test_main.cpp")
target_include_directories(unit_test
  PRIVATE ${PROJ_TEST_DIR})
target_link_libraries(unit_test
  PRIVATE GTest::gtest ${CMAKE_THREAD_LIBS_INIT} icehalo_lib)
set_target_properties(unit_test PROPERTIES
  BUILD_RPATH "$<TARGET_FILE_DIR:GTest::gtest>")

add_executable(unit_test_v3
  "${PROJ_TEST_DIR}/test_v3_crystal.cpp"
  "${PROJ_TEST_DIR}/test_v3_json.cpp"
  "${PROJ_TEST_DIR}/test_v3_math.cpp"
  "${PROJ_TEST_DIR}/test_v3_proj.cpp"
  "${PROJ_TEST_DIR}/test_main.cpp")
target_include_directories(unit_test_v3
  PRIVATE ${PROJ_TEST_DIR})
target_link_libraries(unit_test_v3
  PRIVATE GTest::gtest ${CMAKE_THREAD_LIBS_INIT} icehalo_lib)
set_target_properties(unit_test_v3 PROPERTIES
  BUILD_RPATH "$<TARGET_FILE_DIR:GTest::gtest>")

add_test(NAME "IceHaloUnitTest" COMMAND unit_test
  "${PROJ_TEST_DIR}/config_01.json"
  "${CMAKE_CURRENT_BINARY_DIR}")
add_test(NAME "IceHaloUnitTestV3" COMMAND unit_test_v3
  "${PROJ_ROOT}/v3_config_example.json"
  "${CMAKE_CURRENT_BINARY_DIR}")
