# ==================================================================================================
# Google Test
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
CPMAddPackage(
  NAME GTest
  URL https://github.com/google/googletest/releases/download/v1.15.2/googletest-1.15.2.tar.gz
  URL_HASH SHA256=7b42b4d6ed48810c5362c265a17faebe90dc2373c885e5216439d37927f02926
  OPTIONS "INSTALL_GTEST OFF"
  EXCLUDE_FROM_ALL YES
)

# ==================================================================================================
add_executable(unit_test
  "${PROJ_TEST_DIR}/test_crystal.cpp"
  "${PROJ_TEST_DIR}/test_json.cpp"
  "${PROJ_TEST_DIR}/test_math.cpp"
  "${PROJ_TEST_DIR}/test_proj.cpp"
  "${PROJ_TEST_DIR}/test_rng.cpp"
  "${PROJ_TEST_DIR}/test_main.cpp")
target_include_directories(unit_test
  PRIVATE ${PROJ_TEST_DIR})
target_link_libraries(unit_test
  PRIVATE GTest::gtest ${CMAKE_THREAD_LIBS_INIT} lumice_obj)
set_target_properties(unit_test PROPERTIES
  BUILD_RPATH "$<TARGET_FILE_DIR:GTest::gtest>")

add_test(NAME "LumiceUnitTest" COMMAND unit_test
  "${PROJ_ROOT}/config_example.json"
  "${CMAKE_CURRENT_BINARY_DIR}")
