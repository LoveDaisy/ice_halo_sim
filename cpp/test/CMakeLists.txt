# ==================================================================================================
# Google Test
find_package(GTest)

if(NOT GTest_FOUND)
  include(FetchContent)
  FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/archive/refs/tags/v1.15.2.tar.gz
    URL_HASH SHA256=7b42b4d6ed48810c5362c265a17faebe90dc2373c885e5216439d37927f02926
    PREFIX ${CMAKE_CURRENT_BINARY_DIR}/gtest
  )

  # For Windows: Prevent overriding the parent project's compiler/linker settings
  set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
  FetchContent_Populate(googletest)
  add_subdirectory(${googletest_SOURCE_DIR} ${googletest_BINARY_DIR} EXCLUDE_FROM_ALL)
endif()

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
