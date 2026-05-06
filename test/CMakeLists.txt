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
# Unit tests
add_executable(unit_test
  "${PROJ_TEST_DIR}/test_c_api.cpp"
  "${PROJ_TEST_DIR}/test_color_space.cpp"
  "${PROJ_TEST_DIR}/test_config_snapshot.cpp"
  "${PROJ_TEST_DIR}/test_crystal.cpp"
  "${PROJ_TEST_DIR}/test_geo3d.cpp"
  "${PROJ_TEST_DIR}/test_json.cpp"
  "${PROJ_TEST_DIR}/test_math.cpp"
  "${PROJ_TEST_DIR}/test_projection.cpp"
  "${PROJ_TEST_DIR}/test_queue.cpp"
  "${PROJ_TEST_DIR}/test_render_config.cpp"
  "${PROJ_TEST_DIR}/test_rng.cpp"
  "${PROJ_TEST_DIR}/test_filter.cpp"
  "${PROJ_TEST_DIR}/test_raypath_segments.cpp"
  "${PROJ_TEST_DIR}/test_optics.cpp"
  "${PROJ_TEST_DIR}/test_sim_data.cpp"
  "${PROJ_TEST_DIR}/test_simulator.cpp"
  "${PROJ_TEST_DIR}/test_threading_pool.cpp"
  "${PROJ_TEST_DIR}/test_window_sizing.cpp"
  "${PROJ_TEST_DIR}/test_axis_presets.cpp"
  "${PROJ_TEST_DIR}/test_slider_mapping.cpp"
  "${PROJ_TEST_DIR}/test_main.cpp")
target_include_directories(unit_test
  PRIVATE ${PROJ_TEST_DIR})
target_link_libraries(unit_test
  PRIVATE GTest::gtest ${CMAKE_THREAD_LIBS_INIT} lumice_obj)
set_target_properties(unit_test PROPERTIES
  BUILD_RPATH "$<TARGET_FILE_DIR:GTest::gtest>")

add_test(NAME "LumiceUnitTest" COMMAND unit_test
  "${PROJ_TEST_DIR}/fixtures/v3_config.json"
  "${CMAKE_CURRENT_BINARY_DIR}")
set_tests_properties("LumiceUnitTest" PROPERTIES LABELS "unit")

# ==================================================================================================
# Integration tests
add_executable(integration_test
  "${PROJ_TEST_DIR}/test_proj.cpp"
  "${PROJ_TEST_DIR}/test_integration_main.cpp")
target_include_directories(integration_test
  PRIVATE ${PROJ_TEST_DIR})
target_link_libraries(integration_test
  PRIVATE GTest::gtest ${CMAKE_THREAD_LIBS_INIT} lumice_obj)
set_target_properties(integration_test PROPERTIES
  BUILD_RPATH "$<TARGET_FILE_DIR:GTest::gtest>")

add_test(NAME "LumiceIntegrationTest" COMMAND integration_test
  "${PROJ_TEST_DIR}/fixtures/v3_config.json")
set_tests_properties("LumiceIntegrationTest" PROPERTIES LABELS "integration")
