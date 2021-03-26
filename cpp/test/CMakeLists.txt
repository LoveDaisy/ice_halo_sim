# ==================================================================================================
# Add some definitions
if (BUILD_TEST)
  add_compile_definitions(FOR_TEST)
endif()

message(STATUS "[test] icehalo_libs: ${icehalo_libs}")
message(STATUS "[test] icehalo_include: ${icehalo_include}")
add_executable(unit_test
  ${icehalo_src}
  "${PROJ_TEST_DIR}/test_crystal.cpp"
  "${PROJ_TEST_DIR}/test_context.cpp"
  "${PROJ_TEST_DIR}/test_optics.cpp"
  "${PROJ_TEST_DIR}/test_serialize.cpp"
  "${PROJ_TEST_DIR}/test_main.cpp")
target_include_directories(unit_test
  PUBLIC ${icehalo_include} ${PROJ_TEST_DIR})
target_link_libraries(unit_test
  PUBLIC gtest ${CMAKE_THREAD_LIBS_INIT} ${Boost_LIBRARIES})


add_executable(RayTracingTest
  ${icehalo_src}
  "${PROJ_TEST_DIR}/test_ray_tracing_main.cpp")
target_include_directories(RayTracingTest
    PUBLIC ${icehalo_include} ${PROJ_TEST_DIR})
target_link_libraries(RayTracingTest
    PUBLIC ${CMAKE_THREAD_LIBS_INIT} ${Boost_LIBRARIES})


add_test(NAME "IceHaloUnitTest" COMMAND unit_test
        "${PROJ_ROOT}/test/config_01.json"
        "${CMAKE_CURRENT_BINARY_DIR}")
add_test(NAME "IceHaloRayTracingTest" 
         COMMAND python3 ${PROJECT_SOURCE_DIR}/test/run_ray_tracing_test.py
          --exe $<TARGET_FILE:RayTracingTest>
          --config ${PROJECT_SOURCE_DIR}/test/config_01.json
          --ref ${PROJECT_SOURCE_DIR}/test/ray_tracing_result.log)
