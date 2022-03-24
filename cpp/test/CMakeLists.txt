# ==================================================================================================
# Google Test
set(GoogleTest_DIR "${MODULE_ROOT}/googletest")

configure_file(CMakeLists.GoogleTest.cmake googletest-download/CMakeLists.txt)
execute_process(COMMAND ${CMAKE_COMMAND} -G "${CMAKE_GENERATOR}" .
  RESULT_VARIABLE result
  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/googletest-download)
if(result)
  message(FATAL_ERROR "CMake step for googletest failed: ${result}")
endif()
execute_process(COMMAND ${CMAKE_COMMAND} --build .
  RESULT_VARIABLE result
  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/googletest-download)
if(result)
  message(FATAL_ERROR "Build step for googletest failed: ${result}")
endif()

# Prevent overriding the parent project's compiler/linker
# settings on Windows
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

# Add googletest directly to our build. This defines
# the gtest and gtest_main targets.
add_subdirectory(${GoogleTest_DIR}
                 ${CMAKE_CURRENT_BINARY_DIR}/googletest-build
                 EXCLUDE_FROM_ALL)

# ==================================================================================================
message(STATUS "[test] icehalo_libs: ${icehalo_libs}")
message(STATUS "[test] icehalo_include: ${icehalo_include}")
add_executable(unit_test
  ${icehalo_src}
  "${PROJ_TEST_DIR}/test_crystal.cpp"
  "${PROJ_TEST_DIR}/test_context.cpp"
  "${PROJ_TEST_DIR}/test_optics.cpp"
  "${PROJ_TEST_DIR}/test_rng.cpp"
  "${PROJ_TEST_DIR}/test_serialize.cpp"
  "${PROJ_TEST_DIR}/test_main.cpp")
target_include_directories(unit_test
  PUBLIC ${icehalo_include}
  PRIVATE ${PROJ_TEST_DIR})
target_link_libraries(unit_test
  PRIVATE gtest ${CMAKE_THREAD_LIBS_INIT} ${Boost_LIBRARIES})


add_executable(RayTracingTest
  ${icehalo_src}
  "${PROJ_TEST_DIR}/test_ray_tracing_main.cpp")
target_include_directories(RayTracingTest
    PRIVATE ${icehalo_include} ${PROJ_TEST_DIR})
target_link_libraries(RayTracingTest
    PRIVATE ${CMAKE_THREAD_LIBS_INIT} ${Boost_LIBRARIES})
target_compile_definitions(RayTracingTest PRIVATE FOR_TEST)


add_test(NAME "IceHaloUnitTest" COMMAND unit_test
        "${PROJ_ROOT}/test/config_01.json"
        "${CMAKE_CURRENT_BINARY_DIR}")
add_test(NAME "IceHaloRayTracingTest" 
         COMMAND python3 ${PROJECT_SOURCE_DIR}/test/run_ray_tracing_test.py
          --exe $<TARGET_FILE:RayTracingTest>
          --config ${PROJECT_SOURCE_DIR}/test/config_01.json
          --ref ${PROJECT_SOURCE_DIR}/test/ray_tracing_result.log)
