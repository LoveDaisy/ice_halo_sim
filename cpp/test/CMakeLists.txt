# ==================================================================================================
# Google Test
find_package(GTest)
if(NOT GTest_FOUND)
  if("${GTEST_DIR}" STREQUAL "")
    message(STATUS "[test] GTEST_DIR is NOT set! Use github source.")
    set(GTEST_REPO "https://github.com/google/googletest.git")
  else()
    message(STATUS "[test] GTEST_DIR is set! Use local gtest at ${GTEST_DIR}")
    set(GTEST_REPO "file://${GTEST_DIR}/.git")
  endif()
  
  include(FetchContent)
  FetchContent_Declare(
    googletest
    GIT_REPOSITORY ${GTEST_REPO}
    GIT_TAG release-1.11.0
    PREFIX ${CMAKE_CURRENT_BINARY_DIR}/gtest
  )
  # For Windows: Prevent overriding the parent project's compiler/linker settings
  set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
  FetchContent_Populate(googletest)
  add_subdirectory(${googletest_SOURCE_DIR} ${googletest_BINARY_DIR} EXCLUDE_FROM_ALL)
endif()

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
  PRIVATE GTest::gtest ${CMAKE_THREAD_LIBS_INIT} ${Boost_LIBRARIES})


add_executable(RayTracingTest
  ${icehalo_src}
  "${PROJ_TEST_DIR}/test_ray_tracing_main.cpp")
target_include_directories(RayTracingTest
    PRIVATE ${icehalo_include} ${PROJ_TEST_DIR})
target_link_libraries(RayTracingTest
    PRIVATE ${CMAKE_THREAD_LIBS_INIT} ${Boost_LIBRARIES})
target_compile_definitions(RayTracingTest PRIVATE FOR_TEST)


add_test(NAME "IceHaloUnitTest" COMMAND unit_test
        "${PROJ_TEST_DIR}/config_01.json"
        "${CMAKE_CURRENT_BINARY_DIR}")
add_test(NAME "IceHaloRayTracingTest" 
         COMMAND python3 ${PROJ_TEST_DIR}/run_ray_tracing_test.py
          --exe $<TARGET_FILE:RayTracingTest>
          --config ${PROJ_TEST_DIR}/config_01.json
          --ref ${PROJ_TEST_DIR}/ray_tracing_result.log)


# ==================================================================================================
# Google benchmark
find_package(benchmark)
if(NOT benchmark_FOUND)
  message(STATUS "[test] Can NOT find google-benchmark. Abort benchmarking.")
  return()
else()
  message(STATUS "[test] lib google-benchmark found.")
endif()

add_executable(bench
  ${icehalo_src}
  "${PROJ_TEST_DIR}/bench_math.cpp"
  "${PROJ_TEST_DIR}/bench_optics.cpp")
target_include_directories(bench PRIVATE ${icehalo_include})
target_link_libraries(bench PRIVATE benchmark::benchmark_main ${Boost_LIBRARIES})
