# ==================================================================================================
# Google Benchmark
CPMAddPackage(
  NAME benchmark
  URL https://github.com/google/benchmark/archive/refs/tags/v1.9.1.tar.gz
  OPTIONS "BENCHMARK_ENABLE_TESTING OFF" "BENCHMARK_ENABLE_INSTALL OFF"
  EXCLUDE_FROM_ALL YES
)

# ==================================================================================================
set(PROJ_BENCH_DIR "${CMAKE_CURRENT_SOURCE_DIR}")

add_executable(lumice_bench
  "${PROJ_BENCH_DIR}/bench_main.cpp"
  "${PROJ_BENCH_DIR}/bench_optics.cpp"
  "${PROJ_BENCH_DIR}/bench_scene.cpp")
target_link_libraries(lumice_bench
  PRIVATE benchmark::benchmark ${CMAKE_THREAD_LIBS_INIT} lumice_obj)
