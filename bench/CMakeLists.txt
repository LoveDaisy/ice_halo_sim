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
  "${PROJ_BENCH_DIR}/bench_scene.cpp"
  "${PROJ_BENCH_DIR}/bench_sim.cpp"
  "${PROJ_BENCH_DIR}/bench_filter.cpp"
  "${PROJ_BENCH_DIR}/bench_crystal.cpp"
  "${PROJ_BENCH_DIR}/bench_geom_closedform.cpp")
# Reach the promoted exact prism oracle at test/support/exact_prism_oracle.hpp
# via `#include "test/support/..."`. The dependency direction (bench →
# test/support) is inverted vs. usual but intentional: the oracle has one
# implementation and bench is the secondary consumer (golden-analytic tests
# are primary).
target_include_directories(lumice_bench
  PRIVATE "${PROJ_ROOT}")
target_link_libraries(lumice_bench
  PRIVATE benchmark::benchmark ${CMAKE_THREAD_LIBS_INIT} lumice_obj)
