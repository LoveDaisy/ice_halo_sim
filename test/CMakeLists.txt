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
# Test layout (270.3 — purpose-primary, layer × subsystem):
#   - unit_correctness_test (LABEL unit-correctness)
#   - parity_test           (LABEL parity, abbreviated; full layer name is parity-cross-backend)
#   - golden_analytic_test  (LABEL golden-analytic)
# See doc/testing-architecture.md §5 for the naming contract.

# --------------------------------------------------------------------------------------------------
# unit_correctness_test
add_executable(unit_correctness_test
  "${PROJ_TEST_DIR}/test_main.cpp"
  # core/
  "${PROJ_TEST_DIR}/unit-correctness/core/test_math.cpp"
  "${PROJ_TEST_DIR}/unit-correctness/core/test_geo3d.cpp"
  "${PROJ_TEST_DIR}/unit-correctness/core/test_optics.cpp"
  "${PROJ_TEST_DIR}/unit-correctness/core/test_crystal.cpp"
  "${PROJ_TEST_DIR}/unit-correctness/core/test_rng.cpp"
  "${PROJ_TEST_DIR}/unit-correctness/core/test_filter.cpp"
  "${PROJ_TEST_DIR}/unit-correctness/core/test_filter_spec.cpp"
  "${PROJ_TEST_DIR}/unit-correctness/core/test_component_gate.cpp"
  "${PROJ_TEST_DIR}/unit-correctness/core/test_crosslayer_component.cpp"
  # scrum-color-predicate-symmetry: color pass symmetry (per-ref P/B/D)
  "${PROJ_TEST_DIR}/unit-correctness/core/test_color_spec_groups.cpp"
  "${PROJ_TEST_DIR}/unit-correctness/core/test_collect_data_symmetry_groups.cpp"
  "${PROJ_TEST_DIR}/unit-correctness/core/test_color_symmetry_oracle.cpp"
  "${PROJ_TEST_DIR}/unit-correctness/core/test_device_filter_check_host.cpp"
  "${PROJ_TEST_DIR}/unit-correctness/core/test_reduce_raypath_audit.cpp"
  "${PROJ_TEST_DIR}/unit-correctness/core/test_raypath_segments.cpp"
  "${PROJ_TEST_DIR}/unit-correctness/core/test_simulator.cpp"
  "${PROJ_TEST_DIR}/unit-correctness/core/test_exit_records.cpp"
  "${PROJ_TEST_DIR}/unit-correctness/core/test_pcg_ray_base_split.cpp"
  "${PROJ_TEST_DIR}/unit-correctness/core/test_wl_stream_decouple.cpp"
  "${PROJ_TEST_DIR}/unit-correctness/core/test_proj.cpp"
  # config/
  "${PROJ_TEST_DIR}/unit-correctness/config/test_json.cpp"
  "${PROJ_TEST_DIR}/unit-correctness/config/test_config_snapshot.cpp"
  "${PROJ_TEST_DIR}/unit-correctness/config/test_render_config.cpp"
  "${PROJ_TEST_DIR}/unit-correctness/config/test_sim_data.cpp"
  "${PROJ_TEST_DIR}/unit-correctness/config/test_component_table.cpp"
  "${PROJ_TEST_DIR}/unit-correctness/config/test_color_gate_table.cpp"
  "${PROJ_TEST_DIR}/unit-correctness/config/test_raypath_color_config.cpp"
  "${PROJ_TEST_DIR}/unit-correctness/config/test_color_class_table.cpp"
  # server/
  "${PROJ_TEST_DIR}/unit-correctness/server/test_c_api.cpp"
  "${PROJ_TEST_DIR}/unit-correctness/server/test_stats_consumer.cpp"
  "${PROJ_TEST_DIR}/unit-correctness/server/test_ray_num_semantics.cpp"
  "${PROJ_TEST_DIR}/unit-correctness/server/test_render_consumer_component_lanes.cpp"
  "${PROJ_TEST_DIR}/unit-correctness/server/test_component_compositor.cpp"
  # util/
  "${PROJ_TEST_DIR}/unit-correctness/util/test_color_space.cpp"
  "${PROJ_TEST_DIR}/unit-correctness/util/test_queue.cpp"
  "${PROJ_TEST_DIR}/unit-correctness/util/test_threading_pool.cpp"
  "${PROJ_TEST_DIR}/unit-correctness/util/test_cpu_info.cpp"
  # gui/
  "${PROJ_TEST_DIR}/unit-correctness/gui/test_axis_presets.cpp"
  "${PROJ_TEST_DIR}/unit-correctness/gui/test_slider_mapping.cpp"
  "${PROJ_TEST_DIR}/unit-correctness/gui/test_window_sizing.cpp"
  "${PROJ_TEST_DIR}/unit-correctness/gui/test_ev_auto.cpp"
  "${PROJ_TEST_DIR}/unit-correctness/gui/test_composite_exposure_push.cpp"
  "${PROJ_TEST_DIR}/unit-correctness/gui/test_filter_sop_grammar.cpp")
target_include_directories(unit_correctness_test
  PRIVATE ${PROJ_TEST_DIR})
target_link_libraries(unit_correctness_test
  PRIVATE GTest::gtest ${CMAKE_THREAD_LIBS_INIT} lumice_obj)
set_target_properties(unit_correctness_test PROPERTIES
  BUILD_RPATH "$<TARGET_FILE_DIR:GTest::gtest>")

add_test(NAME "LumiceUnitCorrectnessTest" COMMAND unit_correctness_test
  "${PROJ_TEST_DIR}/fixtures/v3_config.json"
  "${CMAKE_CURRENT_BINARY_DIR}")
set_tests_properties("LumiceUnitCorrectnessTest" PROPERTIES LABELS "unit-correctness")

# --------------------------------------------------------------------------------------------------
# parity_test (parity-cross-backend layer; label abbreviated per §5)
add_executable(parity_test
  "${PROJ_TEST_DIR}/test_main.cpp"
  "${PROJ_TEST_DIR}/parity-cross-backend/backend/test_metal_trace_parity.cpp"
  "${PROJ_TEST_DIR}/parity-cross-backend/backend/test_metal_root_gen.cpp"
  "${PROJ_TEST_DIR}/parity-cross-backend/backend/test_metal_trace_backend.cpp"
  "${PROJ_TEST_DIR}/parity-cross-backend/backend/test_cpu_trace_backend.cpp"
  "${PROJ_TEST_DIR}/parity-cross-backend/backend/test_cuda_rich_exit.cpp"
  # task-331.5 raypath-color foundation: CPU-vs-Metal component-mask parity.
  "${PROJ_TEST_DIR}/parity-cross-backend/backend/test_metal_component_mask_parity.cpp"
  # task-331.6 raypath-color foundation: CPU-vs-CUDA component-mask parity.
  "${PROJ_TEST_DIR}/parity-cross-backend/backend/test_cuda_component_mask_parity.cpp")
if(APPLE)
  # scrum-267 task-msl-filter-match-port: parity harness for device filter MATCH.
  # ObjC++ source (.mm) — needs ARC and the Metal/Foundation frameworks.
  # task-270.8 boundary-hardening: kFilterMatchTestKernelSrc moved out of
  # liblumice into metal_filter_match_test_src.mm (parity layer only).
  target_sources(parity_test PRIVATE
    "${PROJ_TEST_DIR}/parity-cross-backend/backend/test_metal_filter_match_parity.mm"
    "${PROJ_TEST_DIR}/parity-cross-backend/backend/metal_filter_match_test_src.mm")
  set_source_files_properties(
    "${PROJ_TEST_DIR}/parity-cross-backend/backend/test_metal_filter_match_parity.mm"
    "${PROJ_TEST_DIR}/parity-cross-backend/backend/metal_filter_match_test_src.mm"
    PROPERTIES COMPILE_FLAGS "-fobjc-arc")
  target_link_libraries(parity_test PRIVATE "-framework Metal" "-framework Foundation")
  # task-#283 (metal-build-time-metallib): the parity harness compiles a
  # standalone Metal library that calls into the shared MSL helpers
  # (DeviceFilterCheck etc.). Since `kFilterMatchHelperSrc` is retired, the
  # harness prepends `kLumiceCombinedKernelSrc` (the build-time-embedded
  # canonical source) before its test kernel. Wire the generated header dir +
  # build dependency here so the parity test can `#include` it.
  target_include_directories(parity_test PRIVATE "${LUMICE_METAL_GEN_DIR}")
  add_dependencies(parity_test lumice_metal_shaders)
endif()
target_include_directories(parity_test
  PRIVATE ${PROJ_TEST_DIR})
target_link_libraries(parity_test
  PRIVATE GTest::gtest ${CMAKE_THREAD_LIBS_INIT} lumice_obj)
set_target_properties(parity_test PROPERTIES
  BUILD_RPATH "$<TARGET_FILE_DIR:GTest::gtest>")

add_test(NAME "LumiceParityTest" COMMAND parity_test
  "${PROJ_TEST_DIR}/fixtures/v3_config.json"
  "${CMAKE_CURRENT_BINARY_DIR}")
set_tests_properties("LumiceParityTest" PROPERTIES LABELS "parity")

# --------------------------------------------------------------------------------------------------
# golden_analytic_test
#
# The MetalGoldenRay anchors in backend/ (extracted from test_metal_trace_parity.cpp
# by 270.3) drive MetalTraceBackend with analytically constructed rays; the
# CpuGoldenRay anchors (added by scrum-cuda-backend-mvp subtask 1) mirror the
# same physics anchors on CpuTraceBackend so the absolute battery runs cross-
# platform. The projection analytic tests in core/ (test_projection.cpp) use
# pure math formulas and run on all platforms. test_multi_ms_golden.cpp is
# guarded by #if APPLE; on non-Apple platforms the Metal backend golden
# anchors are absent but the CPU anchors plus the 38 projection analytic tests
# still compile and run. The target needs Metal/Foundation frameworks on APPLE
# because test_multi_ms_golden.cpp exercises MetalTraceBackend.
add_executable(golden_analytic_test
  "${PROJ_TEST_DIR}/test_main.cpp"
  "${PROJ_TEST_DIR}/golden-analytic/backend/test_cpu_golden_rays.cpp"
  "${PROJ_TEST_DIR}/golden-analytic/backend/test_multi_ms_golden.cpp"
  "${PROJ_TEST_DIR}/golden-analytic/core/test_projection.cpp")
target_include_directories(golden_analytic_test
  PRIVATE ${PROJ_TEST_DIR})
target_link_libraries(golden_analytic_test
  PRIVATE GTest::gtest ${CMAKE_THREAD_LIBS_INIT} lumice_obj)
if(APPLE)
  target_link_libraries(golden_analytic_test PRIVATE "-framework Metal" "-framework Foundation")
endif()
set_target_properties(golden_analytic_test PROPERTIES
  BUILD_RPATH "$<TARGET_FILE_DIR:GTest::gtest>")

add_test(NAME "LumiceGoldenAnalyticTest" COMMAND golden_analytic_test
  "${PROJ_TEST_DIR}/fixtures/v3_config.json"
  "${CMAKE_CURRENT_BINARY_DIR}")
set_tests_properties("LumiceGoldenAnalyticTest" PROPERTIES LABELS "golden-analytic")

# --------------------------------------------------------------------------------------------------
# CUDA multi-MS parity (Python pytest, dev49-only guard via LUMICE_CUDA_ENABLED env)
#
# Mirrors the test_cuda_exit_seam_parity.py pattern: implemented as pytest to
# reuse the e2e runner fixtures (CApiRunner) rather than duplicating them in a
# new gtest binary. On Mac (LUMICE_CUDA_ENABLED=OFF) the test self-skips; it is
# intended to run on dev49 via `pytest test_cuda_multi_ms_parity.py` or via
# `ctest -R CudaMultiMsParity` on dev49 after LUMICE_CUDA_ENABLED=1 in env.
find_program(PYTEST_EXECUTABLE NAMES pytest pytest3 HINTS ENV PATH)
if(PYTEST_EXECUTABLE)
  add_test(NAME "CudaMultiMsParity"
    COMMAND ${PYTEST_EXECUTABLE} -v
      "${PROJ_TEST_DIR}/parity-cross-backend/backend/test_cuda_multi_ms_parity.py"
    WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}")
  set_tests_properties("CudaMultiMsParity" PROPERTIES
    LABELS "parity;cuda"
    ENVIRONMENT "LUMICE_CUDA_ENABLED=1")
endif()
