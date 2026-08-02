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
#   - gui_unit_test         (LABEL unit-correctness — same layer, second target; see below)
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
  "${PROJ_TEST_DIR}/unit-correctness/core/test_crystal_sync_group_sampling.cpp"
  "${PROJ_TEST_DIR}/unit-correctness/core/test_rng.cpp"
  "${PROJ_TEST_DIR}/unit-correctness/core/test_distribution_slots.cpp"
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
  "${PROJ_TEST_DIR}/unit-correctness/config/test_crystal_sync_group.cpp"
  "${PROJ_TEST_DIR}/unit-correctness/config/test_config_snapshot.cpp"
  "${PROJ_TEST_DIR}/unit-correctness/config/test_render_config.cpp"
  "${PROJ_TEST_DIR}/unit-correctness/config/test_sim_data.cpp"
  "${PROJ_TEST_DIR}/unit-correctness/config/test_color_gate_table.cpp"
  "${PROJ_TEST_DIR}/unit-correctness/config/test_raypath_color_config.cpp"
  "${PROJ_TEST_DIR}/unit-correctness/config/test_color_class_table.cpp"
  # server/
  "${PROJ_TEST_DIR}/unit-correctness/server/test_c_api.cpp"
  "${PROJ_TEST_DIR}/unit-correctness/server/test_c_api_scene.cpp"
  "${PROJ_TEST_DIR}/unit-correctness/server/test_json_parser_parity.cpp"
  "${PROJ_TEST_DIR}/unit-correctness/server/test_stats_consumer.cpp"
  "${PROJ_TEST_DIR}/unit-correctness/server/test_ray_num_semantics.cpp"
  "${PROJ_TEST_DIR}/unit-correctness/server/test_render_consumer_component_lanes.cpp"
  "${PROJ_TEST_DIR}/unit-correctness/server/test_render_consumer_empty_batch.cpp"
  "${PROJ_TEST_DIR}/unit-correctness/server/test_component_compositor.cpp"
  # util/
  "${PROJ_TEST_DIR}/unit-correctness/util/test_color_space.cpp"
  "${PROJ_TEST_DIR}/unit-correctness/util/test_queue.cpp"
  "${PROJ_TEST_DIR}/unit-correctness/util/test_threading_pool.cpp"
  "${PROJ_TEST_DIR}/unit-correctness/util/test_cpu_info.cpp"
  "${PROJ_TEST_DIR}/unit-correctness/util/test_bit_utils.cpp"
  # gui/
  "${PROJ_TEST_DIR}/unit-correctness/gui/test_axis_presets.cpp"
  "${PROJ_TEST_DIR}/unit-correctness/gui/test_slider_mapping.cpp"
  "${PROJ_TEST_DIR}/unit-correctness/gui/test_window_sizing.cpp"
  "${PROJ_TEST_DIR}/unit-correctness/gui/test_ev_auto.cpp"
  "${PROJ_TEST_DIR}/unit-correctness/gui/test_composite_exposure_push.cpp"
  "${PROJ_TEST_DIR}/unit-correctness/gui/test_filter_sop_grammar.cpp"
  # User-defaults eligibility resolver + platform config-dir helpers. Both are header-only, so
  # they link against lumice_obj alone. The half that calls file_io.cpp needs lumice_gui_obj and
  # therefore lives in a different target of this same layer — gui_unit_test, defined at the
  # bottom of this file (test/unit-correctness/gui/test_defaults_diff.cpp), or in gui_test when it
  # also needs a window (test/gui/functional/test_gui_user_defaults.cpp).
  "${PROJ_TEST_DIR}/unit-correctness/gui/test_user_defaults_eligibility.cpp"
  # backend/: CUDA-only white-box tests for the geometry-pool rebuild gate.
  # Body is `#if LUMICE_CUDA_ENABLED`, so on non-CUDA hosts the TU is empty
  # (zero contributed symbols).
  "${PROJ_TEST_DIR}/unit-correctness/backend/test_cuda_geom_pool_rebuild.cpp"
  # CUDA K-shape pool locality + transit coverage + knob-off collapse (Metal
  # KShapePool_* suite CUDA sibling). Body `#if LUMICE_CUDA_ENABLED`, so on
  # non-CUDA hosts the TU is empty (zero contributed symbols).
  "${PROJ_TEST_DIR}/unit-correctness/backend/test_cuda_kshape_pool_locality.cpp")
target_include_directories(unit_correctness_test
  PRIVATE ${PROJ_TEST_DIR})
target_link_libraries(unit_correctness_test
  PRIVATE GTest::gtest ${CMAKE_THREAD_LIBS_INIT} lumice_obj)
set_target_properties(unit_correctness_test PROPERTIES
  BUILD_RPATH "$<TARGET_FILE_DIR:GTest::gtest>")

# 3rd argv = repository root. test_json_parser_parity.cpp enumerates the on-disk config corpora
# (examples/, test/e2e/configs/, test/fixtures/) from it, so newly added corpus files are covered
# automatically without editing a hard-coded file list.
add_test(NAME "LumiceUnitCorrectnessTest" COMMAND unit_correctness_test
  "${PROJ_TEST_DIR}/fixtures/v3_config.json"
  "${CMAKE_CURRENT_BINARY_DIR}"
  "${CMAKE_SOURCE_DIR}")
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
  "${PROJ_TEST_DIR}/parity-cross-backend/backend/test_cuda_component_mask_parity.cpp"
  # CUDA K-shape pool × filter judgment: K=0/K=8 mean-invariance under an
  # active EntryExit filter + loose CPU ballpark. Body `#if LUMICE_CUDA_ENABLED`,
  # so on non-CUDA hosts the TU is empty.
  "${PROJ_TEST_DIR}/parity-cross-backend/backend/test_cuda_kshape_filter_parity.cpp")
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
  "${PROJ_TEST_DIR}/golden-analytic/core/test_projection.cpp"
  "${PROJ_TEST_DIR}/golden-analytic/core/test_closed_form_prism.cpp"
  "${PROJ_TEST_DIR}/golden-analytic/core/test_closed_form_pyramid.cpp"
  "${PROJ_TEST_DIR}/golden-analytic/core/test_incidence_sampling_polygon_oracle.cpp"
  "${PROJ_TEST_DIR}/golden-analytic/core/test_triangulation_swap_spike.cpp")
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
# gui_unit_test — unit-correctness layer, gui subsystem, second CMake target.
#
# Links lumice_gui_obj the way gui_test does, but with NO GLFW window, NO GL context and NO ImGui
# test engine: a windowless home for the gui subsystem's pure-logic cases. Its sources sit in the
# same test/unit-correctness/gui/ directory as unit_correctness_test's, and it carries the same
# LABEL — the split is a LINK boundary (these cases call into file_io.cpp / user_defaults.cpp),
# not a layer or subsystem boundary. Sharing the label is what lets the existing
# `ctest -L "unit-correctness|parity|golden-analytic"` selector in CI and scripts/test.sh pick it
# up with no change. See doc/testing-architecture.md §5 for why this is a second target rather
# than a second layer, and for the naming rule that lets it keep a link-derived name.
#
# imgui_te_stubs.cpp is in the source list for the same reason LumiceGUI has it: lumice_gui_obj
# compiles ImGui with IMGUI_ENABLE_TEST_ENGINE PUBLIC (src/gui/CMakeLists.txt), so every consumer
# must supply the four ImGuiTestEngineHook_* symbols — either the real engine (gui_test compiles
# imgui_test_engine) or these no-ops (LumiceGUI, and now this target). Omitting them links anyway
# under Release+static, where LTO drops the never-reached ImGui rendering code before the
# references materialize; every non-LTO configuration (any shared build, any Debug build) fails
# to link instead. The stubs are unreachable here in practice — this target never creates an
# ImGui context, let alone renders a frame.
if(BUILD_GUI)
  add_executable(gui_unit_test
    "${PROJ_TEST_DIR}/test_main.cpp"
    "${PROJ_SRC_DIR}/gui/imgui_te_stubs.cpp"
    "${PROJ_TEST_DIR}/unit-correctness/gui/gui_unit_test_env.cpp"
    "${PROJ_TEST_DIR}/unit-correctness/gui/test_defaults_diff.cpp"
    "${PROJ_TEST_DIR}/unit-correctness/gui/test_smoke.cpp"
    "${PROJ_TEST_DIR}/unit-correctness/gui/test_crystal_renderer.cpp"
    "${PROJ_TEST_DIR}/unit-correctness/gui/test_state_reconcile.cpp"
    "${PROJ_TEST_DIR}/unit-correctness/gui/test_project_world_dir.cpp"
    "${PROJ_TEST_DIR}/unit-correctness/gui/test_render_handedness_guard.cpp"
    "${PROJ_TEST_DIR}/unit-correctness/gui/test_user_defaults.cpp"
    "${PROJ_TEST_DIR}/unit-correctness/gui/test_render_bg_logic.cpp"
    "${PROJ_TEST_DIR}/unit-correctness/gui/test_defaults_panel_registry.cpp"
    "${PROJ_TEST_DIR}/unit-correctness/gui/test_sampling_density_stats.cpp"
    "${PROJ_TEST_DIR}/unit-correctness/gui/test_lifecycle.cpp"
    "${PROJ_TEST_DIR}/unit-correctness/gui/test_face_number_overlay.cpp"
    "${PROJ_TEST_DIR}/unit-correctness/gui/test_overlay_labels.cpp"
    "${PROJ_TEST_DIR}/unit-correctness/gui/test_composite_preview.cpp"
    "${PROJ_TEST_DIR}/unit-correctness/gui/test_interaction_logic.cpp"
    "${PROJ_TEST_DIR}/unit-correctness/gui/test_import_export_logic.cpp"
    "${PROJ_TEST_DIR}/unit-correctness/gui/test_color_window_logic.cpp")
  target_include_directories(gui_unit_test
    PRIVATE ${PROJ_TEST_DIR})
  # Source-scan gates read app.cpp off disk to prove a helper is REACHED from every document-entry
  # path, not merely that it works when called. gui_test carries the same definition.
  #
  # LUMICE_E2E_CONFIG_DIR: one migrated import/export case round-trips a tracked fixture
  # (test/e2e/configs/raypath_color_three_arcs.json) rather than an inline string literal, so it
  # breaks when the shipped fixture and the GUI importer drift apart. gui_test defines the same
  # path for the cases that stayed there.
  target_compile_definitions(gui_unit_test PRIVATE
    LUMICE_GUI_APP_CPP_PATH="${CMAKE_SOURCE_DIR}/src/gui/app.cpp"
    LUMICE_E2E_CONFIG_DIR="${CMAKE_SOURCE_DIR}/test/e2e/configs")
  target_link_libraries(gui_unit_test
    PRIVATE GTest::gtest ${CMAKE_THREAD_LIBS_INIT} lumice_gui_obj lumice_obj)
  set_target_properties(gui_unit_test PROPERTIES
    BUILD_RPATH "$<TARGET_FILE_DIR:GTest::gtest>")

  add_test(NAME "LumiceGuiUnitTest" COMMAND gui_unit_test)
  set_tests_properties("LumiceGuiUnitTest" PROPERTIES LABELS "unit-correctness")
endif()

# --------------------------------------------------------------------------------------------------
# CUDA multi-MS parity (Python pytest, dev49-only guard via LUMICE_CUDA_ENABLED env)
#
# Mirrors the test_cuda_exit_seam_parity.py pattern: implemented as pytest to
# reuse the e2e runner fixtures (CApiRunner) rather than duplicating them in a
# new gtest binary. On Mac (LUMICE_CUDA_ENABLED=OFF) the test self-skips; it is
# intended to run on dev49 via `pytest -m slow test_cuda_multi_ms_parity.py` or
# via `ctest -R CudaMultiMsParity` on dev49 after LUMICE_CUDA_ENABLED=1 in env.
#
# `-m slow` is REQUIRED, not decorative: every test in both files below is
# slow-marked, and `pyproject.toml`'s `addopts = ["-m", "not slow"]` pins a bare
# `pytest` to the fast subset. Without an explicit `-m`, these two invocations
# collect zero tests and pytest exits 5 ("no tests collected"), which ctest
# reports as a hard FAILURE — on every platform, since both carry the `parity`
# label that `ctest -L "unit-correctness|parity|golden-analytic"` selects.
find_program(PYTEST_EXECUTABLE NAMES pytest pytest3 HINTS ENV PATH)
if(PYTEST_EXECUTABLE)
  add_test(NAME "CudaMultiMsParity"
    COMMAND ${PYTEST_EXECUTABLE} -v -m slow
      "${PROJ_TEST_DIR}/parity-cross-backend/backend/test_cuda_multi_ms_parity.py"
    WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}")
  set_tests_properties("CudaMultiMsParity" PROPERTIES
    LABELS "parity;cuda"
    ENVIRONMENT "LUMICE_CUDA_ENABLED=1")
  # Random-geometry cross-backend regression. Same ctest scaffolding as
  # `CudaMultiMsParity` above so dev49 / win-builder can run it via
  # `ctest -R CudaRandomGeometryParity`; on Mac / no-CUDA hosts the body
  # self-skips at pytest collection time.
  add_test(NAME "CudaRandomGeometryParity"
    COMMAND ${PYTEST_EXECUTABLE} -v -m slow
      "${PROJ_TEST_DIR}/parity-cross-backend/backend/test_cuda_random_geometry_parity.py"
    WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}")
  set_tests_properties("CudaRandomGeometryParity" PROPERTIES
    LABELS "parity;cuda"
    ENVIRONMENT "LUMICE_CUDA_ENABLED=1")
endif()
