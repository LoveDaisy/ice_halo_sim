# ==================================================================================================
# Google Test
# gtest_force_shared_crt drives GoogleTest's own pre-CMP0091 mechanism: it rewrites the
# dynamic-CRT flag into the static one in the inherited compiler flags itself, and knows
# nothing about CMAKE_MSVC_RUNTIME_LIBRARY. It has to agree with the CRT this build
# actually selected or gtest links against a different CRT than everything it is linked
# into — which surfaces as LNK2038/LNK1319 at link time, not as anything readable here.
# Hence it follows LUMICE_MSVC_STATIC_CRT (set in the top-level CMakeLists.txt, where the
# CRT choice and its reasoning live) rather than restating the condition: this variable
# reports what the build measured, so the two cannot drift apart. Kept explicit rather
# than deleted so the coupling stays visible at the point it applies.
if(LUMICE_MSVC_STATIC_CRT)
  set(gtest_force_shared_crt OFF CACHE BOOL "" FORCE)
else()
  set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
endif()
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
  "${PROJ_TEST_DIR}/unit-correctness/core/test_ev_anchor.cpp"
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
  "${PROJ_TEST_DIR}/unit-correctness/server/test_render_consumer_exposure_scale.cpp"
  "${PROJ_TEST_DIR}/unit-correctness/server/test_render_consumer_post_snapshot_fusion.cpp"
  "${PROJ_TEST_DIR}/unit-correctness/server/test_render_consumer_visible_mask.cpp"
  "${PROJ_TEST_DIR}/unit-correctness/server/test_render_consumer_horizon.cpp"
  "${PROJ_TEST_DIR}/unit-correctness/server/test_component_compositor.cpp"
  "${PROJ_TEST_DIR}/unit-correctness/server/test_composite_background.cpp"
  # Result-frame lifetime guard + the buffer pool underneath it, and the completion/drain
  # contract next to it. Physically here rather than under test/regression-sentinel/
  # because that layer has no C++ target — it is pytest-only, and standing one up would be
  # new test infrastructure, not these guards.
  "${PROJ_TEST_DIR}/unit-correctness/server/test_result_frame_lifetime.cpp"
  "${PROJ_TEST_DIR}/unit-correctness/server/test_drain_contract.cpp"
  # util/
  "${PROJ_TEST_DIR}/unit-correctness/util/test_color_space.cpp"
  "${PROJ_TEST_DIR}/unit-correctness/util/test_queue.cpp"
  "${PROJ_TEST_DIR}/unit-correctness/util/test_threading_pool.cpp"
  "${PROJ_TEST_DIR}/unit-correctness/util/test_cpu_info.cpp"
  "${PROJ_TEST_DIR}/unit-correctness/util/test_bit_utils.cpp"
  "${PROJ_TEST_DIR}/unit-correctness/util/test_illuminant.cpp"
  # gui/
  "${PROJ_TEST_DIR}/unit-correctness/gui/test_axis_presets.cpp"
  "${PROJ_TEST_DIR}/unit-correctness/gui/test_filter_sop_grammar.cpp"
  # Widget enable/disable rules and the shape-scalar domain table, extracted out of the draw
  # functions in app_panels.cpp / edit_modals.cpp so they can be asserted over their whole domain
  # (every SimState, every AspectPreset, every LUMICE_SHAPE_SCALAR_* slot) instead of one branch
  # per UI-driven case. Header-only and ImGui-free, hence this target rather than gui_unit_test.
  "${PROJ_TEST_DIR}/unit-correctness/gui/test_gui_widget_rules.cpp"
  # Same reason as the line above: mono_exposure_scale.hpp is header-only and ImGui-free, so its
  # cases belong to the target that does not link lumice_gui_obj.
  "${PROJ_TEST_DIR}/unit-correctness/gui/test_mono_exposure_scale.cpp"
  # Same reason again: preview_jacobian.hpp is header-only and ImGui-free. Its subject is the
  # preview shader's relative-illumination factor, and the oracle it is held to is a numerical
  # Jacobian rather than the GLSL twin, so the case needs no GL context of any kind.
  "${PROJ_TEST_DIR}/unit-correctness/gui/test_preview_jacobian.cpp"
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
  "${PROJ_TEST_DIR}/golden-analytic/core/test_visible_mask.cpp"
  "${PROJ_TEST_DIR}/golden-analytic/core/test_horizon_mask.cpp"
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
# `ctest -L "unit-correctness|composition-correctness|parity|golden-analytic"` selector in CI and scripts/test.sh pick it
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
    "${PROJ_TEST_DIR}/unit-correctness/gui/test_state_reconcile.cpp"
    "${PROJ_TEST_DIR}/unit-correctness/gui/test_preview_renderer.cpp"
    "${PROJ_TEST_DIR}/unit-correctness/gui/test_export_params.cpp"
    "${PROJ_TEST_DIR}/unit-correctness/gui/test_crystal_renderer.cpp"
    "${PROJ_TEST_DIR}/unit-correctness/gui/test_render_handedness_guard.cpp"
    "${PROJ_TEST_DIR}/unit-correctness/gui/test_visible_mask_gui_parity.cpp"
    "${PROJ_TEST_DIR}/unit-correctness/gui/test_horizon_gui_parity.cpp"
    "${PROJ_TEST_DIR}/unit-correctness/gui/test_axis_absent_alignment.cpp"
    "${PROJ_TEST_DIR}/unit-correctness/gui/test_user_defaults.cpp"
    "${PROJ_TEST_DIR}/unit-correctness/gui/test_render_bg_logic.cpp"
    "${PROJ_TEST_DIR}/unit-correctness/gui/test_sampling_density_stats.cpp"
    "${PROJ_TEST_DIR}/unit-correctness/gui/test_server_poller.cpp"
    "${PROJ_TEST_DIR}/unit-correctness/gui/test_face_number_overlay.cpp"
    "${PROJ_TEST_DIR}/unit-correctness/gui/test_overlay_labels.cpp"
    "${PROJ_TEST_DIR}/unit-correctness/gui/test_lens_border_geometry.cpp"
    "${PROJ_TEST_DIR}/unit-correctness/gui/test_composite_preview.cpp"
    "${PROJ_TEST_DIR}/unit-correctness/gui/test_color_window_logic.cpp"
    "${PROJ_TEST_DIR}/unit-correctness/gui/test_gui_identity_formatting.cpp"
    "${PROJ_TEST_DIR}/unit-correctness/gui/test_semantic_colors.cpp")
  target_include_directories(gui_unit_test
    PRIVATE ${PROJ_TEST_DIR})
  # Source-scan gates read app.cpp off disk to prove a helper is REACHED from every document-entry
  # path, not merely that it works when called. gui_test carries the same definition.
  #
  # LUMICE_E2E_CONFIG_DIR: one migrated import/export case round-trips a tracked fixture
  # (test/e2e/configs/raypath_color_three_arcs.json) rather than an inline string literal, so it
  # breaks when the shipped fixture and the GUI importer drift apart. gui_test defines the same
  # path for the cases that stayed there.
  #
  # LUMICE_TEST_FIXTURE_DIR: the absent-`axis` cross-parser oracle reads v3_config.json off disk as
  # its drift sentinel. Not routed through the existing repo_root_dir argv the other corpus-reading
  # tests use — add_test(NAME "LumiceGuiUnitTest") below passes no positional arguments, so that
  # variable is empty in this binary and the tests reading it must skip; a compile definition is
  # what makes the fixture unconditionally reachable here.
  target_compile_definitions(gui_unit_test PRIVATE
    LUMICE_GUI_APP_CPP_PATH="${CMAKE_SOURCE_DIR}/src/gui/app.cpp"
    LUMICE_E2E_CONFIG_DIR="${CMAKE_SOURCE_DIR}/test/e2e/configs"
    LUMICE_TEST_FIXTURE_DIR="${CMAKE_SOURCE_DIR}/test/fixtures")
  # `lumice_obj` (objects), not `lumice` — a NAMED exemption, not an unexamined default.
  # Measured 2026-08-06 by switching this line to `lumice` and building the shared Release
  # flavor (the only configuration where it bites: -fvisibility=hidden is set for
  # Release/MinSizeRel, so liblumice.dylib exports the 53 LUMICE_* C API symbols and nothing
  # else). It fails to link on exactly four symbols, all from one file:
  #
  #   lumice::Rotation::Rotation() / ::Rotation(const float*, float)
  #   lumice::Rotation::Chain(const Rotation&) / ::Apply(float*, size_t) const
  #     <- test/unit-correctness/gui/test_render_handedness_guard.cpp
  #
  # That is the guard asserting the GUI's preview projection and core's renderer agree on
  # handedness, and it computes its oracle by calling core's Rotation directly. Being white-box
  # is what makes it an oracle rather than a second implementation of the thing under test, so
  # the fix is not to rewrite it against the C API.
  #
  # gui_test does NOT need this exemption — it links plain `lumice` (see test/gui/CMakeLists.txt).
  target_link_libraries(gui_unit_test
    PRIVATE GTest::gtest ${CMAKE_THREAD_LIBS_INIT} lumice_gui_obj lumice_obj)
  set_target_properties(gui_unit_test PROPERTIES
    BUILD_RPATH "$<TARGET_FILE_DIR:GTest::gtest>")

  add_test(NAME "LumiceGuiUnitTest" COMMAND gui_unit_test)
  set_tests_properties("LumiceGuiUnitTest" PROPERTIES LABELS "unit-correctness")
endif()

# --------------------------------------------------------------------------------------------------
# composition_correctness_test — composition-correctness layer, gui subsystem.
#
# A LAYER of its own, not a third link-variant of the unit layer. The unit layer answers "does this
# one src/gui unit compute the right answer"; this one answers "do these two-or-more units, wired
# together the way the app wires them, produce the right observable result". The distinction is not
# stylistic: 173 cases / 5285 lines were measured to already be making cross-unit claims while
# living in a bucket named for single-unit ones, which is what made "is this suite too big" an
# unanswerable question — a claim's home has to say what kind of claim it is.
#
# Hence its OWN CTest LABEL (`composition-correctness`), not a shared `unit-correctness`. Layer
# identity lives in the label: reusing the unit layer's label would make this layer non-existent to
# every selector in the repo, which is the same failure the label was introduced to end.
#
# Link shape mirrors gui_unit_test exactly (lumice_gui_obj + lumice_obj, imgui_te_stubs.cpp for the
# ImGuiTestEngineHook_* symbols lumice_gui_obj's PUBLIC IMGUI_ENABLE_TEST_ENGINE demands, no window,
# no GL context, no test engine) — see that target's comment above for why each piece is there. What
# it does NOT share is a process: gui_unit_test_env.cpp is compiled in here as well, because the
# personal-defaults isolation it installs is per-process. Leaving it out would silently resolve
# user-config lookups to the real OS config directory of whatever machine runs the suite.
if(BUILD_GUI)
  add_executable(composition_correctness_test
    "${PROJ_TEST_DIR}/test_main.cpp"
    "${PROJ_SRC_DIR}/gui/imgui_te_stubs.cpp"
    "${PROJ_TEST_DIR}/unit-correctness/gui/gui_unit_test_env.cpp"
    "${PROJ_TEST_DIR}/composition-correctness/gui/test_document_roundtrip_chain.cpp"
    "${PROJ_TEST_DIR}/composition-correctness/gui/test_lmc_load_atomicity_chain.cpp"
    "${PROJ_TEST_DIR}/composition-correctness/gui/test_document_defaults_chain.cpp"
    "${PROJ_TEST_DIR}/composition-correctness/gui/test_document_switch_chain.cpp"
    "${PROJ_TEST_DIR}/composition-correctness/gui/test_legacy_document_chain.cpp"
    "${PROJ_TEST_DIR}/composition-correctness/gui/test_json_document_contract_chain.cpp"
    "${PROJ_TEST_DIR}/composition-correctness/gui/test_scene_commit_chain.cpp"
    "${PROJ_TEST_DIR}/composition-correctness/gui/test_filter_reconstruct_chain.cpp"
    "${PROJ_TEST_DIR}/composition-correctness/gui/test_raypath_color_document_chain.cpp"
    "${PROJ_TEST_DIR}/composition-correctness/gui/test_run_lifecycle_chain.cpp"
    "${PROJ_TEST_DIR}/composition-correctness/gui/test_run_warning_chain.cpp"
    "${PROJ_TEST_DIR}/composition-correctness/gui/test_user_defaults_chain.cpp"
    "${PROJ_TEST_DIR}/composition-correctness/gui/test_field_editor_chain.cpp"
    "${PROJ_TEST_DIR}/composition-correctness/gui/test_edit_modal_chain.cpp"
    "${PROJ_TEST_DIR}/composition-correctness/gui/test_preview_projection_chain.cpp"
    "${PROJ_TEST_DIR}/composition-correctness/gui/test_absolute_ev_additivity_chain.cpp")
  target_include_directories(composition_correctness_test
    PRIVATE ${PROJ_TEST_DIR})
  # Same three definitions gui_unit_test carries, for the same reasons (see its block above):
  # source-scan gates that read app.cpp off disk, the shipped e2e config fixtures, and the
  # test/fixtures corpus. add_test() below passes no positional arguments, so a compile definition
  # is what makes these paths reachable at all in this binary.
  target_compile_definitions(composition_correctness_test PRIVATE
    LUMICE_GUI_APP_CPP_PATH="${CMAKE_SOURCE_DIR}/src/gui/app.cpp"
    LUMICE_E2E_CONFIG_DIR="${CMAKE_SOURCE_DIR}/test/e2e/configs"
    LUMICE_TEST_FIXTURE_DIR="${CMAKE_SOURCE_DIR}/test/fixtures")
  # `lumice_obj`, not `lumice` — the same named exemption gui_unit_test documents above: a
  # composition chain that cross-checks the GUI against core's own primitives needs the internal
  # symbols that liblumice.dylib's -fvisibility=hidden does not export.
  target_link_libraries(composition_correctness_test
    PRIVATE GTest::gtest ${CMAKE_THREAD_LIBS_INIT} lumice_gui_obj lumice_obj)
  set_target_properties(composition_correctness_test PROPERTIES
    BUILD_RPATH "$<TARGET_FILE_DIR:GTest::gtest>")

  add_test(NAME "LumiceCompositionCorrectnessTest" COMMAND composition_correctness_test)
  set_tests_properties("LumiceCompositionCorrectnessTest" PROPERTIES LABELS "composition-correctness")
endif()

# --------------------------------------------------------------------------------------------------
# CUDA multi-MS parity (Python pytest, CUDA-host-only guard via LUMICE_CUDA_ENABLED env)
#
# Mirrors the test_cuda_exit_seam_parity.py pattern: implemented as pytest to
# reuse the e2e runner fixtures (CApiRunner) rather than duplicating them in a
# new gtest binary. On Mac (LUMICE_CUDA_ENABLED=OFF) the test self-skips; it is
# intended to run on a CUDA-capable host via `pytest -m slow test_cuda_multi_ms_parity.py`
# or via `ctest -R CudaMultiMsParity` there after LUMICE_CUDA_ENABLED=1 in env.
#
# `-m slow` is REQUIRED, not decorative: every test in both files below is
# slow-marked, and `pyproject.toml`'s `addopts = ["-m", "not slow"]` pins a bare
# `pytest` to the fast subset. Without an explicit `-m`, these two invocations
# collect zero tests and pytest exits 5 ("no tests collected"), which ctest
# reports as a hard FAILURE — on every platform, since both carry the `parity`
# label that `ctest -L "unit-correctness|composition-correctness|parity|golden-analytic"` selects.
# Where these two actually run, which is not where a reader assumes:
#
# This find_program gates the whole block, and CI's four build-matrix jobs never install
# pytest — so on those runners PYTEST_EXECUTABLE is not found, the if() body does not execute,
# and neither test is ever REGISTERED. Their ctest step reports "out of 3"; the same selector
# locally reports 5. The two missing ones are exactly CudaMultiMsParity and
# CudaRandomGeometryParity.
#
# So the capture point for these is a local `./scripts/test.sh quick` (or a direct ctest on
# a CUDA-capable host), NOT CI. Stated here because "it's an add_test, therefore CI runs it" is
# the natural reading and it is wrong.
#
# Deliberately NOT resolved here: whether to install pytest in the build jobs (making them run)
# or to accept the status quo (leaving them local-only) is a decision with cost on both sides
# and is not this comment's to make. Both readings leave the paragraph above true.
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
  # `CudaMultiMsParity` above so a CUDA-capable host can run it via
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
