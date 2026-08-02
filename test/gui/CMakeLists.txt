# ==================================================================================================
# GUI test target: gui_test
message(STATUS "[gui-test] Building GUI tests")

set(IMGUI_TE_DIR ${imgui_test_engine_SOURCE_DIR}/imgui_test_engine)

add_executable(gui_test
    test_gui_main.cpp
    functional/test_gui_smoke.cpp
    functional/test_gui_interaction.cpp
    functional/test_gui_preview_animation.cpp
    functional/test_gui_export.cpp
    functional/test_gui_import_export.cpp
    functional/test_gui_color_window.cpp
    functional/test_gui_face_number_overlay.cpp
    functional/test_gui_overlay_labels.cpp
    functional/test_gui_lifecycle.cpp
    functional/test_gui_composite_preview.cpp
    functional/test_gui_state_reconcile.cpp
    functional/test_gui_sampling_density_stats.cpp
    functional/test_gui_user_defaults.cpp
    functional/test_gui_defaults_panel.cpp
    functional/test_project_world_dir.cpp
    functional/test_render_handedness_guard.cpp
    visual/test_gui_visual.cpp
    visual/test_gui_render.cpp
    visual/test_gui_bg.cpp
    visual/test_gui_crystal_renderer.cpp
    visual/test_gui_auto_ev.cpp
    visual/test_gui_capture_smoke.cpp
    visual/test_gui_lens_projection.cpp
    visual/test_gui_modal_layout.cpp
    visual/test_gui_defaults_panel.cpp
    responsiveness/test_gui_perf.cpp
    test_screenshot.cpp
    ${IMGUI_TE_DIR}/imgui_te_engine.cpp
    ${IMGUI_TE_DIR}/imgui_te_context.cpp
    ${IMGUI_TE_DIR}/imgui_te_coroutine.cpp
    ${IMGUI_TE_DIR}/imgui_te_utils.cpp
    ${IMGUI_TE_DIR}/imgui_te_exporters.cpp
    ${IMGUI_TE_DIR}/imgui_te_ui.cpp
    ${IMGUI_TE_DIR}/imgui_te_perftool.cpp
    ${IMGUI_TE_DIR}/imgui_capture_tool.cpp)

# ${PROJ_TEST_DIR} (the test/ root) is on the list so gui_test can reach the target-agnostic
# helpers under test/support/ by their "support/..." path — the same spelling gui_unit_test uses,
# which keeps one include line valid in both targets.
target_include_directories(gui_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${PROJ_TEST_DIR}
    ${IMGUI_TE_DIR})

# `lumice_obj` (objects), not `lumice`: the white-box tests reference internal core
# symbols that a shared `lumice` does not export, and linking the objects keeps exactly
# one copy of core state in the process. lumice_gui_obj deliberately declares no core
# dependency of its own — see src/gui/CMakeLists.txt.
target_link_libraries(gui_test PRIVATE lumice_gui_obj lumice_obj)

target_compile_definitions(gui_test PRIVATE
    IMGUI_ENABLE_TEST_ENGINE
    IMGUI_TEST_ENGINE_ENABLE_COROUTINE_STDTHREAD_IMPL=1
    LUMICE_TEST_REF_DIR="${CMAKE_CURRENT_SOURCE_DIR}/references"
    LUMICE_E2E_CONFIG_DIR="${CMAKE_SOURCE_DIR}/test/e2e/configs"
    # task-command-reset-owner AC4: source-scan single-owner gate — the test
    # reads app.cpp to prove reset primitives (InvalidateStagedTexture,
    # g_crystal_mesh_hash=0) only appear inside ResetFrontendState.
    LUMICE_GUI_APP_CPP_PATH="${CMAKE_SOURCE_DIR}/src/gui/app.cpp")

# macOS frameworks required by GLFW
if(APPLE)
  target_link_libraries(gui_test PRIVATE
      "-framework Cocoa"
      "-framework IOKit"
      "-framework CoreVideo")
endif()

add_test(NAME "LumiceGUITest" COMMAND gui_test)
set_tests_properties(LumiceGUITest PROPERTIES LABELS "gui")
