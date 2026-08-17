# ==================================================================================================
# GUI test target: gui_test
message(STATUS "[gui-test] Building GUI tests")

set(IMGUI_TE_DIR ${imgui_test_engine_SOURCE_DIR}/imgui_test_engine)

add_executable(gui_test
    test_gui_main.cpp
    functional/test_gui_preview_animation.cpp
    functional/test_export.cpp
    functional/test_file_ops.cpp
    functional/test_color_window.cpp
    functional/test_edit_modal.cpp
    functional/test_filter_editor.cpp
    functional/test_gui_face_number_overlay.cpp
    functional/test_gui_overlay_labels.cpp
    functional/test_run_lifecycle.cpp
    functional/test_status_bar.cpp
    functional/test_defaults_panel.cpp
    functional/test_background_overlay.cpp
    functional/test_entry_management.cpp
    functional/test_scene_controls.cpp
    functional/test_execution_cluster.cpp
    functional/test_shell_chrome.cpp
    functional/test_document_column.cpp
    functional/test_log_panel.cpp
    functional/test_overlay_controls.cpp
    functional/test_preview_viewport.cpp
    functional/test_preview_texture.cpp
    functional/test_view_display_controls.cpp
    functional/test_gui_sim_smoke.cpp
    visual/test_preview_pixels.cpp
    visual/test_gui_capture_smoke.cpp
    visual/test_gui_lens_projection.cpp
    # visual/test_gui_modal_layout.cpp — OUT OF THE BUILD, not deleted.
    #
    # Its four scenes photograph the "Edit Entry" window's internal layout, and that window no
    # longer exists: the crystal / axis / filter editors are the document inspector's crystal page.
    # The scenes therefore have no subject, and the file cannot compile — it drives OpenEditModal.
    #
    # Left in the tree rather than removed because deciding what replaces it is a separate piece of
    # work (the reference images under test/gui/references/, the `modal_layout` entry in
    # scripts/regen_gui_test_refs.py's GROUPS, and its thresholds in _thresholds.json are all still
    # there and all still name it). Whoever picks that up needs the scene table and the capture
    # protocol in this file, and reconstructing them from git history would be pure waste.
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

# `lumice`, the public library — not the internal object library. This target used to link
# `lumice_obj` on the grounds that its white-box tests referenced internal core symbols a shared
# `lumice` would not export. That stopped being true when the handedness guard moved to
# test/unit-correctness/gui/ (commit 1526696b), and this line was simply not revisited.
#
# Measured 2026-08-06 in the shared Release flavor, the only configuration that can tell the
# difference (-fvisibility=hidden applies to Release/MinSizeRel, so liblumice.dylib exports the
# 53 LUMICE_* C API symbols and nothing else):
#   - gui_test links and runs against it;
#   - it defines ZERO core-internal symbols of its own (the static flavor defines 11);
#   - it has ZERO undefined references into lumice that are not C API.
# So this is the first hard evidence for a question the repo could not previously answer —
# whether the GUI can live on the C API alone. For gui_test, it can.
#
# Keeping it this way makes the boundary load-bearing rather than aspirational: a new GUI test
# that reaches past the C API now fails to link here instead of silently widening the surface.
# The one target that genuinely cannot do this is gui_unit_test — see the named exemption at its
# link line in test/CMakeLists.txt. lumice_gui_obj declares no core dependency of its own; see
# src/gui/CMakeLists.txt.
target_link_libraries(gui_test PRIVATE lumice_gui_obj lumice)

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
