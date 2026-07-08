# ==================================================================================================
# GUI test target: gui_test
message(STATUS "[gui-test] Building GUI tests")

set(IMGUI_TE_DIR ${imgui_test_engine_SOURCE_DIR}/imgui_test_engine)

add_executable(gui_test
    test_gui_main.cpp
    functional/test_gui_smoke.cpp
    functional/test_gui_interaction.cpp
    functional/test_gui_export.cpp
    functional/test_gui_import_export.cpp
    functional/test_gui_color_window.cpp
    functional/test_gui_face_number_overlay.cpp
    functional/test_gui_overlay_labels.cpp
    functional/test_gui_lifecycle.cpp
    functional/test_gui_composite_preview.cpp
    functional/test_project_world_dir.cpp
    functional/test_render_handedness_guard.cpp
    visual/test_gui_visual.cpp
    visual/test_gui_render.cpp
    visual/test_gui_bg.cpp
    visual/test_gui_crystal_renderer.cpp
    visual/test_gui_auto_ev.cpp
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

target_include_directories(gui_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${IMGUI_TE_DIR})

target_link_libraries(gui_test PRIVATE lumice_gui_obj)

target_compile_definitions(gui_test PRIVATE
    IMGUI_ENABLE_TEST_ENGINE
    IMGUI_TEST_ENGINE_ENABLE_COROUTINE_STDTHREAD_IMPL=1
    LUMICE_TEST_REF_DIR="${CMAKE_CURRENT_SOURCE_DIR}/references"
    LUMICE_E2E_CONFIG_DIR="${CMAKE_SOURCE_DIR}/test/e2e/configs")

# macOS frameworks required by GLFW
if(APPLE)
  target_link_libraries(gui_test PRIVATE
      "-framework Cocoa"
      "-framework IOKit"
      "-framework CoreVideo")
endif()

add_test(NAME "LumiceGUITest" COMMAND gui_test)
set_tests_properties(LumiceGUITest PROPERTIES LABELS "gui")
