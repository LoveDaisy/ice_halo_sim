# ==================================================================================================
# GUI test target: LumiceGUITests
message(STATUS "[gui-test] Building GUI tests")

set(IMGUI_TE_DIR ${imgui_test_engine_SOURCE_DIR}/imgui_test_engine)

add_executable(LumiceGUITests
    test_gui_main.cpp
    test_gui_smoke.cpp
    test_gui_interaction.cpp
    test_gui_render.cpp
    test_gui_export.cpp
    test_gui_visual.cpp
    test_gui_bg.cpp
    test_gui_import_export.cpp
    test_gui_perf.cpp
    test_gui_overlay_labels.cpp
    test_screenshot.cpp
    ${IMGUI_TE_DIR}/imgui_te_engine.cpp
    ${IMGUI_TE_DIR}/imgui_te_context.cpp
    ${IMGUI_TE_DIR}/imgui_te_coroutine.cpp
    ${IMGUI_TE_DIR}/imgui_te_utils.cpp
    ${IMGUI_TE_DIR}/imgui_te_exporters.cpp
    ${IMGUI_TE_DIR}/imgui_te_ui.cpp
    ${IMGUI_TE_DIR}/imgui_te_perftool.cpp
    ${IMGUI_TE_DIR}/imgui_capture_tool.cpp)

target_include_directories(LumiceGUITests PRIVATE ${IMGUI_TE_DIR})

target_link_libraries(LumiceGUITests PRIVATE lumice_gui_obj)

target_compile_definitions(LumiceGUITests PRIVATE
    IMGUI_ENABLE_TEST_ENGINE
    IMGUI_TEST_ENGINE_ENABLE_COROUTINE_STDTHREAD_IMPL=1
    LUMICE_TEST_REF_DIR="${CMAKE_CURRENT_SOURCE_DIR}/references")

# macOS frameworks required by GLFW
if(APPLE)
  target_link_libraries(LumiceGUITests PRIVATE
      "-framework Cocoa"
      "-framework IOKit"
      "-framework CoreVideo")
endif()

add_test(NAME "LumiceGUITest" COMMAND LumiceGUITests)
set_tests_properties(LumiceGUITest PROPERTIES LABELS "gui")
