# ==================================================================================================
message(STATUS "[src] CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}")

# ==================================================================================================
# Add targets

# cpu_info.cpp is compiled directly into the executable: main.cpp calls the
# internal lumice::PhysicalCoreCount(), which carries hidden visibility and is
# therefore not exported from the shared `lumice` library (BUILD_SHARED_LIBS).
add_executable(Lumice
    "${PROJ_SRC_DIR}/main.cpp"
    "${PROJ_SRC_DIR}/util/cpu_info.cpp")
target_include_directories(Lumice PRIVATE
    ${stb_SOURCE_DIR})
target_link_libraries(Lumice PRIVATE
    lumice)
# Shared builds link the CLI against @rpath/liblumice.dylib. Give the installed
# CLI an rpath into the sibling lib/ dir (where the dylib is installed, see the
# root CMakeLists) so build/cmake_install/Lumice is self-contained and runnable
# without rebuilding static — fixes "dyld: no LC_RPATH's found" when switching
# from a static to a shared build. Static builds archive lumice into the CLI, so
# there is no @rpath dependency and this rpath is simply not applied.
if(BUILD_SHARED_LIBS)
  if(APPLE)
    set_target_properties(Lumice PROPERTIES INSTALL_RPATH "@loader_path/lib")
  else()
    set_target_properties(Lumice PROPERTIES INSTALL_RPATH "$ORIGIN/lib")
  endif()
endif()
install(TARGETS Lumice
    DESTINATION "${CMAKE_INSTALL_PREFIX}")
