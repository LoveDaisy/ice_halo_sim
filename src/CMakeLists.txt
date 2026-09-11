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
# root CMakeLists) so build/cmake_install/shared/Lumice is self-contained and
# runnable without rebuilding static — fixes "dyld: no LC_RPATH's found" when
# switching from a static to a shared build. Static builds archive lumice into the CLI, so
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

# Linux and Windows x64 releases: the package carries two builds of every entry point
# (`<name>.baseline` + `<name>.x86-64-v4` on Linux, `<name>.baseline.exe` +
# `<name>.x86-64-v3.exe` on Windows, produced by two configures at different LUMICE_ISA_LEVEL)
# and this launcher installed under the user-facing name picks one by CPUID at run time
# (src/launcher/isa_launcher.c and isa_launcher_win.c have the full story). It deliberately
# gets none of the engine: no lumice_obj, no -march flag — it has to run on exactly the CPUs
# the faster sidecar cannot. Built on every x86_64 configure of either OS so a plain CI build
# compiles it, but only release.yml's merge step installs it under the entry-point names; a
# local build/cmake_install/<flavor>/Lumice is still the real binary.
# Linux compiler floor: `__builtin_cpu_supports("x86-64-v4")` needs GCC 12+ or Clang 18+ (it
# is a compile error below that, not a silent misdetection). The Windows launcher uses the
# `<intrin.h>` CPUID/XGETBV intrinsics, which cl.exe and clang-cl both provide.
# -Wall -Wextra stay on the Linux branch only: they are GCC/Clang spellings, and the Windows
# file is compiled by whichever compiler the configure chose (cl.exe for the baseline tree
# release.yml takes the launcher from; clang-cl in the v3 tree, whose copy is discarded).
if(CMAKE_SYSTEM_NAME STREQUAL "Linux" AND CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|AMD64|amd64)$")
  add_executable(LumiceIsaLauncher "${PROJ_SRC_DIR}/launcher/isa_launcher.c")
  target_compile_options(LumiceIsaLauncher PRIVATE -Wall -Wextra)
elseif(CMAKE_SYSTEM_NAME STREQUAL "Windows" AND CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|AMD64|amd64)$")
  add_executable(LumiceIsaLauncher "${PROJ_SRC_DIR}/launcher/isa_launcher_win.c")
endif()
if(TARGET LumiceIsaLauncher)
  install(TARGETS LumiceIsaLauncher
      DESTINATION "${CMAKE_INSTALL_PREFIX}")
endif()
