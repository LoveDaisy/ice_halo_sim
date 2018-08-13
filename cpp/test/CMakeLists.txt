# Download and unpack googletest at configure time
configure_file(CMakeLists.txt.in googletest-download/CMakeLists.txt)
execute_process(COMMAND ${CMAKE_COMMAND} -G "${CMAKE_GENERATOR}" .
  RESULT_VARIABLE result
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/test/googletest-download )
if(result)
  message(FATAL_ERROR "CMake step for googletest failed: ${result}")
endif()
execute_process(COMMAND ${CMAKE_COMMAND} --build .
  RESULT_VARIABLE result
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/test/googletest-download )
if(result)
  message(FATAL_ERROR "Build step for googletest failed: ${result}")
endif()

# Prevent overriding the parent project's compiler/linker
# settings on Windows
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

# Add googletest directly to our build. This defines
# the gtest and gtest_main targets.
add_subdirectory(${CMAKE_BINARY_DIR}/googletest-src
                 ${CMAKE_BINARY_DIR}/googletest-build
                 EXCLUDE_FROM_ALL)

# The gtest/gtest_main targets carry header search path
# dependencies automatically when using CMake 2.8.11 or
# later. Otherwise we have to add them here ourselves.
if (CMAKE_VERSION VERSION_LESS 2.8.11)
  include_directories("${gtest_SOURCE_DIR}/include")
endif()
include_directories("${IceHalo_SOURCE_DIR}/src")


set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "-Wall -Wextra -msse4 -std=c++11")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")


# We need thread support
find_package(Threads REQUIRED)

add_subdirectory(test_crystal)

# # Enable ExternalProject CMake module
# include(ExternalProject)

# # Get GTest source and binary directories from CMake project
# ExternalProject_Get_Property(gtest source_dir binary_dir)

# # Create a libgtest target to be used as a dependency by test programs
# add_library(libgtest IMPORTED STATIC GLOBAL)
# add_dependencies(libgtest gtest)

# # Set libgtest properties
# set_target_properties(libgtest PROPERTIES
#     "IMPORTED_LOCATION" "${binary_dir}/googlemock/gtest/libgtest.a"
#     "IMPORTED_LINK_INTERFACE_LIBRARIES" "${CMAKE_THREAD_LIBS_INIT}"
# )

# # Create a libgmock target to be used as a dependency by test programs
# add_library(libgmock IMPORTED STATIC GLOBAL)
# add_dependencies(libgmock gtest)

# # Set libgmock properties
# set_target_properties(libgmock PROPERTIES
#     "IMPORTED_LOCATION" "${binary_dir}/googlemock/libgmock.a"
#     "IMPORTED_LINK_INTERFACE_LIBRARIES" "${CMAKE_THREAD_LIBS_INIT}"
# )

# # I couldn't make it work with INTERFACE_INCLUDE_DIRECTORIES
# include_directories("${source_dir}/googletest/include"
#                     "${source_dir}/googlemock/include")