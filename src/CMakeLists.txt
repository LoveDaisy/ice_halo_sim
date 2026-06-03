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
install(TARGETS Lumice
    DESTINATION "${CMAKE_INSTALL_PREFIX}")
