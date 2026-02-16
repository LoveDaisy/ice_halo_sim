# ==================================================================================================
message(STATUS "[src] CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}")

# ==================================================================================================
# Add targets

install(TARGETS icehalo
    DESTINATION "${CMAKE_INSTALL_PREFIX}")

add_executable(IceHalo
    "${PROJ_SRC_DIR}/main.c"
    "${PROJ_SRC_DIR}/stb_impl.c")
target_include_directories(IceHalo PRIVATE
    ${stb_SOURCE_DIR})
target_link_libraries(IceHalo PRIVATE
    icehalo)
install(TARGETS IceHalo
    DESTINATION "${CMAKE_INSTALL_PREFIX}")
