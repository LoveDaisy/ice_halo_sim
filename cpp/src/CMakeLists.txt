# ==================================================================================================
message(STATUS "[src] CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}")

# ==================================================================================================
# Add targets

install(TARGETS icehalo_lib
    DESTINATION "${CMAKE_INSTALL_PREFIX}")

add_executable(IceHaloV3C
    "${PROJ_SRC_DIR}/main_v3_c.c")
target_link_libraries(IceHaloV3C PRIVATE
    icehalo_lib)
install(TARGETS IceHaloV3C
    DESTINATION "${CMAKE_INSTALL_PREFIX}")
