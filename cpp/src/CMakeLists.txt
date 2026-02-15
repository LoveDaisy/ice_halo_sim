# ==================================================================================================
message(STATUS "[src] CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}")

# ==================================================================================================
# Add targets

install(TARGETS icehalo
    DESTINATION "${CMAKE_INSTALL_PREFIX}")

add_executable(IceHalo
    "${PROJ_SRC_DIR}/main.c")
target_link_libraries(IceHalo PRIVATE
    icehalo)
install(TARGETS IceHalo
    DESTINATION "${CMAKE_INSTALL_PREFIX}")
