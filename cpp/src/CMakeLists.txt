# ==================================================================================================
message(STATUS "[src] CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}")

# ==================================================================================================
# Add targets
add_executable(IceHaloEndless
    "${PROJ_SRC_DIR}/endless_main.cpp")
target_link_libraries(IceHaloEndless
    PUBLIC icehalo_lib ${OpenCV_LIBS} Boost::filesystem Boost::system)
target_compile_definitions(IceHaloEndless
    PRIVATE $<$<CONFIG:Debug>:DEBUG>)
install(TARGETS IceHaloEndless
    DESTINATION "${CMAKE_INSTALL_PREFIX}")

add_executable(IceHaloV3
    "${PROJ_SRC_DIR}/main_v3.cpp")
target_link_libraries(IceHaloV3
    PUBLIC icehalo_lib ${OpenCV_LIBS} Boost::filesystem Boost::system)
target_compile_definitions(IceHaloV3
    PRIVATE $<$<CONFIG:Debug>:DEBUG>)
install(TARGETS IceHaloV3
    DESTINATION "${CMAKE_INSTALL_PREFIX}")

add_library(IceHaloLibV3 STATIC
    "${PROJ_SRC_DIR}/server/cserver.cpp")
target_link_libraries(IceHaloLibV3 PRIVATE
    icehalo_lib ${OpenCV_LIBS} Boost::filesystem Boost::system)
target_include_directories(IceHaloLibV3 PUBLIC
    $<BUILD_INTERFACE:${PROJ_SRC_DIR}/src/include>
    $<INSTALL_INTERFACE:include>)
target_compile_definitions(IceHaloLibV3 PRIVATE
    $<$<CONFIG:Debug>:DEBUG>)
install(TARGETS IceHaloLibV3
    DESTINATION "${CMAKE_INSTALL_PREFIX}")

add_executable(IceHaloV3C
    "${PROJ_SRC_DIR}/main_v3_c.c")
target_link_libraries(IceHaloV3C PRIVATE
    IceHaloLibV3)
install(TARGETS IceHaloV3C
    DESTINATION "${CMAKE_INSTALL_PREFIX}")
