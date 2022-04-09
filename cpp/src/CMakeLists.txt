# ==================================================================================================
message(STATUS "[src] CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}")


# ==================================================================================================
# Add targets
add_executable(IceHaloEndless
    endless_main.cpp)
target_link_libraries(IceHaloEndless
    PUBLIC icehalo_lib ${OpenCV_LIBS} Boost::filesystem Boost::system)
target_compile_definitions(IceHaloEndless
    PRIVATE $<$<CONFIG:Debug>:DEBUG>)
install(TARGETS IceHaloEndless
    DESTINATION "${CMAKE_INSTALL_PREFIX}")
