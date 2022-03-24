# ==================================================================================================
# Add some definitions
if(MULTI_THREAD)
  add_compile_definitions(MULTI_THREAD)
endif()

if(RANDOM_SEED)
  add_compile_definitions(RANDOM_SEED)
endif()

message(STATUS "[src] CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}")


# ==================================================================================================
# Add targets
message(STATUS "[src] icehalo_include: ${icehalo_include}")

add_executable(IceHaloEndless
    endless_main.cpp
    ${icehalo_src})

target_include_directories(IceHaloEndless
    PUBLIC ${icehalo_include})
target_link_libraries(IceHaloEndless
    PUBLIC ${OpenCV_LIBS} ${Boost_LIBRARIES})
target_compile_definitions(IceHaloEndless
    PRIVATE $<$<CONFIG:Debug>:DEBUG>)
install(TARGETS IceHaloEndless
    DESTINATION "${CMAKE_INSTALL_PREFIX}")
