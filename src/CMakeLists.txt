# ==================================================================================================
message(STATUS "[src] CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}")

# ==================================================================================================
# Add targets

if(NOT WIN32)
  add_executable(Lumice
      "${PROJ_SRC_DIR}/main.c")
  target_include_directories(Lumice PRIVATE
      ${stb_SOURCE_DIR})
  target_link_libraries(Lumice PRIVATE
      lumice)
  install(TARGETS Lumice
      DESTINATION "${CMAKE_INSTALL_PREFIX}")
endif()
