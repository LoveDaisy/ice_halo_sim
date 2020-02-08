if(BUILD_TEST)
  add_compile_definitions(FOR_TEST)
endif()

set(SOURCE_FILE
  ${PROJ_SRC_DIR}/context.cpp
  ${PROJ_SRC_DIR}/crystal.cpp
  ${PROJ_SRC_DIR}/file.cpp
  ${PROJ_SRC_DIR}/mymath.cpp
  ${PROJ_SRC_DIR}/obj_pool.cpp
  ${PROJ_SRC_DIR}/optics.cpp
  ${PROJ_SRC_DIR}/render.cpp
  ${PROJ_SRC_DIR}/simulation.cpp
  ${PROJ_SRC_DIR}/threadingpool.cpp)

add_executable(unit_test
  ${SOURCE_FILE}
  test_crystal.cpp
  test_context.cpp
  test_optics.cpp
  test_serialize.cpp
  test_main.cpp)
target_include_directories(unit_test
  PUBLIC ${PROJ_SRC_DIR} ${Boost_INCLUDE_DIRS} "${MODULE_ROOT}/rapidjson/include")
target_link_libraries(unit_test
  PUBLIC ${Boost_LIBRARIES} gtest)


add_executable(RayTracingTest test_ray_tracing_main.cpp ${SOURCE_FILE})
target_include_directories(RayTracingTest
    PUBLIC ${PROJ_SRC_DIR} ${Boost_INCLUDE_DIRS} "${MODULE_ROOT}/rapidjson/include")
target_link_libraries(RayTracingTest
    PUBLIC ${CMAKE_THREAD_LIBS_INIT} ${Boost_LIBRARIES})


add_test(NAME "IceHaloUnitTest" COMMAND unit_test
        "${PROJ_ROOT}/test/config_01.json"
        "${CMAKE_CURRENT_BINARY_DIR}")
add_test(NAME "IceHaloRayTracingTest" 
         COMMAND python3 ${PROJECT_SOURCE_DIR}/test/run_ray_tracing_test.py
          --exe $<TARGET_FILE:RayTracingTest>
          --config ${PROJECT_SOURCE_DIR}/test/config_01.json
          --ref ${PROJECT_SOURCE_DIR}/test/ray_tracing_result.log)
