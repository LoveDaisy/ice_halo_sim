set(TEST_SOURCE_FILE
        main.cpp
        test_crystal.cpp)
set(SOURCE_FILE
        ${IceHalo_SOURCE_DIR}/src/crystal.cpp
        ${IceHalo_SOURCE_DIR}/src/mymath.cpp)

add_executable(test_crystal ${TEST_SOURCE_FILE} ${SOURCE_FILE})

target_link_libraries(test_crystal
    gtest
)
