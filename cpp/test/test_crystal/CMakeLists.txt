set(SOURCE_FILE
        main.cpp
        test_crystal.cpp)

add_executable(test_crystal ${SOURCE_FILE})

target_link_libraries(test_crystal
    libgtest
    libgmock
)

# install(TARGETS test_crystal DESTINATION bin)