file(GLOB SRCS *.cpp)

add_executable(test_crystal ${SRCS})

target_link_libraries(test_crystal
    libgtest
    libgmock
)

# install(TARGETS test_crystal DESTINATION bin)