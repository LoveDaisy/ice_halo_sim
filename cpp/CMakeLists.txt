cmake_minimum_required(VERSION 2.8.2)

project(IceHalo)

# set(CMAKE_INCLUDE_CURRENT_DIR ON)

add_subdirectory(src bin)
if(NEED_TEST)
    add_subdirectory(test)
endif()

