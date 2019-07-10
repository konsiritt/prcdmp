cmake_minimum_required(VERSION 3.8)
project(PRCDMP)

set(CMAKE_CXX_STANDARD 11)

set(    CMAKE_RUNTIME_OUTPUT_DIRECTORY
        ${CMAKE_HOME_DIRECTORY}/PRCDMP/bin
    )

set(    CMAKE_LIBRARY_OUTPUT_DIRECTORY
        ${CMAKE_HOME_DIRECTORY}/lib
    )


INCLUDE_DIRECTORIES(PRCDMP/include)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

add_subdirectory(PRCDMP/src)
add_subdirectory(PRCDMP/test)
