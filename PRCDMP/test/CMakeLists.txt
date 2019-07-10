cmake_minimum_required(VERSION 3.8)
project(RLF_test)

set(CMAKE_CXX_STANDARD 11)
set(SOURCE_FILES generaltest.cpp)

find_package(OpenMP REQUIRED)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

add_executable(RLF_test ${SOURCE_FILES})
target_link_libraries (RLF_test OpenMP::OpenMP_CXX "${OpenMP_CXX_FLAGS}")

