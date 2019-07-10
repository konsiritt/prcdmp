cmake_minimum_required(VERSION 3.8)
project(UTILS)

set(CMAKE_CXX_STANDARD 11)

set(SOURCE_FILES trajectoryUtils.cpp Config.cpp common.cpp)

add_library(UTILS ${SOURCE_FILES})

target_link_libraries (UTILS)
