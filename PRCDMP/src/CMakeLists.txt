cmake_minimum_required(VERSION 3.8)

set(CMAKE_CXX_STANDARD 11)

add_subdirectory(DMP)
add_subdirectory(UTILS)
set(SOURCE_FILES1  record.cpp interface.cpp common.cpp)
set(SOURCE_FILES2  rundmp.cpp interface.cpp common.cpp)
set(SOURCE_FILES3  runtrajectory.cpp interface.cpp common.cpp)
find_package(Franka REQUIRED)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "-Wall -Wextra -std=gnu++11")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(record ${SOURCE_FILES1})
add_executable(rundmp ${SOURCE_FILES2})
add_executable(runtraj ${SOURCE_FILES3})
target_link_libraries (record franka DMP UTILS jsoncpp)
target_link_libraries (rundmp franka DMP UTILS jsoncpp)
target_link_libraries (runtraj franka DMP UTILS jsoncpp)

install(TARGETS record rundmp DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
