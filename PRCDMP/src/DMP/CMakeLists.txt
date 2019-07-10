cmake_minimum_required(VERSION 3.8)
project(DMP)

set(CMAKE_CXX_STANDARD 11)

set(    CMAKE_RUNTIME_OUTPUT_DIRECTORY
        ${CMAKE_HOME_DIRECTORY}/RLF/bin
    )

set(    CMAKE_LIBRARY_OUTPUT_DIRECTORY
        ${CMAKE_HOME_DIRECTORY}/lib
    )

find_package (Eigen3 3.3 REQUIRED NO_MODULE)


set(SOURCE_FILES
        DMP.cpp
        CanonicalSystem.cpp
        DiscreteDMP.cpp)

add_library(DMP ${SOURCE_FILES})
target_link_libraries(DMP Eigen3::Eigen)
