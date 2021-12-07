cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(pcd_tutorials)

find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (paper paper.cpp)
target_link_libraries (paper ${PCL_LIBRARIES})