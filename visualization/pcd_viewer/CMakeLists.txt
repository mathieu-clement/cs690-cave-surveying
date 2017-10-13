cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(view)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (view pcd_viewer.cpp)
target_link_libraries (view ${PCL_LIBRARIES})