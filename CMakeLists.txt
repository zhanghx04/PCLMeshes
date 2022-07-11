cmake_minimum_required(VERSION 3.2)
project(pc2mesh)

find_package(Qt5 COMPONENTS Core Widgets REQUIRED)
find_package(PCL 1.3 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})
add_executable(main main.cpp)

target_link_libraries(main ${PCL_LIBRARIES})
