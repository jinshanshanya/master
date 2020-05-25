cmake_minimum_required(VERSION 2.8)
project(point_cloud_tools)
# add_compile_options(-std=c++11)
if (CMAKE_COMPILER_IS_GUNCXX)
    set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
    message(STATUS "optional: -std=c++11")
endif (CMAKE_COMPILER_IS_GUNCXX)

set(CMAKE_BUILD_TYPE Release)
message(STATUS "CMAKE_BUILD_TYPE is ${CMAKE_BUILD_TYPE}")

find_package(PCL REQUIRED)

find_package(Eigen3 REQUIRED)

include_directories(
    include
    ${PCL_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS}
)

set(
    SRC
    src/show_bounding_box.cpp
    src/eigen_transformation.cpp
)

set(
    INC
    include/radar_external_parameter_adjustment/point_cloud_tools/show_bounding_box.h
    include/radar_external_parameter_adjustment/point_cloud_tools/eigen_transformation.h
)

ADD_LIBRARY(
    point_cloud_tools
    SHARED
    ${SRC}
    ${INC}
)
TARGET_LINK_LIBRARIES(
    point_cloud_tools
    ${PCL_LIBRARIES}
    ${Eigen3_LIBS}
)
