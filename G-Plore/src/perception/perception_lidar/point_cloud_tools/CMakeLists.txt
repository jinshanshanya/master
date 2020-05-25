cmake_minimum_required(VERSION 2.8)
project(point_cloud_tools)
# add_compile_options(-std=c++11)
if (CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
    message(STATUS "optional: -std=c++11")
endif(CMAKE_COMPILER_IS_GNUCXX)

set(CMAKE_BUILD_TYPE Release)
message(STATUS "CMAKE_BUILD_TYPE is: ${CMAKE_BUILD_TYPE}")

find_package(PCL REQUIRED)

find_package(Eigen3 REQUIRED)

find_package(OpenCV REQUIRED)

include_directories(
    include
    ${PCL_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

# AUX_SOURCE_DIRECTORY(src DIR_SRC)
set(
    SRC
    src/point_cloud_transformation.cpp
)
message(STATUS "SRC is: ${SRC}")

set(
    INC
    include/perception_lidar/point_cloud_tools/base_data.h
    include/perception_lidar/point_cloud_tools/feature_extraction.h
    include/perception_lidar/point_cloud_tools/ground_cut.h
    include/perception_lidar/point_cloud_tools/point_cloud_transformation.h
    include/perception_lidar/point_cloud_tools/show_bounding_box.h
)
message(STATUS "INC is: ${INC}")

ADD_LIBRARY(
    point_cloud_tools_library
    SHARED
    ${SRC}
    ${INC}
)
TARGET_LINK_LIBRARIES(
    point_cloud_tools_library
    ${PCL_LIBRARIES}
    ${Eigen3_LIBS}
    ${OpenCV_LIBS}
)

 # ADD_EXECUTABLE(
 #     point_cloud_tools_test1
 #    src/test1.cpp
#  )
 # TARGET_LINK_LIBRARIES(
 #     point_cloud_tools_test1
 #     ${Eigen3_INCLUDE_DIRS}
 # )