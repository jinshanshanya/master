cmake_minimum_required(VERSION 2.8)
project(obstacle_tracking)
# add_compile_options(-std=c++11)
if (CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
    message(STATUS "optional: -std=c++11")
endif(CMAKE_COMPILER_IS_GNUCXX)

find_package(PCL REQUIRED)

set(CMAKE_BUILD_TYPE Release)
message(STATUS "CMAKE_BUILD_TYPE is: ${CMAKE_BUILD_TYPE}")

include_directories(
    include
    ../point_cloud_tools/include
    ${PCL_INCLUDE_DIRS}
)

AUX_SOURCE_DIRECTORY(src DIR_SRC)
message(STATUS "DIR_SRC is: ${DIR_SRC}")

ADD_LIBRARY(
    obstacle_tracking
    SHARED
    ${DIR_SRC}
)
TARGET_LINK_LIBRARIES(
    obstacle_tracking
    ${PCL_LIBRARIES}
)
