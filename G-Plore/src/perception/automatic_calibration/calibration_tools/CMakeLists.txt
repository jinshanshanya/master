cmake_minimum_required(VERSION 2.8)
project(calibration_tools)

# add_compille_options(-std=c++11)
if (CMAKE_COMPILER_IS_GUNCXX)
    set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_COMPILER_IS_GUNCXX}")
    message(STATUS "CMAKE_CXX_FLAGS is: ${CMAKE_CXX_FLAGS}")
endif (CMAKE_COMPILER_IS_GUNCXX)

set(CMAKE_BUILD_TYPE Release)
message(STATUS "CMAKE_BUILD_TYPE is: ${CMAKE_BUILD_TYPE}")

find_package(PCL REQUIRED)

include_directories(
    include
    ${PCL_INCLUDE_DIRS}
)

set(
    SRC
    src/closest_point_matching.cpp
)

set(
    INC
    include/automatic_calibration/calibration_tools/closest_point_matching.h
)

ADD_LIBRARY(
    calibration_tools
    SHARED
    ${SRC}
    ${INC}
)
TARGET_LINK_LIBRARIES(
    calibration_tools
    ${PCL_LIBRARIES}
)
