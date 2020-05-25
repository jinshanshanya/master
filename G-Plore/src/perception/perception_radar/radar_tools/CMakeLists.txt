cmake_minimum_required(VERSION 2.8)
project(radar_tools)
# add_compile_options(-std=c++11)
if (CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS "-std=c++11" ${CMAKE_CXX_FLAGS})
    message(STATUS "optional: -std=c++11")
endif(CMAKE_COMPILER_IS_GNUCXX)

find_package(Eigen3 REQUIRED)

set(CMAKE_BUILD_TYPE Release)
message(STATUS "CMAKE_BUILD_TYPE is: ${CMAKE_BUILD_TYPE}")

include_directories(
    include
    ${Eigen3_INCLUDE_DIRS}
)

# AUX_SOURCE_DIRECTORY(
#     src
#     SRC
# )
set(
    SRC
    src/radar_tools.cpp
    src/anyarg.cpp
)
message(STATUS "SRC is: ${SRC}")

set(
    INC
    include/perception_radar/radar_tools/radar_data.h
    include/perception_radar/radar_tools/radar_tools.h
    include/perception_radar/radar_tools/anyarg.h
)
message(STATUS "INC is: ${INC}")

ADD_LIBRARY(
    radar_tools
    SHARED
    ${SRC}
    ${INC}
)
TARGET_LINK_LIBRARIES(
    radar_tools
    ${Eigen3_LIBS}
)