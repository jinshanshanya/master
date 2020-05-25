cmake_minimum_required(VERSION 2.8)
project(ars408)

# add_compile_options(-std=c++11)
if (CMAKE_COMPILER_IS_GUNCXX)
    set(CMAKE_CXX_FLAGS"-std=c++11 ${CMAKE_CXX_FLAGS}")
    message(STATUS "CMAKE_CXX_FLAGS is ${CMAKE_CXX_FLAGS}")
endif (CMAKE_COMPILER_IS_GUNCXX)

set(CMAKE_BUILD_TYPE Release)
message(STATUS "CMAKE_BUILD_TYPE is  ${CMAKE_BUILD_TYPE}")

include_directories(
    include
)

set(
    SRC
    src/ars408_60.cpp
    src/ars408_62.cpp
    src/ars408_63.cpp
    src/ars408_64.cpp
    src/ars408_65.cpp
    src/ars408_66.cpp
    src/ars408_67.cpp
)

set(
    INC
    include/perception_radar/ars408/ars408_60.h
    include/perception_radar/ars408/ars408_62.h
    include/perception_radar/ars408/ars408_63.h
    include/perception_radar/ars408/ars408_64.h
    include/perception_radar/ars408/ars408_66.h
    include/perception_radar/ars408/ars408_67.h
)

ADD_LIBRARY(
    ars408
    SHARED
    ${SRC}
    ${INC}
)