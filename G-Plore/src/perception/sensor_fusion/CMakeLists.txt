cmake_minimum_required(VERSION 2.8.8)
project(sensorfusion)

add_compile_options(-std=c++11)
add_definitions(-std=c++11)
set(CMAKE_BUILD_TYPE "Debug")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror -Wl,--as-needed")
## -pg : produces an executable file for gprof profiling. gprof is a tool for performance analysis of C++ programs under Linux
## -g : can generate executable files for gdb debugging, and the executable file contains the information needed for 
## debugging by the gdb debugger.

############### catkin dependencies ######################
set(${PROJECT_NAME}_CATKIN_DEPS
    roscpp
    roslaunch 
    rostest
    sensor_msgs
    tf
    tf_conversions
    )

find_package(catkin REQUIRED COMPONENTS
             ${${PROJECT_NAME}_CATKIN_DEPS}
             std_msgs
             common_msgs
             lidar_msgs
             fusion_msgs
             radar_msgs
             )
message("FOUND THE LidarMsg Packge:" ${LIDARMSG_FOUND})

############### glog 0.3.3 ######################
set(GLOG_INCLUDE_DIR "/usr/local/include/")
execute_process(COMMAND arch OUTPUT_VARIABLE ARCH)
message("arch is :" ${ARCH})

if(EXISTS /usr/local/lib/libglog.so)
set(GLOG_LIBRARY "/usr/local/lib/libglog.so")
else(EXISTS /usr/local/lib/libglog.so)
if(ARCH MATCHES "x86_64")
set(GLOG_LIBRARY "/usr/lib/x86_64-linux-gnu/libglog.so")
else(ARCH MATCHES "x86_64")
set(GLOG_LIBRARY "/usr/lib/aarch64-linux-gnu/libglog.so")
endif(ARCH MATCHES "x86_64")
endif(EXISTS /usr/local/lib/libglog.so)

############### boost 1.54.0 ######################
find_package(Boost REQUIRED)

############### protobuf 3.3 ######################
include(FindProtobuf)
find_package(Protobuf 3.3 REQUIRED EXACT)
message("Protobuf Include path: " ${PROTOBUF_INCLUDE_DIR})
message("Protobuf libraries: " ${PROTOBUF_LIBRARIES})
message("Protobuf library: " ${PROTOBUF_LIBRARY})

############### gflags 2.0 ######################
set(GFLAGS_INCLUDE_DIR "/usr/local/include/")
set(GFLAGS_LIBRARY "/usr/local/lib/libgflags.so")

############### eigen 3 ######################
#find_package (Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Eigen3 REQUIRED)
message("Eigen include path : " ${EIGEN3_INCLUDE_DIR})
#include_directories(${EIGEN3_INCLUDE_DIR})
#INCLUDE_DIRECTORIES(/usr/include/eigen3)

############### include directories ######################
include_directories(include
                    ${Boost_INCLUDE_DIR}
                    ${catkin_INCLUDE_DIRS}
                    ${PROTOBUF_INCLUDE_DIR}
                    ${GLOG_INCLUDE_DIR}
                    ${GFLAGS_INCLUDE_DIR}
                    ${EIGEN3_INCLUDE_DIR})

catkin_package(CATKIN_DEPENDS ${${PROJECT_NAME}_CATKIN_DEPS})

############### define third party libs ######################
set( THIRD_PARTY_LIBS
        ${catkin_LIBRARIES}
        ${Boost_LIBRARIES}
        ${PROTOBUF_LIBRARY}
        ${GLOG_LIBRARY}
        ${GFLAGS_LIBRARY}
        yaml-cpp)


############### subfolders ######################
add_subdirectory( src )
