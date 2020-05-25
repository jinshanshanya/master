cmake_minimum_required(VERSION 2.8.3)
project(IFSimu)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs InsMsg)
message("IFSimu dependece package path:"  ${catkin_INCLUDE_DIRS})

find_package(Boost REQUIRED COMPONENTS)



catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES 
  CATKIN_DEPENDS roscpp rospy std_msgs InsMsg 
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

set(CMAKE_CXX_FLAGS "-Wall -Wno-deprecated")

set(SOURCE_FILES main.cpp
                  src/App.cpp
                  src/Node.cpp
	                src/IFS2000_standard_CAN.cpp 
                  src/UDPSocket.cpp)

add_executable(IFSimu_node ${SOURCE_FILES})

add_dependencies(IFSimu_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(IFSimu_node 
                      ${catkin_LIBRARIES}
                      ${Boost_LIBRARIES})