cmake_minimum_required(VERSION 2.8.3)
project(Car)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
#find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs InsMsg PlanningMsg RadarMsg vehicle_logging VehicleMsg CameraMsg)
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs VehicleMsg )
message("Car dependece package path:"  ${catkin_INCLUDE_DIRS})

find_package(Boost REQUIRED COMPONENTS)



catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES 
  #CATKIN_DEPENDS roscpp rospy std_msgs InsMsg PlanningMsg RadarMsg vehicle_logging VehicleMsg CameraMsg
  CATKIN_DEPENDS roscpp rospy std_msgs VehicleMsg 
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
	                src/HaiLuo_extend_CAN_v1.cpp
	  # src/HaiLuo_extend_CAN_v1.h
    #src/hailuo_extend.h
    #src/hailuo_extend.cpp
    #src/hailuo_extend.h
                  src/hai_luo_extev2.cpp
                  src/hai_luo_extev2.cpp
    #src/hai_luo_extev2.h
                  src/pved_cls.cpp
    #src/pved_cls.h
                  src/hai_luo_extend_new.cpp 
    #src/hai_luo_extend_new.h 
                  src/UDPSocket.cpp)

add_executable(Car_node ${SOURCE_FILES})

add_dependencies(Car_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(Car_node 
                      ${catkin_LIBRARIES}
                      ${Boost_LIBRARIES})