cmake_minimum_required(VERSION 2.8.3)
project(stanley_control)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
#find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs InsMsg PlanningMsg RadarMsg vehicle_logging VehicleMsg CameraMsg)
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs autoware_planning_msgs autoware_control_msgs )
#message("Lon_Control dependece package path:"  ${catkin_INCLUDE_DIRS})

find_package(Eigen3 REQUIRED)

find_package(Boost REQUIRED COMPONENTS)

##find_package(catkin REQUIRED COMPONENTS
##	  image_transport
##	)
##double topic

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    autoware_control_msgs
    autoware_planning_msgs
    geometry_msgs
    sensor_msgs
    std_msgs
    tf2
    tf2_eigen
    tf2_ros
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

set(CMAKE_CXX_FLAGS "-Wall -Wno-deprecated")

set(SOURCE_FILES main.cpp
                  src/stanley_control.cpp
	                src/stanley_control.h
                 )

add_executable(Lontitude_node ${SOURCE_FILES})

add_dependencies(Lontitude_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(Lontitude_node 
                      ${catkin_LIBRARIES}
                      ${Boost_LIBRARIES})