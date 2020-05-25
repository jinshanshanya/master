cmake_minimum_required(VERSION 2.8.3)
project(vehicle_logging)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs PlanningMsg)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
	logging_msg.msg
)

generate_messages(DEPENDENCIES std_msgs PlanningMsg)


catkin_package(
    DEPENDS
    CATKIN_DEPENDS message_runtime
)
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


