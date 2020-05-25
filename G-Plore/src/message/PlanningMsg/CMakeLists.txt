cmake_minimum_required(VERSION 2.8.3)
project(PlanningMsg)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
	lane_point.msg
	path_data.msg
)

generate_messages(DEPENDENCIES std_msgs)


catkin_package(
    DEPENDS
    CATKIN_DEPENDS message_runtime
)
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


