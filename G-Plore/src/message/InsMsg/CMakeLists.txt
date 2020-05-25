cmake_minimum_required(VERSION 2.8.3)
project(InsMsg)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
	ins_p2.msg
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


