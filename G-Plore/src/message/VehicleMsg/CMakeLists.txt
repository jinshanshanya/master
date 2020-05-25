cmake_minimum_required(VERSION 2.8.3)
project(VehicleMsg)

find_package(catkin REQUIRED COMPONENTS roscpp rospy message_generation std_msgs)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
	vehcile_cmd.msg
  cdm_cmd.msg
  adm_cmd.msg
  adm_lat.msg
	vehicle_status.msg
	vehicle_upload.msg
	vehicle_download.msg
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


