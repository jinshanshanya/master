#include "ros/ros.h"
#include "src/Lon_Control.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "ros_Lontitued");

  ros::NodeHandle nh;
  ros::NodeHandle pri_nh("~");
  lontitued_Control rp(nh, pri_nh);

  ros::MultiThreadedSpinner spinner(2);

  ros::spin(spinner);
  return 0;
}