
#include <ros/ros.h>
#include "perception_yx/perception_yx.h"
#include "glog/logging.h"
#include "util/log.h"
/** Main entry point. */
int main(int argc, char **argv)
{

  google::InitGoogleLogging(argv[0]);

  ros::init(argc, argv, "perception_yx_node");
  ros::NodeHandle node;
  ros::NodeHandle priv_nh("~");
  AINFO << "***************************Sensor fusion start***************************";
  ADEBUG << "track_ind is matched to measurement_ind for sensor ";
  AWARN << "***************************Sensor fusion start***************************";
  // create perception class
  perception_yx::PerceptionYX yx(node, priv_nh);

   ros::MultiThreadedSpinner s(2);
  // handle callbacks until shut down
   ros::spin(s);
  //ros::spin();

  return 0;
}
