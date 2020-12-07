#include "ros/ros.h"
#include "src/stanley_control.h"

int main(int argc, char **argv){
    ros::init(argc, argv, "Stanly_pure");
    ros::NodeHandle nh;
    ros::NodeHandle pri_nh("~");
    Stanley_control rp(nh, pri_nh);

    ros::spin();
    return 0;
}