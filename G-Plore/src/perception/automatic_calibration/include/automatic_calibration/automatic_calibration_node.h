#ifndef _AUTOMATIC_CALIBRATION_NODE_
#define _AUTOMATIC_CALIBRATION_NODE_

#include "automatic_calibration/calibration_tools/closest_point_matching.h"

// ros
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

// thread
#include <thread>
#include <mutex>
#include <queue>

bool print_message = true;

namespace global {
namespace perception {
namespace calibration {

class Automatic_Calibration_Node
{
    public:
    // constructor
    Automatic_Calibration_Node(ros::NodeHandle& public_node, ros::NodeHandle& private_node);
    // destructor
    ~Automatic_Calibration_Node(){};
    protected:
    // variable
    std::mutex mutex_lidar1_, mutex_lidar2_, mutex_lidar3_; 
    std::queue<sensor_msgs::PointCloud2> queue_lidar1_;
    std::queue<sensor_msgs::PointCloud2> queue_lidar2_;
    std::queue<sensor_msgs::PointCloud2> queue_lidar3_;
    std::thread automatic_calibration_;
    Eigen::Matrix4f transform_matrix_lidar1_, transform_matrix_lidar2_, transform_matrix_lidar3_;
    // **** subscriber **** //
    ros::Subscriber sub_lidar1_, sub_lidar2_, sub_lidar3_;
    // ******************* //
    // ***** publisher ***** //
    ros::Publisher pub_lidar_calibration_;
    // ******************* //
    // pcl::PointCloud<pcl::PointXYZI>::Ptr cloud1;
    // function
    void callbackLidar1(const sensor_msgs::PointCloud2::Ptr msgs);
    void callbackLidar2(const sensor_msgs::PointCloud2::Ptr msgs);
    void callbackLidar3(const sensor_msgs::PointCloud2::Ptr msgs);
    void getParameter(ros::NodeHandle& public_node, ros::NodeHandle& private_node);
    void automaticCalibration();
};

}   // calibration
}   // perception
}   // global


#endif