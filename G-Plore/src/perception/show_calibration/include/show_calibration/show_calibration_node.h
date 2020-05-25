#ifndef _SHOW_CALIBRATION_NODE_
#define _SHOW_CALIBRATION_NODE_

// ros
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

// thread
#include <thread>
#include <mutex>
#include <queue>

// pcl
#include <pcl/common/transforms.h>

// message
#include "radar_msgs/RadarObjectList.h"
#include "radar_msgs/RadarObjectList.h"

// point_cloud_tools
#include "show_calibration/point_cloud_tools/show_bounding_box.h"

bool print_message = true;

namespace global {
namespace perception {
namespace calibration {

class Show_Calibration_Node
{
    public:
    // constructor
    Show_Calibration_Node(ros::NodeHandle& public_node, ros::NodeHandle& private_node);
    // destructor
    ~Show_Calibration_Node(){};
    protected:
    // variable
    std::mutex mutex_lidar1_, mutex_lidar2_, mutex_lidar3_; 
    std::queue<sensor_msgs::PointCloud2> queue_lidar1_;
    std::queue<sensor_msgs::PointCloud2> queue_lidar2_;
    std::queue<sensor_msgs::PointCloud2> queue_lidar3_;
    std::thread show_calibration_;
    Eigen::Matrix4f transform_matrix_lidar1_, transform_matrix_lidar2_, transform_matrix_lidar3_;
    // **** subscriber **** //
    ros::Subscriber sub_lidar1_, sub_lidar2_, sub_lidar3_;
    ros::Subscriber sub_radar_fusion_;
    // ******************* //
    // ***** publisher ***** //
    ros::Publisher pub_lidar_calibration_;
    ros::Publisher pub_radar_fusion_;
    // ******************* //
    // pcl::PointCloud<pcl::PointXYZI>::Ptr cloud1;
    // function
    void callbackLidar1(const sensor_msgs::PointCloud2::Ptr msgs);
    void callbackLidar2(const sensor_msgs::PointCloud2::Ptr msgs);
    void callbackLidar3(const sensor_msgs::PointCloud2::Ptr msgs);
    void getParameter(ros::NodeHandle& public_node, ros::NodeHandle& private_node);
    void showCalibration();
    void showRadarFusion(const radar_msgs::RadarObjectList::Ptr radar_objectlist);
};

}   // calibration
}   // perception
}   // global


#endif