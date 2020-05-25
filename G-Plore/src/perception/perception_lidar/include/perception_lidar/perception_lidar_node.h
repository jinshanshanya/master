#ifndef _PERCEPTION_LIDAR_NODE_
#define _PERCEPTION_LIDAR_NODE_

bool print_message = true;

#define FACTOR_Z 10.0

// ros
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <ros/subscriber.h>
#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/Marker.h>

// pcl
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// thread
#include <thread>
#include <mutex>
#include <queue>

// point_cloud_tools
#include "perception_lidar/point_cloud_tools/point_cloud_transformation.h"
#include "perception_lidar/point_cloud_tools/feature_extraction.h"
#include "perception_lidar/point_cloud_tools/show_bounding_box.h"
#include "perception_lidar/point_cloud_tools/ground_cut.h"

// obstacle_tracking
#include "perception_lidar/obstacle_tracking/obstacle_tracking.h"

// defined message
#include "lidar_msgs/LidarObjectList.h"
#include "InsMsg/ins_p2.h"

namespace perception
{
    class Perception_Lidar_Node
    {
        public:
            // function
            Perception_Lidar_Node(ros::NodeHandle public_node,  ros::NodeHandle private_node,
            double cost_threshold, double memory_time);
            ~Perception_Lidar_Node();
        protected:
            // function
            void callbackLidar1(const sensor_msgs::PointCloud2::ConstPtr msgs);
            void callbackLidar2(const sensor_msgs::PointCloud2::ConstPtr msgs);
            void callbackLidar3(const sensor_msgs::PointCloud2::ConstPtr msgs);
            void getParameter(ros::NodeHandle public_node, ros::NodeHandle private_node);
            void lidarPerception();
            // void lidarPreprocessing(pcl::PointCloud<pcl::PointXYZI>& lidar, pcl::PointCloud<pcl::PointXYZI>& lidar_groundless, Eigen::Matrix4f& transform_matrix);
            void lidarPreprocessing(pcl::PointCloud<pcl::PointXYZI>& lidar,  Eigen::Matrix4f& transform_matrix);
            void drawCubes(std::vector<perception::Obstacle_Feature_Point_Cloud<pcl::PointXYZI, float>>&  obstacles_feature, ros::Time& time_stamp);
            void drawDCubes(std::vector<perception::Obstacle_Feature_Point_Cloud<pcl::PointXYZI, float>>&  obstacles_feature, ros::Time& time_stamp);
            void showCutCarCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr lidar_ptr, ros::Time& time_stamp);
            void drawText(Point3D<float> position, uint32_t id, const Point3D<float>& velocity);
            void drawObstaclesInfo();
            void drawTrackingCubes();
            void drawTrackingObstacles();
            void obstacleMarkerInitialization();
            void lidarTracking();
            void publishLidarMessage(ros::Time& time_stamp);
            void showLidarCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr lidar_ptr, ros::Time& time_stamp);
            void showFrontBackLidar(pcl::PointCloud<pcl::PointXYZI>& lidar1, pcl::PointCloud<pcl::PointXYZI>& lidar2,
            pcl::PointCloud<pcl::PointXYZI>& lidar3, ros::Time& time_stamp);
            // variable
            std::mutex mutex_lidar1_, mutex_lidar2_, mutex_lidar3_; 
            std::queue<sensor_msgs::PointCloud2> queue_lidar1_;
            std::queue<sensor_msgs::PointCloud2> queue_lidar2_;
            std::queue<sensor_msgs::PointCloud2> queue_lidar3_;
            std::thread lidar_perception_thread1_, lidar_perception_thread2_, lidar_perception_thread3_;
            std::thread lidar_perception_thread4_, lidar_perception_thread5_;
            std::thread lidar_tracking_thread_;
            Eigen::Matrix4f transform_matrix_lidar1_, transform_matrix_lidar2_, transform_matrix_lidar3_;
            // **** subscriber **** //
            ros::Subscriber sub_lidar1_, sub_lidar2_, sub_lidar3_;
            // ******************* //
            // ***** publisher ***** //
            ros::Publisher pub_lidar_, pub_lidar_cutcar_, pub_cubes_, pub_dcubes_;
            ros::Publisher pub_obstacles_info_, pub_tracking_cubes_;
            ros::Publisher pub_lidar_message_;
            ros::Publisher pub_lidar_front_, pub_lidar_back_;
            // ******************** //
            // **** id iterator **** //
            uint32_t iterator_id_ = (uint32_t) 0;
            // ******************** //
            // **** obstacle feature tracking **** //
            boost::shared_ptr<std::vector<perception::Obstacle_Feature_Tracking<float>>> obstacles_featue_tracking_;
            // ********************************* //
            perception::Obstacle_Tracking<pcl::PointXYZI, float> obstacle_tracking_;
            visualization_msgs::Marker obstacle_marker_;
            std::queue<std::vector<perception::Obstacle_Feature_Point_Cloud<pcl::PointXYZI, float>>> queue_obstacles_feature_;
            std::queue<ros::Time> queue_time_stamp_;
            std::mutex mutex_queue_obstacles_;
    };
}

#endif

