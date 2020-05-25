#ifndef _RADAR_EXTERNAL_PARAMETER_ADJUSTMENT_SERVER_
#define _RADAR_EXTERNAL_PARAMETER_ADJUSTMENT_SERVER_

// ros
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

// message
#include "radar_msgs/RadarObject.h"
#include "radar_msgs/RadarObjectList.h"

// point cloud tools
#include "radar_external_parameter_adjustment/point_cloud_tools/show_bounding_box.h"
#include "radar_external_parameter_adjustment/point_cloud_tools/eigen_transformation.h"

// project
#include "radar_external_parameter_adjustment/parameterConfig.h"

namespace global {
namespace perception {
namespace radar {

class Radar_External_Parameter_Adjustment
{
    public:
        // constructor
        Radar_External_Parameter_Adjustment(dynamic_reconfigure::Server<radar_external_parameter_adjustment::parameterConfig>& server,
        ros::NodeHandle& public_node, ros::NodeHandle& private_node);
        // destructor
        ~Radar_External_Parameter_Adjustment(){ };
    protected:
        // variable
        dynamic_reconfigure::Server<radar_external_parameter_adjustment::parameterConfig>::CallbackType f_;
        std::string original_radar_topic_, transformation_radar_bbox_topic_;
        double translation_x_, translation_y_, translation_z_, rotation_x_, rotation_y_, rotation_z_;
        ros::Publisher pub_radar_bounding_box_;
        Eigen::Matrix4f transformation_matrix_;
        // function
        void callback(radar_external_parameter_adjustment::parameterConfig& config);
        void getParameter(ros::NodeHandle& public_node, ros::NodeHandle& private_node);
        void radarCallback(const radar_msgs::RadarObjectList& radar_object_list);
        // void drawRadarBoundingbox(const radar_msgs::RadarObject& radar_object_message);
        ros::Subscriber sub_radar_;
};

}   // radar
}   // perception
}   // global
#endif