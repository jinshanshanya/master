//ros
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <sensor_msgs/PointCloud2.h>

//pcl
#include <pcl_conversions/pcl_conversions.h>

// project
#include "external_parameter_adjustment/parameterConfig.h"
#include "external_parameter_adjustment/pointcloud_transformation.h"


namespace perception
{
    class Parameter_Adjustment
    {
        public:
            Parameter_Adjustment(dynamic_reconfigure::Server<external_parameter_adjustment::parameterConfig>& server,
            ros::NodeHandle& public_node, ros::NodeHandle& private_node);
            ~Parameter_Adjustment(){ };
        protected:
            //variable
            dynamic_reconfigure::Server<external_parameter_adjustment::parameterConfig>::CallbackType f_;
            ros::Subscriber sub_lidar;
            ros::Publisher pub_lidar;
            double translation_x_ = 0, translation_y_ = 0, translation_z_ = 0;
            double rotation_x_ = 0, rotation_y_ = 0, rotation_z_ = 0;
            std::string original_lidar_topic, transformation_lidar_topic;
            // function
            void callback(external_parameter_adjustment::parameterConfig& config);
            void callbackLidar(const sensor_msgs::PointCloud2::ConstPtr& msgs);
            void getParameter(ros::NodeHandle& public_node, ros::NodeHandle& private_node);
    };
}