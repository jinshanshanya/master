#include "external_parameter_adjustment/parameter_adjustment_server.h"

perception::Parameter_Adjustment::Parameter_Adjustment(dynamic_reconfigure::Server<external_parameter_adjustment::parameterConfig>& server,
ros::NodeHandle& public_node, ros::NodeHandle& private_node)
{
    f_ = boost::bind(&perception::Parameter_Adjustment::callback, this, _1);
    server.setCallback(f_);
    getParameter(public_node, private_node);
}

void perception::Parameter_Adjustment::callback(external_parameter_adjustment::parameterConfig& config)
{
    std::cout << original_lidar_topic << std::endl << transformation_lidar_topic <<std::endl;
    std::cout << "translation x is: " << config.translation_x << std::endl;
    std::cout << "translation y is: " << config.translation_y << std::endl;
    std::cout << "translation z is: " << config.translation_z << std::endl;
    std::cout << "rotation x is: " << config.rotation_x << std::endl;
    std::cout << "rotation y is: " << config.rotation_y << std::endl;
    std::cout << "rotation z is: " << config.rotation_z << std::endl << std::endl;
    translation_x_ = config.translation_x;
    translation_y_ = config.translation_y;
    translation_z_ = config.translation_z;
    rotation_x_ = config.rotation_x;
    rotation_y_ = config.rotation_y;
    rotation_z_ = config.rotation_z;
}

void perception::Parameter_Adjustment::getParameter(ros::NodeHandle& public_node,
ros::NodeHandle& private_node)
{
    private_node.param("original_lidar_topic", original_lidar_topic, original_lidar_topic);
    sub_lidar = public_node.subscribe(original_lidar_topic, 2, &perception::Parameter_Adjustment::callbackLidar, this);
    private_node.param("transformation_lidar_topic", transformation_lidar_topic, transformation_lidar_topic);
    pub_lidar = public_node.advertise<sensor_msgs::PointCloud2>(transformation_lidar_topic, 10);
}

void perception::Parameter_Adjustment::callbackLidar(const sensor_msgs::PointCloud2::ConstPtr& msgs)
{
    pcl::PointCloud<pcl::PointXYZI> lidar;
    pcl::fromROSMsg(*msgs, lidar);
    Eigen::Matrix4f transformation_matrix;
    perception::pointcloudTransformationAffine3f(lidar, transformation_matrix, translation_x_, translation_y_, translation_z_,
    rotation_x_, rotation_y_, rotation_z_);
    // perception::pointcloudTransformationAngleAxisTranslation3f(lidar, transformation_matrix, translation_x_, translation_y_, translation_z_,
    // rotation_x_, rotation_y_, rotation_z_);
    // perception::pointcloudTransformation(lidar, transformation_matrix, translation_x_, translation_y_, translation_z_,
    // rotation_x_, rotation_y_, rotation_z_);
    sensor_msgs::PointCloud2 lidar_msg;
    pcl::toROSMsg(lidar, lidar_msg);
    lidar_msg.header.stamp = ros::Time::now();
    lidar_msg.header.frame_id = "perception";
    pub_lidar.publish(lidar_msg);
    std::cout << original_lidar_topic << std::endl << transformation_lidar_topic <<std::endl;
    std::cout << "transformation marix is: " << std::endl << transformation_matrix << std::endl << std::endl;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "parameter_adjustment_server");
    ros::NodeHandle public_node, private_node("~");
    dynamic_reconfigure::Server<external_parameter_adjustment::parameterConfig> server;
    perception::Parameter_Adjustment parameter_adjustment(server, public_node, private_node);
    ros::spin();
    return 0;
}