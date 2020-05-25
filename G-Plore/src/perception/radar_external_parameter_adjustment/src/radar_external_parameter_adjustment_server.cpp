#include "radar_external_parameter_adjustment/radar_external_parameter_adjustment_server.h"

global::perception::radar::Radar_External_Parameter_Adjustment::Radar_External_Parameter_Adjustment(
dynamic_reconfigure::Server<radar_external_parameter_adjustment::parameterConfig>& server,
ros::NodeHandle& public_node, ros::NodeHandle& private_node)
{
    transformation_matrix_ << 1,0,0,0,
                                                            0,1,0,0,
                                                            0,0,1,0,
                                                            0,0,0,1;
    std::cout << "transformation_matrix_ = " << transformation_matrix_ << std::endl;
    getParameter(public_node, private_node);
    f_ = boost::bind(&global::perception::radar::Radar_External_Parameter_Adjustment::callback, this, _1);
    server.setCallback(f_);
}

void global::perception::radar::Radar_External_Parameter_Adjustment::callback(
radar_external_parameter_adjustment::parameterConfig& config)
{
    std::cout << "original radar topic: " << original_radar_topic_ << std::endl;
    std::cout << "translation x is: " << config.translation_x << std::endl;
    std::cout << "translation y is: " << config.translation_y << std::endl;
    std::cout << "translation z is: " << config.translation_z << std::endl;
    std::cout << "rotation x is: " << config.rotation_x << std::endl;
    std::cout << "rotation y is: " << config.rotation_y << std::endl;
    std::cout << "rotation z is: " << config.rotation_z << std::endl;
    translation_x_ = config.translation_x;
    translation_y_ = config.translation_y;
    translation_z_ = config.translation_z;
    rotation_x_ = config.rotation_x;
    rotation_y_ = config.rotation_y;
    rotation_z_ = config.rotation_z;
    global::perception::radar::eigenTransformationAffine3f(transformation_matrix_, translation_x_, translation_y_, translation_z_, rotation_x_, rotation_y_, rotation_z_);
}

void global::perception::radar::Radar_External_Parameter_Adjustment::getParameter(ros::NodeHandle& public_node, ros::NodeHandle& private_node)
{
    private_node.param("original_radar_topic", original_radar_topic_,  original_radar_topic_);
    private_node.param("transformation_radar_bbox_topic", transformation_radar_bbox_topic_, transformation_radar_bbox_topic_);
    sub_radar_ = public_node.subscribe(original_radar_topic_, 2, &global::perception::radar::Radar_External_Parameter_Adjustment::radarCallback, this);
    pub_radar_bounding_box_ = public_node.advertise<sensor_msgs::PointCloud2>(transformation_radar_bbox_topic_, 100);
}

void global::perception::radar::Radar_External_Parameter_Adjustment::radarCallback(const radar_msgs::RadarObjectList& radar_object_list)
{
    pcl::PointCloud<pcl::PointXYZRGB> bounding_box;
    uint8_t obj_num = radar_object_list.ObjectList.size();
    for (size_t i=0; i<obj_num; i++)
    {
        Eigen::Vector4f dcube[8];
        pcl::PointXYZ points[8];
        for (int j=0;j<8; j++)
        {
            dcube[j](0) = radar_object_list.ObjectList[i].Rel_Bbox[j].x;
            dcube[j](1) = radar_object_list.ObjectList[i].Rel_Bbox[j].y;
            dcube[j](2) = radar_object_list.ObjectList[i].Rel_Bbox[j].z;
            dcube[j](3) = 1;
            // std::cout << "dcube[" << j << "] = " << dcube[j] << std::endl;
            dcube[j] = transformation_matrix_ * dcube[j];
            points[j].x = dcube[j](0);
            points[j].y = dcube[j](1);
            points[j].z = dcube[j](2);
        }
        global::perception::radar::draw3DBoundingBox(points, bounding_box, 0, 0, 255);
    }
    sensor_msgs::PointCloud2 pointcloud2;
    pcl::toROSMsg(bounding_box, pointcloud2);
    pointcloud2.header.stamp = radar_object_list.header.stamp;
    pointcloud2.header.frame_id = "perception";
    pub_radar_bounding_box_.publish(pointcloud2);
    std::cout << "original radar topic is: " << original_radar_topic_ << std::endl;
    std::cout << "transformation matrix is: " << std::endl << transformation_matrix_ << std::endl ;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "radar_external_parameter_adjustment_node" );
    ros::NodeHandle public_node, private_node("~");
    dynamic_reconfigure::Server<radar_external_parameter_adjustment::parameterConfig> server;
    global::perception::radar::Radar_External_Parameter_Adjustment radar_external_parameter_adjustment(server, public_node, private_node);
    ros::Rate loop_rate(20);
    while (ros::ok())
    {
        ros::spinOnce();
        usleep(1);
    }
    return 0;
}