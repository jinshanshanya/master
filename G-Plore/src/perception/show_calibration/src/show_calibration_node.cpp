#include "show_calibration/show_calibration_node.h"

global::perception::calibration::Show_Calibration_Node::Show_Calibration_Node(
ros::NodeHandle& public_node, ros::NodeHandle& private_node)
{
    if (print_message)
        std::cout << "Constructor Show_Calibration_Node  start" << std::endl;
    getParameter(public_node, private_node);
    pub_lidar_calibration_ = public_node.advertise<sensor_msgs::PointCloud2>("/perception/lidar/showcalibration", 100);
    pub_radar_fusion_ = public_node.advertise<sensor_msgs::PointCloud2>("perception/radar/showcalibration", 100);
    show_calibration_ = std::thread(&global::perception::calibration::Show_Calibration_Node::showCalibration, this);
    if (print_message)
        std::cout << "Constructor Show_Calibration_Node  end" << std::endl;
}

void global::perception::calibration::Show_Calibration_Node::getParameter(
ros::NodeHandle& public_node, ros::NodeHandle& private_node)
{
    std::string topic_lidar1 = "/ns1/rslidar_points", topic_lidar2 = "/ns2/rslidar_points";
    std::string topic_lidar3 = "/lslidar_point_cloud";
    private_node.param("topic_lidar1", topic_lidar1, topic_lidar1);
    private_node.param("topic_lidar2", topic_lidar2, topic_lidar2);
    private_node.param("topic_lidar3", topic_lidar3, topic_lidar3);
    sub_lidar1_ = public_node.subscribe(topic_lidar1, 2, &global::perception::calibration::Show_Calibration_Node::callbackLidar1, this);
    sub_lidar2_ = public_node.subscribe(topic_lidar2, 2, &global::perception::calibration::Show_Calibration_Node::callbackLidar2, this);
    sub_lidar3_ = public_node.subscribe(topic_lidar3, 2, &global::perception::calibration::Show_Calibration_Node::callbackLidar3, this);
    std::vector<float> transform_matrix_lidar1, transform_matrix_lidar2, transform_matrix_lidar3;
    private_node.param("transform_matrix_lidar1", transform_matrix_lidar1, transform_matrix_lidar1);
    private_node.param("transform_matrix_lidar2", transform_matrix_lidar2, transform_matrix_lidar2);
    private_node.param("transform_matrix_lidar3", transform_matrix_lidar3, transform_matrix_lidar3);
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
        {
            transform_matrix_lidar1_(i, j) = transform_matrix_lidar1[4*i+j];
            transform_matrix_lidar2_(i, j) = transform_matrix_lidar2[4*i+j];
            transform_matrix_lidar3_(i, j) = transform_matrix_lidar3[4*i+j];
        }
    std::string topic_radar_fusion = "/perception/radar_fusion/object_list";
    sub_radar_fusion_ = public_node.subscribe(topic_radar_fusion, 2, &global::perception::calibration::Show_Calibration_Node::showRadarFusion, this);
}

void global::perception::calibration::Show_Calibration_Node::callbackLidar1(
const sensor_msgs::PointCloud2::Ptr msgs)
{
    sensor_msgs::PointCloud2 pointcloud2_lidar1 = *msgs;
    mutex_lidar1_.lock();
    if (!queue_lidar1_.empty())
        queue_lidar1_.pop();
    queue_lidar1_.push(pointcloud2_lidar1);
    mutex_lidar1_.unlock();
    // pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    // pcl::fromROSMsg(pointcloud2_lidar1, *cloud);
    // iterativeClosestPoint(cloud1, cloud);
    // cloud1 = cloud;
}

void global::perception::calibration::Show_Calibration_Node::callbackLidar2(
const sensor_msgs::PointCloud2::Ptr msgs)
{
    sensor_msgs::PointCloud2 pointcloud2_lidar2 = *msgs;
    mutex_lidar2_.lock();
    if (!queue_lidar2_.empty())
        queue_lidar2_.pop();
    queue_lidar2_.push(pointcloud2_lidar2);
    mutex_lidar2_.unlock();
}

void global::perception::calibration::Show_Calibration_Node::callbackLidar3(
const sensor_msgs::PointCloud2::Ptr msgs)
{
    sensor_msgs::PointCloud2 pointcloud2_lidar3 = *msgs;
    mutex_lidar3_.lock();
    if (!queue_lidar3_.empty())
        queue_lidar3_.pop();
    queue_lidar3_.push(pointcloud2_lidar3);
    mutex_lidar3_.unlock();
}

void global::perception::calibration::Show_Calibration_Node::showCalibration()
{
    while (true)
    {
        if (queue_lidar1_.empty() || queue_lidar2_.empty() || queue_lidar3_.empty())
        {
            usleep(5000);
        }
        else
        {
            if (print_message)
                std::cout << "showCalibration start" << std::endl;
            pcl::PointCloud<pcl::PointXYZI>::Ptr lidar(new pcl::PointCloud<pcl::PointXYZI>);
            pcl::PointCloud<pcl::PointXYZI>::Ptr lidar1(new pcl::PointCloud<pcl::PointXYZI>);
            pcl::PointCloud<pcl::PointXYZI>::Ptr lidar2(new pcl::PointCloud<pcl::PointXYZI>);
            pcl::PointCloud<pcl::PointXYZI>::Ptr lidar3(new pcl::PointCloud<pcl::PointXYZI>);
            // ***** lidar 1 ***** // 
            mutex_lidar1_.lock(); 
            if (!queue_lidar1_.empty())
            {
                pcl::fromROSMsg(queue_lidar1_.front(), *lidar1);
                queue_lidar1_.pop();
            }
            mutex_lidar1_.unlock();
            // **************** // 
            // ***** lidar 2***** // 
            mutex_lidar2_.lock();
            if (!queue_lidar2_.empty())
            {
                pcl::fromROSMsg(queue_lidar2_.front(), *lidar2);
                queue_lidar2_.pop();
            }
            mutex_lidar2_.unlock();
            // **************** // 
            // ***** lidar 3***** // 
            mutex_lidar3_.lock();
            if (!queue_lidar3_.empty())
            {
                pcl::fromROSMsg(queue_lidar3_.front(), *lidar3);
                queue_lidar3_.pop();
            }
            mutex_lidar3_.unlock();
            // **************** // 
            pcl::transformPointCloud(*lidar1, *lidar1, transform_matrix_lidar1_);
            pcl::transformPointCloud(*lidar2, *lidar2, transform_matrix_lidar2_);
            pcl::transformPointCloud(*lidar3, *lidar3, transform_matrix_lidar3_);
            *lidar = *lidar1 + *lidar2;
            *lidar += *lidar3;
            // *lidar = *lidar1 + *lidar3;
            sensor_msgs::PointCloud2 pointcloud2calibration;
            pcl::toROSMsg(*lidar, pointcloud2calibration);
            pointcloud2calibration.header.stamp = ros::Time::now();
            pointcloud2calibration.header.frame_id = "perception";
            pub_lidar_calibration_.publish(pointcloud2calibration);
            if (print_message)
                std::cout << "showCalibration end" << std::endl;
        }
    }
}

void global::perception::calibration::Show_Calibration_Node::showRadarFusion(const radar_msgs::RadarObjectList::Ptr radar_objectlist)
{
    pcl::PointCloud<pcl::PointXYZRGB> bounding_box;
    uint8_t obj_num =radar_objectlist->ObjectList.size();
    for (size_t i=0; i<obj_num; i++)
    {
        pcl::PointXYZ points[8];
        for (int j=0;j<8; j++)
        {
            points[j].x = radar_objectlist->ObjectList[i].Rel_Bbox[j].x;
            points[j].y = radar_objectlist->ObjectList[i].Rel_Bbox[j].y;
            points[j].z = radar_objectlist->ObjectList[i].Rel_Bbox[j].z;
        }
        global::perception::radar::draw3DBoundingBox(points, bounding_box, 0, 0, 255);
    }
    sensor_msgs::PointCloud2 pointcloud2;
    pcl::toROSMsg(bounding_box, pointcloud2);
    pointcloud2.header.stamp = radar_objectlist->header.stamp;
    pointcloud2.header.frame_id = "perception";
    pub_radar_fusion_.publish(pointcloud2);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "show_calibration_node");
    ros::NodeHandle private_node("~");
    ros::NodeHandle public_node;
    global::perception::calibration::Show_Calibration_Node show_calibration_node(public_node, private_node);
    ros::MultiThreadedSpinner spinner(8);
    ros::spin(spinner);
    return 0;
}