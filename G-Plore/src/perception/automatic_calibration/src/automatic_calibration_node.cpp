#include "automatic_calibration/automatic_calibration_node.h"

global::perception::calibration::Automatic_Calibration_Node::Automatic_Calibration_Node(
ros::NodeHandle& public_node, ros::NodeHandle& private_node)
{
    if (print_message)
        std::cout << "Constructor Automatic_Calibration_Node  start" << std::endl;
    getParameter(public_node, private_node);
    pub_lidar_calibration_ = public_node.advertise<sensor_msgs::PointCloud2>("/perception/lidar/pointcloudcalibration", 100);
    automatic_calibration_ = std::thread(&global::perception::calibration::Automatic_Calibration_Node::automaticCalibration, this);
    if (print_message)
        std::cout << "Constructor Automatic_Calibration_Node  end" << std::endl;
}

void global::perception::calibration::Automatic_Calibration_Node::getParameter(
ros::NodeHandle& public_node, ros::NodeHandle& private_node)
{
    std::string topic_lidar1 = "/ns1/rslidar_points", topic_lidar2 = "/ns2/rslidar_points";
    std::string topic_lidar3 = "/lslidar_point_cloud";
    private_node.param("topic_lidar1", topic_lidar1, topic_lidar1);
    private_node.param("topic_lidar2", topic_lidar2, topic_lidar2);
    private_node.param("topic_lidar3", topic_lidar3, topic_lidar3);
    sub_lidar1_ = public_node.subscribe(topic_lidar1, 10, &global::perception::calibration::Automatic_Calibration_Node::callbackLidar1, this);
    sub_lidar2_ = public_node.subscribe(topic_lidar2, 10, &global::perception::calibration::Automatic_Calibration_Node::callbackLidar2, this);
    sub_lidar3_ = public_node.subscribe(topic_lidar3, 10, &global::perception::calibration::Automatic_Calibration_Node::callbackLidar3, this);
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

}

void global::perception::calibration::Automatic_Calibration_Node::callbackLidar1(
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

void global::perception::calibration::Automatic_Calibration_Node::callbackLidar2(
const sensor_msgs::PointCloud2::Ptr msgs)
{
    sensor_msgs::PointCloud2 pointcloud2_lidar2 = *msgs;
    mutex_lidar2_.lock();
    if (!queue_lidar2_.empty())
        queue_lidar2_.pop();
    queue_lidar2_.push(pointcloud2_lidar2);
    mutex_lidar2_.unlock();
}

void global::perception::calibration::Automatic_Calibration_Node::callbackLidar3(
const sensor_msgs::PointCloud2::Ptr msgs)
{
    sensor_msgs::PointCloud2 pointcloud2_lidar3 = *msgs;
    mutex_lidar3_.lock();
    if (!queue_lidar3_.empty())
        queue_lidar3_.pop();
    queue_lidar3_.push(pointcloud2_lidar3);
    mutex_lidar3_.unlock();
}

void global::perception::calibration::Automatic_Calibration_Node::automaticCalibration()
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
                std::cout << "automaticCalibration start" << std::endl;
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
            removeInfinitePoint(lidar1);
            removeInfinitePoint(lidar2);
            removeInfinitePoint(lidar3);
            pcl::transformPointCloud(*lidar1, *lidar1, transform_matrix_lidar1_);
            pcl::transformPointCloud(*lidar2, *lidar2, transform_matrix_lidar2_);
            pcl::transformPointCloud(*lidar3, *lidar3, transform_matrix_lidar3_);
            float score_21, score_31;
            Eigen::Matrix4f transform_matrix_21, transform_matrix_31;
            bool has_converged_21 = iterativeClosestPoint(lidar2, lidar1, score_21, transform_matrix_21);
            pcl::transformPointCloud(*lidar2, *lidar2, transform_matrix_21);
            std::cout << "lidar2 --> lidar 1:"<< std::endl;
            std::cout << "----------------------------------------------------------"<< std::endl;
            std::cout << "has converged: " << has_converged_21 <<std::endl;
            std::cout << "score: " << score_21 << std::endl;
            std::cout << "transform_matirx_21:" << std::endl << transform_matrix_21 << std::endl;
            bool has_converged_31 = iterativeClosestPoint(lidar3, lidar1, score_31, transform_matrix_31);
            pcl::transformPointCloud(*lidar3, *lidar3, transform_matrix_31);
            std::cout << "lidar3 --> lidar 1:"<< std::endl;
            std::cout << "----------------------------------------------------------"<< std::endl;
            std::cout << "has converged: " << has_converged_31 <<std::endl;
            std::cout << "score: " << score_31 << std::endl;
            std::cout << "transform_matirx_31:" << std::endl << transform_matrix_31 << std::endl;
            std::cout << "lidar 1:"<< std::endl;
            std::cout << transform_matrix_lidar1_ << std::endl;
            std::cout << "lidar 2:"<< std::endl;
            std::cout << transform_matrix_21 * transform_matrix_lidar2_ << std::endl;
            std::cout << "lidar 3:"<< std::endl;
            std::cout << transform_matrix_31 * transform_matrix_lidar3_ << std::endl;
            *lidar = *lidar1 + *lidar2;
            *lidar += *lidar3;
            // *lidar = *lidar1 + *lidar3;
            sensor_msgs::PointCloud2 pointcloud2calibration;
            pcl::toROSMsg(*lidar, pointcloud2calibration);
            pointcloud2calibration.header.stamp = ros::Time::now();
            pointcloud2calibration.header.frame_id = "perception";
            pub_lidar_calibration_.publish(pointcloud2calibration);
            if (print_message)
                std::cout << "automaticCalibration end" << std::endl;
        }
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "automatic_calibration_node");
    ros::NodeHandle private_node("~");
    ros::NodeHandle public_node;
    global::perception::calibration::Automatic_Calibration_Node automatic_calibration_node(public_node, private_node);
    ros::MultiThreadedSpinner spinner(8);
    ros::spin(spinner);
    return 0;
}