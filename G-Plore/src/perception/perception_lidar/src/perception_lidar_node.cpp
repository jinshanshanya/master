#include "perception_lidar/perception_lidar_node.h"

perception::Perception_Lidar_Node::Perception_Lidar_Node(ros::NodeHandle public_node, ros::NodeHandle private_node,
double cost_threshold, double memory_time):
obstacle_tracking_(cost_threshold, memory_time)
{
    if (print_message)
        std::cout << "Constructor Perception_Lidar_Node  start" << std::endl;
    getParameter(public_node, private_node);
    obstacleMarkerInitialization();
    pub_lidar_ = public_node.advertise<sensor_msgs::PointCloud2>("/perception/lidar/pointcloud", 100);
    pub_lidar_cutcar_ = public_node.advertise<sensor_msgs::PointCloud2>("/perception/lidar/pointcloudcutcar", 100);
    pub_cubes_ = public_node.advertise<sensor_msgs::PointCloud2>("/perception/lidar/cubes", 100);
    pub_dcubes_ = public_node.advertise<sensor_msgs::PointCloud2>("/perception/lidar/dcubes", 100);
    pub_obstacles_info_ = public_node.advertise<visualization_msgs::Marker>("/perception/lidar/obstacles_information", 999999);
    pub_tracking_cubes_ = public_node.advertise<sensor_msgs::PointCloud2>("/perception/lidar/tracking_cubes", 100);
    pub_lidar_message_ = public_node.advertise<lidar_msgs::LidarObjectList>("/perception/lidar/object_list", 100);
    pub_lidar_front_ = public_node.advertise<sensor_msgs::PointCloud2>("/perception/lidar/front", 100);
    pub_lidar_back_ = public_node.advertise<sensor_msgs::PointCloud2>("/perception/lidar/back", 100);
    lidar_perception_thread1_ = std::thread(&perception::Perception_Lidar_Node::lidarPerception, this);
    lidar_perception_thread2_ = std::thread(&perception::Perception_Lidar_Node::lidarPerception, this);
    lidar_perception_thread3_ = std::thread(&perception::Perception_Lidar_Node::lidarPerception, this);
    lidar_perception_thread4_ = std::thread(&perception::Perception_Lidar_Node::lidarPerception, this);
    lidar_perception_thread5_ = std::thread(&perception::Perception_Lidar_Node::lidarPerception, this);
    obstacles_featue_tracking_ = boost::make_shared<std::vector<perception::Obstacle_Feature_Tracking<float>>>();
    lidar_tracking_thread_ = std::thread(&perception::Perception_Lidar_Node::lidarTracking, this);
    if (print_message)
        std::cout << "Constructor Perception_Lidar_Node  end" << std::endl;
}

perception::Perception_Lidar_Node::~Perception_Lidar_Node()
{
    if (print_message)
        std::cout << "Destructor Perception_Lidar_Node  start" << std::endl;
    lidar_perception_thread1_ .join();
    lidar_perception_thread2_ .join();
    lidar_perception_thread3_ .join();
    lidar_perception_thread4_ .join();
    lidar_perception_thread5_ .join();
    lidar_tracking_thread_.join();
    if (print_message)
        std::cout << "Destructor Perception_Lidar_Node  end" << std::endl;
}

void perception::Perception_Lidar_Node::getParameter(ros::NodeHandle public_node, ros::NodeHandle private_node )
{
    std::string topic_lidar1 = "/front_left/rslidar_points", topic_lidar2 = "/front_right/rslidar_points";
    std::string topic_lidar3 = "/lslidar_point_cloud";
    private_node.param("topic_lidar1", topic_lidar1, topic_lidar1);
    private_node.param("topic_lidar2", topic_lidar2, topic_lidar2);
    private_node.param("topic_lidar3", topic_lidar3, topic_lidar3);
    sub_lidar1_ = public_node.subscribe(topic_lidar1, 10, &perception::Perception_Lidar_Node::callbackLidar1, this);
    sub_lidar2_ = public_node.subscribe(topic_lidar2, 10, &perception::Perception_Lidar_Node::callbackLidar2, this);
    sub_lidar3_ = public_node.subscribe(topic_lidar3, 10, &perception::Perception_Lidar_Node::callbackLidar3, this);
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
    std::cout << "transform matrix lidar1 is: " << std::endl << std::fixed << std::setprecision(4) << transform_matrix_lidar1_ << std::endl;
    std::cout << "transform matrix lidar2 is: " << std::endl << std::fixed << std::setprecision(4) << transform_matrix_lidar2_ << std::endl;
    std::cout << "transform matrix lidar3 is: " << std::endl << std::fixed << std::setprecision(4) << transform_matrix_lidar3_ << std::endl;
}

void perception::Perception_Lidar_Node::callbackLidar1(const sensor_msgs::PointCloud2::ConstPtr msgs)
{
    // if (print_message)
    //     std::cout << "Function callbackLidar1 start" << std::endl;
    sensor_msgs::PointCloud2 pointcloud2_lidar1 = *msgs;
    mutex_lidar1_.lock();
    if (!queue_lidar1_.empty())
        queue_lidar1_.pop();
    queue_lidar1_.push(pointcloud2_lidar1);
    mutex_lidar1_.unlock();
    // if (print_message)
    //     std::cout << "Function callbackLidar1 end" << std::endl;
}

void perception::Perception_Lidar_Node::callbackLidar2(const sensor_msgs::PointCloud2::ConstPtr msgs)
{
    // if (print_message)
    //     std::cout << "Function callbackLidar2 start" << std::endl;
    sensor_msgs::PointCloud2 pointcloud2_lidar2 = *msgs;
    mutex_lidar2_.lock();
    if (!queue_lidar2_.empty())
        queue_lidar2_.pop();
    queue_lidar2_.push(pointcloud2_lidar2);
    mutex_lidar2_.unlock();
    // if (print_message)
    //     std::cout << "Function callbackLidar2 end" << std::endl;
}

void perception::Perception_Lidar_Node::callbackLidar3(const sensor_msgs::PointCloud2::ConstPtr msgs)
{
    // if (print_message)
    //     std::cout << "Function callbackLidar2 start" << std::endl;
    sensor_msgs::PointCloud2 pointcloud2_lidar3 = *msgs;
    mutex_lidar3_.lock();
    if (!queue_lidar3_.empty())
        queue_lidar3_.pop();
    queue_lidar3_.push(pointcloud2_lidar3);
    mutex_lidar3_.unlock();
    // if (print_message)
    //     std::cout << "Function callbackLidar2 end" << std::endl;
}

void perception::Perception_Lidar_Node::lidarPerception()
{
    while (true)
    {
        if (queue_lidar1_.empty() || queue_lidar2_.empty() || queue_lidar3_.empty())
        // if (queue_lidar1_.empty() || queue_lidar2_.empty())
        {
            // if (print_message)
            // {
            //     std::cout << "lidar queue is empty!" << std::endl;
            //     std::cout << "lidar1 size is: " << queue_lidar1_.size() << std::endl;
            //     std::cout << "lidar2 size is: " << queue_lidar2_.size() << std::endl;
            //     std::cout << "lidar3 size is: " << queue_lidar3_.size() << std::endl;
            //     std::cout << "lidar4 size is: " << queue_lidar4_.size() << std::endl;
            // }
            usleep(5000);
        }
        else
        {
            if (print_message)
                std::cout << "lidarPerception start" << std::endl;
            pcl::PointCloud<pcl::PointXYZI>::Ptr lidar(new pcl::PointCloud<pcl::PointXYZI>);
            // pcl::PointCloud<pcl::PointXYZI> lidar1, lidar1_groundless;
            // pcl::PointCloud<pcl::PointXYZI> lidar2, lidar2_groundless;
            // pcl::PointCloud<pcl::PointXYZI> lidar3, lidar3_groundless;
            // pcl::PointCloud<pcl::PointXYZI> lidar4, lidar4_groundless;
            pcl::PointCloud<pcl::PointXYZI> lidar1, lidar2, lidar3;
            std::thread thread_lidar1, thread_lidar2, thread_lidar3;
            ros::Time time_stamp;
            // ***** lidar 1 ***** // 
            mutex_lidar1_.lock(); 
            if (!queue_lidar1_.empty())
            {
                time_stamp = queue_lidar1_.front().header.stamp;
                pcl::fromROSMsg(queue_lidar1_.front(), lidar1);
                queue_lidar1_.pop();
            }
            mutex_lidar1_.unlock();
            // **************** // 
            // ***** lidar 2***** // 
            mutex_lidar2_.lock();
            if (!queue_lidar2_.empty())
            {
                if (time_stamp.toSec() < queue_lidar2_.front().header.stamp.toSec())
                    time_stamp = queue_lidar2_.front().header.stamp;
                pcl::fromROSMsg(queue_lidar2_.front(), lidar2);
                queue_lidar2_.pop();
            }
            mutex_lidar2_.unlock();
            // **************** // 
            // ***** lidar 3***** // 
            mutex_lidar3_.lock();
            if (!queue_lidar3_.empty())
            {
                if (time_stamp.toSec() < queue_lidar3_.front().header.stamp.toSec())
                    time_stamp = queue_lidar3_.front().header.stamp;
                pcl::fromROSMsg(queue_lidar3_.front(), lidar3);
                queue_lidar3_.pop();
            }
            mutex_lidar3_.unlock();
            // **************** // 
            // thread_lidar1 = std::thread(&perception::Perception_Lidar_Node::lidarPreprocessing, this, std::ref(lidar1), std::ref(lidar1_groundless), std::ref(transform_matrix_lidar1_));
            // thread_lidar2 = std::thread(&perception::Perception_Lidar_Node::lidarPreprocessing, this, std::ref(lidar2), std::ref(lidar2_groundless), std::ref(transform_matrix_lidar2_));
            // thread_lidar3 = std::thread(&perception::Perception_Lidar_Node::lidarPreprocessing, this, std::ref(lidar3), std::ref(lidar3_groundless), std::ref(transform_matrix_lidar3_));
            // thread_lidar4 = std::thread(&perception::Perception_Lidar_Node::lidarPreprocessing, this, std::ref(lidar4), std::ref(lidar4_groundless), std::ref(transform_matrix_lidar4_));
            thread_lidar1 = std::thread(&perception::Perception_Lidar_Node::lidarPreprocessing, this, std::ref(lidar1), std::ref(transform_matrix_lidar1_));
            thread_lidar2 = std::thread(&perception::Perception_Lidar_Node::lidarPreprocessing, this, std::ref(lidar2), std::ref(transform_matrix_lidar2_));
            thread_lidar3 = std::thread(&perception::Perception_Lidar_Node::lidarPreprocessing, this, std::ref(lidar3), std::ref(transform_matrix_lidar3_));
            thread_lidar1.join();
            thread_lidar2.join();
            thread_lidar3.join();
            // *lidar = lidar1_groundless + lidar2_groundless;
            // *lidar += lidar3_groundless;
            // *lidar += lidar4_groundless;
            *lidar = lidar1 + lidar2;
            *lidar += lidar3;
            if (lidar->size() == 0)
                continue;
            if (print_message)
            {
                std::cout << "original point size is: " << lidar->size() << std::endl;
                showLidarCloud(lidar, time_stamp);
                showFrontBackLidar(lidar1, lidar2, lidar3, time_stamp);
            }
            pcl::PointCloud<pcl::PointXYZI>::Ptr lidar_roi(new pcl::PointCloud<pcl::PointXYZI>);
            perception::setRoi(lidar, lidar_roi, -2.5, 2.5, -1.4, 7.9, -1.5, 1.0, -25.0, 25.0, -35.0, 90.0, -1.5, 200.0);
            if (print_message)
            {
                std::cout << "Point size (cut car and set ROI) is: " << lidar_roi->size() << std::endl;
                showCutCarCloud(lidar_roi, time_stamp);
            }
            pcl::PointCloud<pcl::PointXYZI>::Ptr lidar_voxel(new pcl::PointCloud<pcl::PointXYZI>);
            perception::voxelGridFilter(lidar_roi, lidar_voxel, 0.2, 0.2, 0.1);
            if (print_message)
                std::cout << "voxel point size is: " << lidar_voxel->size() << std::endl;
            if (lidar_voxel->size() == 0)
                continue;
            pcl::PointCloud<pcl::PointXYZI>::Ptr lidar_groundless(new pcl::PointCloud<pcl::PointXYZI>);
            pcl::PointCloud<pcl::PointXYZI>::Ptr ground(new pcl::PointCloud<pcl::PointXYZI>);
            // perception::groundCutMorphologicalFilter<pcl::PointXYZI, float>(lidar_voxel, lidar_groundless, ground, 1.0f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f, true);
            perception::groundCutMorphologicalFilter<pcl::PointXYZI, float>(lidar_voxel, lidar_groundless, ground, 1.0f, 1.2f, 0.6f, 0.5f, 1.5f, 0.5f, true);
            if (print_message)
                std::cout << "Point size (MorphologicalFilter) is: " << lidar_groundless->size() << std::endl;
            perception::zAxisCompression(lidar_groundless, FACTOR_Z);
            // // ****** cut car ******* //
            // pcl::PointCloud<pcl::PointXYZI>::Ptr lidar_car_cut(new pcl::PointCloud<pcl::PointXYZI>);
            // // perception::carCut(lidar_groundless, lidar_car_cut, -2.5, 2.5, -1.4, 7.9, -1.0, 1.0, -200.0, 200.0, -200.0, 200.0, -200.0, 200.0, FACTOR_Z);
            // perception::carCut(lidar_groundless, lidar_car_cut, -2.5, 2.5, -1.4, 7.9, -1.5, 1.0, -200.0, 200.0, -200.0, 200.0, -1.5, 200.0, FACTOR_Z);
            // // *lidar_car_cut = lidar_groundless;
            // if (print_message)
            // {
            //     std::cout << "Point size (car cut) is: " << lidar_car_cut->size() << std::endl;
            //     showCutCarCloud(lidar_car_cut, time_stamp);
            // }
            // ******************** //
            // sensor_msgs::PointCloud2 pointcloud2cutcar;
            // pcl::toROSMsg(*lidar_car_cut,pointcloud2cutcar);
            // pointcloud2cutcar.header.stamp = ros::Time::now();
            // pointcloud2cutcar.header.frame_id = "perception";
            // pub_lidar_cutcar.publish(pointcloud2cutcar);
            // pcl::PointCloud<pcl::PointXYZI>::Ptr lidar_filted(new pcl::PointCloud<pcl::PointXYZI>);
            // perception::voxelGridFilter(lidar_car_cut, lidar_filted, 0.05, 0.05, 0.01);
            // if (print_message)
            //     std::cout << "Point size (voxel grid filter) is: " << lidar_filted->size() << std::endl;
            std::vector<pcl::PointIndices> cluster_indices;
            // perception::euclideanCluster<pcl::PointXYZI>(cluster_indices, lidar_car_cut, 0.3, 10, 30000);
            // perception::regionGrowing<pcl::PointXYZI, pcl::Normal>(cluster_indices, lidar_car_cut, 30, 10, 30000, 40, 5.0/180.0*M_PI, 100);
            perception::conditionalEculideanCluster<::pcl::PointXYZI, ::pcl::PointXYZINormal>(cluster_indices, lidar_groundless, 0.4, 0.5, 5, 30000);
            if (print_message)
                std::cout << "Cluster_indices size is: " << cluster_indices.size() << std::endl;
            std::vector<perception::Obstacle_Feature_Point_Cloud<pcl::PointXYZI, float>>  obstacles_feature;
            perception::getObstaclesFeature(obstacles_feature, cluster_indices, lidar_groundless, FACTOR_Z);
            if (print_message)
            {
                drawCubes(obstacles_feature, time_stamp);
                drawDCubes(obstacles_feature, time_stamp);
            }
            mutex_queue_obstacles_.lock();
            queue_time_stamp_.push(time_stamp);
            std::cout << "queue time_stamp is: " << time_stamp << std::endl;
            queue_obstacles_feature_.push(obstacles_feature);
            mutex_queue_obstacles_.unlock();
            // for (int i=0; i<cluster_indices.size(); i++)
            // {
            //     std::cout << i << "  zmin: "<< obstacles_feature[i].cube_.zmin_ << std::endl;
            //     std::cout << i << "  zmax: "<< obstacles_feature[i].cube_.zmax_ << std::endl;
            //     std::cout << i << "  ymin: "<< obstacles_feature[i].cube_.ymin_ << std::endl;
            //     std::cout << i << "  ymax: "<< obstacles_feature[i].cube_.ymax_ << std::endl;
            //     std::cout << i << "  xmin: "<< obstacles_feature[i].cube_.xmin_ << std::endl;
            //     std::cout << i << "  xmax: "<< obstacles_feature[i].cube_.xmax_ << std::endl;
            // }
            if (print_message)
                std::cout << "lidarPerception end" << std::endl;
        }
    }
}

void perception::Perception_Lidar_Node::showLidarCloud(
pcl::PointCloud<pcl::PointXYZI>::Ptr lidar_ptr,
ros::Time& time_stamp)
{
    sensor_msgs::PointCloud2 pointcloud2;
    pcl::toROSMsg(*lidar_ptr, pointcloud2);
    pointcloud2.header.stamp = time_stamp;
    pointcloud2.header.frame_id = "perception";
    pub_lidar_.publish(pointcloud2);
}

// void perception::Perception_Lidar_Node::lidarPreprocessing(pcl::PointCloud<pcl::PointXYZI>& lidar, pcl::PointCloud<pcl::PointXYZI>& lidar_groundless, Eigen::Matrix4f& transform_matrix)
// {
//     // rotation and translation
//     pcl::PointCloud<pcl::PointXYZI>::Ptr lidar_transform(new pcl::PointCloud<pcl::PointXYZI>);
//     pcl::transformPointCloud(lidar, *lidar_transform, transform_matrix);
//     pcl::PointCloud<pcl::PointXYZI> ground;
//     // perception::groundCutMorphologicalFilter<pcl::PointXYZI, float>(lidar_transform, lidar_groundless, ground, 33, 3.0f, 0.5f, 2.0f, 1.0f, 2.0f, true);
//     // perception::groundCutMorphologicalFilter<pcl::PointXYZI, float>(lidar_transform, lidar_groundless, ground, 1.0f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f, true);
//     lidar_groundless = *lidar_transform;
// }

void perception::Perception_Lidar_Node::lidarPreprocessing(pcl::PointCloud<pcl::PointXYZI>& lidar, Eigen::Matrix4f& transform_matrix)
{
    // rotation and translation
    pcl::transformPointCloud(lidar, lidar, transform_matrix);
}

void perception::Perception_Lidar_Node::drawCubes(
std::vector<perception::Obstacle_Feature_Point_Cloud<pcl::PointXYZI, float>>&  obstacles_feature,
ros::Time& time_stamp)
{
    pcl::PointCloud<pcl::PointXYZRGB> bounding_box;
    size_t len = obstacles_feature.size();
    for (size_t i=0; i<len; i++)
    {
        Point3D<float> dcube[8];
        dcube[0].x_ = obstacles_feature[i].cube_.xmax_;
        dcube[0].y_ = obstacles_feature[i].cube_.ymax_;
        dcube[0].z_ = obstacles_feature[i].cube_.zmax_;
        dcube[1].x_ = obstacles_feature[i].cube_.xmin_;
        dcube[1].y_ = obstacles_feature[i].cube_.ymax_;
        dcube[1].z_ = obstacles_feature[i].cube_.zmax_;
        dcube[2].x_ = obstacles_feature[i].cube_.xmin_;
        dcube[2].y_ = obstacles_feature[i].cube_.ymin_;
        dcube[2].z_ = obstacles_feature[i].cube_.zmax_;
        dcube[3].x_ = obstacles_feature[i].cube_.xmax_;
        dcube[3].y_ = obstacles_feature[i].cube_.ymin_;
        dcube[3].z_ = obstacles_feature[i].cube_.zmax_;
        dcube[4].x_ = obstacles_feature[i].cube_.xmax_;
        dcube[4].y_ = obstacles_feature[i].cube_.ymax_;
        dcube[4].z_ = obstacles_feature[i].cube_.zmin_;
        dcube[5].x_ = obstacles_feature[i].cube_.xmin_;
        dcube[5].y_ = obstacles_feature[i].cube_.ymax_;
        dcube[5].z_ = obstacles_feature[i].cube_.zmin_;
        dcube[6].x_ = obstacles_feature[i].cube_.xmin_;
        dcube[6].y_ = obstacles_feature[i].cube_.ymin_;
        dcube[6].z_ = obstacles_feature[i].cube_.zmin_;
        dcube[7].x_ = obstacles_feature[i].cube_.xmax_;
        dcube[7].y_ = obstacles_feature[i].cube_.ymin_;
        dcube[7].z_ = obstacles_feature[i].cube_.zmin_;
        perception::draw3DBoundingBox<float>(dcube, bounding_box, 255, 0, 0);
    }
    sensor_msgs::PointCloud2 pointcloud2;
    pcl::toROSMsg(bounding_box, pointcloud2);
    pointcloud2.header.stamp = time_stamp;
    pointcloud2.header.frame_id = "perception";
    pub_cubes_.publish(pointcloud2);
}

void perception::Perception_Lidar_Node::drawDCubes(
std::vector<perception::Obstacle_Feature_Point_Cloud<pcl::PointXYZI, float>>&  obstacles_feature,
ros::Time& time_stamp)
{
    size_t len = obstacles_feature.size();
    pcl::PointCloud<pcl::PointXYZRGB> bounding_box;
    for (size_t i=0; i<len; i++)
        perception::draw3DBoundingBox<float>(obstacles_feature[i].dcube_, bounding_box, 255, 0, 0);
    sensor_msgs::PointCloud2 pointcloud2;
    pcl::toROSMsg(bounding_box, pointcloud2);
    pointcloud2.header.stamp = time_stamp;
    pointcloud2.header.frame_id = "perception";
    pub_dcubes_.publish(pointcloud2);
}

void perception::Perception_Lidar_Node::showCutCarCloud(
pcl::PointCloud<pcl::PointXYZI>::Ptr lidar_ptr,
ros::Time& time_stamp)
{
    pcl::PointCloud<pcl::PointXYZI> lidar_inflated = *lidar_ptr;
    for (int i=0; i<lidar_inflated.size(); i++)
        lidar_inflated.points[i].z *= FACTOR_Z;
    sensor_msgs::PointCloud2 pointcloud2cutcar;
    pcl::toROSMsg(lidar_inflated,pointcloud2cutcar);
    pointcloud2cutcar.header.stamp = time_stamp;
    pointcloud2cutcar.header.frame_id = "perception";
    pub_lidar_cutcar_.publish(pointcloud2cutcar);
}

void perception::Perception_Lidar_Node::obstacleMarkerInitialization()
{
    obstacle_marker_.header.frame_id = "perception";
    obstacle_marker_.header.stamp = ros::Time::now();
    obstacle_marker_.ns = "obstacles_infomstion";
    obstacle_marker_.action = visualization_msgs::Marker::ADD;
    obstacle_marker_.pose.orientation.x = 0.0;
    obstacle_marker_.pose.orientation.y = 0.0;
    obstacle_marker_.pose.orientation.z = 0.0;
    obstacle_marker_.pose.orientation.w = 1.0;
    obstacle_marker_.lifetime = ros::Duration(0.18);
    obstacle_marker_.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    // obstacle_marker_.scale.x = 1.0;
    // obstacle_marker_.scale.y = 1.0;
    obstacle_marker_.scale.z = 0.5;
    obstacle_marker_.color.r = 1.0;
    obstacle_marker_.color.g = 0.0;
    obstacle_marker_.color.b = 0.0;
    obstacle_marker_.color.a = 1.0;
}

void perception::Perception_Lidar_Node::drawText(Point3D<float> position,
uint32_t id, const Point3D<float>& velocity)
{
    obstacle_marker_.id = id;
    obstacle_marker_.pose.position.x = position.x_;
    obstacle_marker_.pose.position.y = position.y_;
    obstacle_marker_.pose.position.z = position.z_;
    obstacle_marker_.text = std::string("id: ") + std::to_string(id)
    + std::string("\nvx: ") + std::to_string(velocity.x_).substr(0, 6)
    + std::string("\nvy: ") + std::to_string(velocity.y_).substr(0, 6)
    + std::string("\nvz: ") + std::to_string(velocity.z_).substr(0, 6);
    pub_obstacles_info_.publish(obstacle_marker_);
}

void perception::Perception_Lidar_Node::drawObstaclesInfo()
{
    size_t obstacles_size = obstacles_featue_tracking_->size();
    for (size_t i=0; i<obstacles_size; i++)
    {
        Point3D<float> position((*obstacles_featue_tracking_)[i].cube_.xmin_,
        (*obstacles_featue_tracking_)[i].cube_.ymax_+1.2,
        (*obstacles_featue_tracking_)[i].cube_.zmax_);
        drawText(position, (*obstacles_featue_tracking_)[i].id_,
        (*obstacles_featue_tracking_)[i].velocity_);
    }
}

void perception::Perception_Lidar_Node::lidarTracking()
{
    while (true)
    {
        if (queue_obstacles_feature_.size() > 0)
        {
            if (print_message)
                std::cout << "queue_obstacles_feature_ size is: " << queue_obstacles_feature_.size() << std::endl;
            boost::shared_ptr<std::vector<perception::Obstacle_Feature_Tracking<float>>> new_obstacles_featue_tracking
            = boost::make_shared<std::vector<perception::Obstacle_Feature_Tracking<float>>>();
            mutex_queue_obstacles_.lock();
            std::cout << "queue front time stamp is: " << queue_time_stamp_.front().toSec() << std::endl;
            obstacle_tracking_.obstacleTracking(obstacles_featue_tracking_, queue_obstacles_feature_.front(),
            new_obstacles_featue_tracking, iterator_id_, queue_time_stamp_.front().toSec(), 30);
            obstacles_featue_tracking_ = new_obstacles_featue_tracking;
            publishLidarMessage(queue_time_stamp_.front());
            queue_obstacles_feature_.pop();
            queue_time_stamp_.pop();
            mutex_queue_obstacles_.unlock();
            if (print_message)
            {
                std::cout << "obstacles_featue_tracking_ size is: " << obstacles_featue_tracking_->size() << std::endl;
                drawTrackingObstacles();
            }
        }
        else
        {
            usleep(1000);
        }
    }
}

void perception::Perception_Lidar_Node::drawTrackingCubes()
{
    pcl::PointCloud<pcl::PointXYZRGB> bounding_box;
    size_t len = obstacles_featue_tracking_->size();
    for (size_t i=0; i<len; i++)
    {
        Point3D<float> dcube[8];
        dcube[0].x_ = (*obstacles_featue_tracking_)[i].cube_.xmax_;
        dcube[0].y_ = (*obstacles_featue_tracking_)[i].cube_.ymax_;
        dcube[0].z_ = (*obstacles_featue_tracking_)[i].cube_.zmax_;
        dcube[1].x_ = (*obstacles_featue_tracking_)[i].cube_.xmin_;
        dcube[1].y_ = (*obstacles_featue_tracking_)[i].cube_.ymax_;
        dcube[1].z_ = (*obstacles_featue_tracking_)[i].cube_.zmax_;
        dcube[2].x_ = (*obstacles_featue_tracking_)[i].cube_.xmin_;
        dcube[2].y_ = (*obstacles_featue_tracking_)[i].cube_.ymin_;
        dcube[2].z_ = (*obstacles_featue_tracking_)[i].cube_.zmax_;
        dcube[3].x_ = (*obstacles_featue_tracking_)[i].cube_.xmax_;
        dcube[3].y_ = (*obstacles_featue_tracking_)[i].cube_.ymin_;
        dcube[3].z_ = (*obstacles_featue_tracking_)[i].cube_.zmax_;
        dcube[4].x_ = (*obstacles_featue_tracking_)[i].cube_.xmax_;
        dcube[4].y_ = (*obstacles_featue_tracking_)[i].cube_.ymax_;
        dcube[4].z_ = (*obstacles_featue_tracking_)[i].cube_.zmin_;
        dcube[5].x_ = (*obstacles_featue_tracking_)[i].cube_.xmin_;
        dcube[5].y_ = (*obstacles_featue_tracking_)[i].cube_.ymax_;
        dcube[5].z_ = (*obstacles_featue_tracking_)[i].cube_.zmin_;
        dcube[6].x_ = (*obstacles_featue_tracking_)[i].cube_.xmin_;
        dcube[6].y_ = (*obstacles_featue_tracking_)[i].cube_.ymin_;
        dcube[6].z_ = (*obstacles_featue_tracking_)[i].cube_.zmin_;
        dcube[7].x_ = (*obstacles_featue_tracking_)[i].cube_.xmax_;
        dcube[7].y_ = (*obstacles_featue_tracking_)[i].cube_.ymin_;
        dcube[7].z_ = (*obstacles_featue_tracking_)[i].cube_.zmin_;
        perception::draw3DBoundingBox<float>(dcube, bounding_box, 255, 0, 0);
    }
    sensor_msgs::PointCloud2 pointcloud2;
    pcl::toROSMsg(bounding_box, pointcloud2);
    pointcloud2.header.stamp = ros::Time::now();
    pointcloud2.header.frame_id = "perception";
    pub_tracking_cubes_.publish(pointcloud2);
}

void perception::Perception_Lidar_Node::drawTrackingObstacles()
{
    drawTrackingCubes();
    drawObstaclesInfo();
}

void perception::Perception_Lidar_Node::publishLidarMessage(ros::Time& time_stamp)
{
    lidar_msgs::LidarObjectList object_list;
    object_list.ObjNum = obstacles_featue_tracking_->size();
    object_list.header.stamp = time_stamp;
    object_list.header.frame_id = "perception";
    for (size_t i=0; i<object_list.ObjNum; i++)
    {
        lidar_msgs::LidarObject object;
        object.ID = (*obstacles_featue_tracking_)[i].id_;
        object.Rel_Pos.x = ((*obstacles_featue_tracking_)[i].dcube_[0].x_ + (*obstacles_featue_tracking_)[i].dcube_[2].x_) / 2;
        object.Rel_Pos.y = ((*obstacles_featue_tracking_)[i].dcube_[0].y_ + (*obstacles_featue_tracking_)[i].dcube_[2].y_) / 2;
        object.Rel_Pos.z = ((*obstacles_featue_tracking_)[i].dcube_[0].z_ + (*obstacles_featue_tracking_)[i].dcube_[4].z_) / 2;
        for (size_t j=0; j<8; j++)
        {
            object.Rel_Bbox[j].x = (*obstacles_featue_tracking_)[i].dcube_[j].x_;
            object.Rel_Bbox[j].y = (*obstacles_featue_tracking_)[i].dcube_[j].y_;
            object.Rel_Bbox[j].z = (*obstacles_featue_tracking_)[i].dcube_[j].z_;
        }
        object.Rel_Range.xMax = (*obstacles_featue_tracking_)[i].cube_.xmax_;
        object.Rel_Range.xMin = (*obstacles_featue_tracking_)[i].cube_.xmin_;
        object.Rel_Range.yMax = (*obstacles_featue_tracking_)[i].cube_.ymax_;
        object.Rel_Range.yMin = (*obstacles_featue_tracking_)[i].cube_.ymin_;
        object.Rel_Range.zMax = (*obstacles_featue_tracking_)[i].cube_.zmax_;
        object.Rel_Range.zMin = (*obstacles_featue_tracking_)[i].cube_.zmin_;
        object.Rel_Vel.x = (*obstacles_featue_tracking_)[i].velocity_.x_;
        object.Rel_Vel.y = (*obstacles_featue_tracking_)[i].velocity_.y_;
        object.Rel_Vel.z = (*obstacles_featue_tracking_)[i].velocity_.z_;
        object_list.ObjectList.push_back(object);
    }
    pub_lidar_message_.publish(object_list);
}

void perception::Perception_Lidar_Node::showFrontBackLidar(pcl::PointCloud<pcl::PointXYZI>& lidar1, pcl::PointCloud<pcl::PointXYZI>& lidar2,
pcl::PointCloud<pcl::PointXYZI>& lidar3, ros::Time& time_stamp)
{
    pcl::PointCloud<pcl::PointXYZI> lidar_front = lidar1 + lidar2;
    // lidar front
    sensor_msgs::PointCloud2 pointcloud2_front;
    pcl::toROSMsg(lidar_front, pointcloud2_front);
    pointcloud2_front.header.stamp = time_stamp;
    pointcloud2_front.header.frame_id = "perception";
    pub_lidar_front_.publish(pointcloud2_front); 
    // lidar back
    sensor_msgs::PointCloud2 pointcloud2_back;
    pcl::toROSMsg(lidar3, pointcloud2_back);
    pointcloud2_back.header.stamp = time_stamp;
    pointcloud2_back.header.frame_id = "perception";
    pub_lidar_back_.publish(pointcloud2_back); 
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "perception_lidar_node");
    ros::NodeHandle private_node("~");
    ros::NodeHandle public_node;
    double cost_threshold = 0.05, memory_time = 0.35;
    private_node.param("cost_threshold", cost_threshold, cost_threshold);
    private_node.param("memory_time", memory_time, memory_time);
    std::cout << "memory_time = " << memory_time << std::endl;
    
    perception::Perception_Lidar_Node perception_lidar_node(public_node, private_node,
    cost_threshold, memory_time);
    ros::MultiThreadedSpinner spinner(8);
    ros::spin(spinner);
    ROS_INFO("Lidar perception node has been opened!");
    ROS_INFO("Lidar perception node has be closed!");
    return 0;
}

