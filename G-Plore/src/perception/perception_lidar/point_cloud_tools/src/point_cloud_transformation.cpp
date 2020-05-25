#include "perception_lidar/point_cloud_tools/point_cloud_transformation.h"

void perception::point_cloud_transformation(pcl::PointCloud<pcl::PointXYZI>& lidar, Eigen::Matrix4f& transform)
{
    pcl::transformPointCloud(lidar, lidar, transform);
}

void perception::point_cloud_transformation(pcl::PointCloud<pcl::PointXYZI>& lidar, Eigen::Affine3f& transform)
{
    pcl::transformPointCloud(lidar, lidar, transform);
}

void perception::getXRotatedMatrix4f(Eigen::Matrix4f& rotated_matrix, double theta)
{
    rotated_matrix << 1.0, 0.0, 0.0, 0.0, 
    0.0, cos(theta), -sin(theta), 0.0,
    0.0, sin(theta), cos(theta), 0.0,
    0.0, 0.0, 0.0, 1; 
}

void perception::getYRotatedMatrix4f(Eigen::Matrix4f& rotated_matrix, double theta)
{
    rotated_matrix << cos(theta), 0, sin(theta), 0,
    0, 1, 0, 0,
    -sin(theta), 0, cos(theta), 0,
    0, 0, 0, 1;
}

void perception::getZRotatedMatrix4f(Eigen::Matrix4f& rotated_matrix, double theta)
{
    rotated_matrix << cos(theta), -sin(theta), 0, 0,
    sin(theta), cos(theta), 0, 0,
    0, 0, 1, 0,
    0, 0, 0, 1;
}

void perception::getTranslationMatrix4f(Eigen::Matrix4f& rotated_matrix, double x, double y, double z)
{
    rotated_matrix << 1, 0, 0, x,
    0, 1, 0, y,
    0, 0, 1, z,
    0, 0, 0, 1;
}

void perception::Rodrigues(double rotated_vector[3], double rotated_matrix[9])
{
    CvMat pr_vector, pr_matrix;
    cvInitMatHeader(&pr_vector, 1, 3, CV_64FC1, rotated_vector, CV_AUTOSTEP);
    cvInitMatHeader(&pr_matrix, 3, 3, CV_64FC1, rotated_matrix, CV_AUTOSTEP);
    cvRodrigues2(&pr_vector, &pr_matrix, 0);
}

// cut car
void perception::carCut(pcl::PointCloud<pcl::PointXYZI>& lidar, pcl::PointCloud<pcl::PointXYZI>::Ptr lidar_car_cut,
float xmin_car, float xmax_car, float ymin_car, float ymax_car, float zmin_car, float zmax_car,
float xmin, float xmax, float ymin, float ymax, float zmin, float zmax, float factor_z)
{
    for (pcl::PointXYZI point : lidar)
    {
        if (point.x >= xmin_car && point.x <=xmax_car
        && point.y >= ymin_car && point.y <= ymax_car
        && point.z >= zmin_car && point.z <= zmax_car)
            continue;
        if (point.x >= xmin && point.x <= xmax
        && point.y >= ymin && point.y <= ymax
        && point.z >= zmin && point.z <= zmax)
        {
            point.z /= factor_z;
            lidar_car_cut->push_back(point);
        }
    }
}

void perception::voxelGridFilter(pcl::PointCloud<pcl::PointXYZI>::Ptr lidar, pcl::PointCloud<pcl::PointXYZI>::Ptr lidar_filted,
float leaf_size_x, float leaf_size_y, float leaf_size_z)
{
    pcl::VoxelGrid<pcl::PointXYZI> vg ;
    vg.setInputCloud(lidar);
    vg.setLeafSize(leaf_size_x, leaf_size_y, leaf_size_z);
    vg.filter(*lidar_filted);
}
