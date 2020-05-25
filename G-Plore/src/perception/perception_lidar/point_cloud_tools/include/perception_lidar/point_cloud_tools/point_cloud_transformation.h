#ifndef _POINT_CLOUD_TRANSFORMATION_
#define _POINT_CLOUD_TRANSFORMATION_

// pcl
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/kdtree.h>

// Eigen
#include <Eigen/Dense>

// opencv
#include <cv.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>

// c++
#include <math.h>

namespace perception
{
    // rotation and translation

    void point_cloud_transformation(pcl::PointCloud<pcl::PointXYZI>& lidar, Eigen::Matrix4f& transform);

    void point_cloud_transformation(pcl::PointCloud<pcl::PointXYZI>& lidar, Eigen::Affine3f& transform);

    void getXRotatedMatrix4f(Eigen::Matrix4f& rotated_matrix, double theta=0.0);

    void getYRotatedMatrix4f(Eigen::Matrix4f& rotated_matrix, double theta=0.0);

    void getZRotatedMatrix4f(Eigen::Matrix4f& rotated_matrix, double theta=0.0);

    void getTranslationMatrix4f(Eigen::Matrix4f& rotated_matrix, double x=0.0, double y=0.0, double z=0.0);

    void Rodrigues(double rotated_vector[3], double rotated_matirx[9]);

    // cut car
    void carCut(pcl::PointCloud<pcl::PointXYZI>& lidar, pcl::PointCloud<pcl::PointXYZI>::Ptr lidar_car_cut,
    float xmin_car, float xmax_car, float ymin_car, float ymax_car, float zmin_car, float zmax_car,
    float xmin, float xmax, float ymin, float ymax, float zmin = -200, float zmax = 200, float factor_z = 10);

    // set roi
    template<typename PointT, typename T>
    void setRoi(boost::shared_ptr<pcl::PointCloud<PointT>> input_cloud, boost::shared_ptr<pcl::PointCloud<PointT>> output_cloud,
    T xmin_car, T xmax_car, T ymin_car, T ymax_car, T zmin_car, T zmax_car,
    T xmin, T xmax, T ymin, T ymax, T zmin =-200, T zmax =200);

    // compress z axis
    template<typename PointT, typename T>
    void zAxisCompression(boost::shared_ptr<pcl::PointCloud< PointT>> cloud, T factor_z = 10);

    //  filter
    void voxelGridFilter(pcl::PointCloud<pcl::PointXYZI>::Ptr lidar, pcl::PointCloud<pcl::PointXYZI>::Ptr lidar_filted,
    float leaf_size_x = 0.05, float leaf_size_y = 0.05, float leaf_size_z = 0.01);

}

template<typename PointT, typename T>
void perception::setRoi(boost::shared_ptr<pcl::PointCloud<PointT>> input_cloud, boost::shared_ptr<pcl::PointCloud<PointT>> output_cloud,
T xmin_car, T xmax_car, T ymin_car, T ymax_car, T zmin_car, T zmax_car,
T xmin, T xmax, T ymin, T ymax, T zmin, T zmax)
{
    for (PointT point: *input_cloud)
    {
        if (point.x >= xmin_car && point.x <=xmax_car
        && point.y >= ymin_car && point.y <= ymax_car
        && point.z >= zmin_car && point.z <= zmax_car)
            continue;
        if (point.x >= xmin && point.x <= xmax
        && point.y >= ymin && point.y <= ymax
        && point.z >= zmin && point.z <= zmax)
            output_cloud->push_back(point);
    }
}

template<typename PointT, typename T>
void perception::zAxisCompression(boost::shared_ptr<pcl::PointCloud< PointT>> cloud, T factor_z)
{
    for (size_t i=0; i<cloud->size(); i++)
        cloud->points[i].z /= factor_z;
}

#endif