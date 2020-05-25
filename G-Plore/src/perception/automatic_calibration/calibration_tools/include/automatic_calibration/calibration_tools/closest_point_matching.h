#ifndef _CLOSEST_POINT_MATCHING_
#define _CLOSEST_POINT_MATCHING_

// pcl
#include <pcl/point_cloud.h>
#include <pcl/registration/icp.h>


namespace global {
namespace perception {
namespace calibration {

template<typename PointSource, typename PointTarget>
bool iterativeClosestPoint(::boost::shared_ptr<::pcl::PointCloud<PointSource>> source_cloud,
::boost::shared_ptr<::pcl::PointCloud<PointTarget>> target_cloud, float& score, Eigen::Matrix4f& transform_matrix);

template<typename PointT>
void removeInfinitePoint(::boost::shared_ptr<::pcl::PointCloud<PointT>> cloud);

}   // calibration
}   // perception
}   // global

template<typename PointSource, typename PointTarget>
bool global::perception::calibration::iterativeClosestPoint(::boost::shared_ptr<::pcl::PointCloud<PointSource>> source_cloud,
::boost::shared_ptr<::pcl::PointCloud<PointTarget>> target_cloud, float& score, Eigen::Matrix4f& transform_matrix)
{
    ::pcl::IterativeClosestPoint<PointSource, PointTarget> icp;
    ::pcl::PointCloud<PointSource> cloud_source_registered;
    icp.setInputCloud(source_cloud);
    icp.setInputTarget(target_cloud);
    // Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
    icp.setMaxCorrespondenceDistance (0.2);
    // Set the maximum number of iterations (criterion 1)
    icp.setMaximumIterations (999);
    // Set the transformation epsilon (criterion 2)
    icp.setTransformationEpsilon (1e-8);
    // Set the euclidean distance difference epsilon (criterion 3)
    icp.setEuclideanFitnessEpsilon (10);
    icp.align(cloud_source_registered);
    //Return the state of convergence after the last align run. 
    //If the two PointClouds align correctly then icp.hasConverged() = 1 (true). 
    // std::cout << "has converged: " << icp.hasConverged() <<std::endl;

    //Obtain the Euclidean fitness score (e.g., sum of squared distances from the source to the target) 
    score = icp.getFitnessScore();
    // std::cout << "score: " <<icp.getFitnessScore() << std::endl; 
    // std::cout << "----------------------------------------------------------"<< std::endl;

    //Get the final transformation matrix estimated by the registration method. 
    // std::cout << icp.getFinalTransformation() << std::endl;
    transform_matrix = icp.getFinalTransformation();
    return icp.hasConverged();
};

template<typename PointT>
void global::perception::calibration::removeInfinitePoint(::boost::shared_ptr<::pcl::PointCloud<PointT>> cloud)
{
    typename ::pcl::PointCloud<PointT>::iterator it = cloud->points.begin();
    while (it != cloud->points.end())
    {
        float x, y, z;
        x = it->x;
        y = it->y;
        z = it->z;
        if (!pcl_isfinite(x) || !pcl_isfinite(y) || !pcl_isfinite(z))
        {
            it = cloud->points.erase(it);
        }
        else
            ++it;
    }
}

#endif