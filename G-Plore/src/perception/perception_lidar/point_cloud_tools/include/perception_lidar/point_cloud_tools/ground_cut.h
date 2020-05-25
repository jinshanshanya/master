#ifndef _GROUND_CUT_
#define _GROUND_CUT_

// pcl
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// #include <pcl/PointIndices.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/filters/extract_indices.h>

namespace perception
{
    template<typename PointT, typename T>
    void groundCutMorphologicalFilter(boost::shared_ptr<pcl::PointCloud<PointT>> lidar_input,
    boost::shared_ptr<pcl::PointCloud<PointT>> lidar_groundless, boost::shared_ptr<pcl::PointCloud<PointT>> ground,
    T max_window_size,  T max_distance, T init_distance, T cell_size, T slope, T base, bool exponential);
}

template<typename PointT, typename T>
void perception::groundCutMorphologicalFilter(boost::shared_ptr<pcl::PointCloud<PointT>> lidar_input,
boost::shared_ptr<pcl::PointCloud<PointT>> lidar_groundless, boost::shared_ptr<pcl::PointCloud<PointT>> ground,
T max_window_size, T max_distance, T init_distance, T cell_size, T slope, T base, bool exponential)
{
    pcl::PointIndicesPtr groundIndices(new pcl::PointIndices);
    pcl::ProgressiveMorphologicalFilter<PointT> pmf;
    pmf.setInputCloud(lidar_input);
    pmf.setMaxWindowSize(max_window_size);
    pmf.setMaxDistance(max_distance);
    pmf.setInitialDistance(init_distance);
    pmf.setCellSize(cell_size);
    pmf.setSlope(slope);
    pmf.setBase(base);
    pmf.setExponential(exponential);
    pmf.extract(groundIndices->indices);

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(lidar_input);
    extract.setIndices(groundIndices);
    extract.filter(*ground);
    extract.setNegative(true);
    extract.filter(*lidar_groundless);
}

#endif