#ifndef _FEATURE_EXTRACTION_
#define _FEATURE_EXTRACTION_

// pcl
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/search/search.h>

// c++
// #include <limits>
#include <cfloat>

// point_cloud_tools
#include "perception_lidar/point_cloud_tools/base_data.h"

namespace perception
{
    // eculidean cluster
    template<typename PointT>
    void euclideanCluster(std::vector<pcl::PointIndices>& cluster_indices,  boost::shared_ptr<pcl::PointCloud<PointT>> lidar,
    float cluster_tolerance = 0.4, int min_cluster_size = 3, int max_cluster_size = 30000);

    // region growing
    template<typename PointT, typename NormalT>
    void regionGrowing(std::vector<pcl::PointIndices>& cluster_indices, boost::shared_ptr<pcl::PointCloud<PointT>> lidar,
    const uint32_t k_search, const uint32_t min_cluster_size, const uint32_t max_cluster_size, const uint32_t neighbour_number,
    const float smoothness_threshold, const float curvature_threshold);

    template<typename PointT>
    bool enforceIntensitySimilarity(const PointT& point_a, const PointT& point_b, float squared_distance);

    template<typename PointT>
    bool enforceCurvatureOrIntensitySimilarity(const PointT& point_a, const PointT& point_b, float squared_distance);

    template<typename PointT>
    bool customRegionGrowing(const PointT& point_a, const PointT& point_b, float squared_distance);

    template<typename PointT, typename PointNT>
    void conditionalEculideanCluster(std::vector<pcl::PointIndices>& cluster_indices, boost::shared_ptr<pcl::PointCloud<PointT>> lidar,
    const float search_radius = 0.4f, const float cluster_tolerance = 0.5f, const uint32_t min_cluster_size = 10,
    const uint32_t max_cluster_size = 30000);

    template<typename PointT, typename T>
    class Obstacle_Feature_Point_Cloud
    {
        public:
            // constructor
            Obstacle_Feature_Point_Cloud(){ };
            Obstacle_Feature_Point_Cloud(Cube<T> cube, Point3D<T> dcube[8])
            :
            cube_(cube)
            {
                for (size_t i = 0; i < 8; i++)
                    dcube_[i] = dcube[i];
            };
            // destructor
            ~ Obstacle_Feature_Point_Cloud(){ };
            // function
            inline void setDCube(Point3D<T> dcube[8]);
            inline void setCube(Cube<T> cube);
            void getXYZmaxmin(pcl::PointIndices& point_indices, pcl::PointCloud<pcl::PointXYZI>::Ptr lidar,  float factor_z);
            void getminRectangleArea(pcl::PointIndices& point_indices, pcl::PointCloud<pcl::PointXYZI>::Ptr lidar);
            // variable
            Point3D<T> dcube_[8];
            Cube<T> cube_;
            bool label_tracking_ = false;
        protected:
            T getRectangleArea(pcl::PointIndices& point_indices, pcl::PointCloud<pcl::PointXYZI>::Ptr lidar, perception::Point2D<T> rectangle[4], T theta=0.0);
    };

    // extract the feature of all obstacles
    template<typename PointT, typename T>
    void getObstaclesFeature(std::vector<Obstacle_Feature_Point_Cloud<PointT, T>>& obstacles_feature,
    std::vector<pcl::PointIndices>& cluster_indices, pcl::PointCloud<pcl::PointXYZI>::Ptr lidar, float factor_z);
}

template<typename PointT>
void perception::euclideanCluster(std::vector<pcl::PointIndices>& cluster_indices, boost::shared_ptr<pcl::PointCloud<PointT>> lidar,
float cluster_tolerance , int min_cluster_size, int max_cluster_size)
{
    boost::shared_ptr<pcl::search::KdTree<PointT>> tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(lidar);
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(cluster_tolerance);
    ec.setMinClusterSize(min_cluster_size);
    ec.setMaxClusterSize(max_cluster_size);
    ec.setSearchMethod(tree);
    ec.setInputCloud(lidar);
    ec.extract(cluster_indices);
}

template<typename PointT, typename NormalT>
void perception::regionGrowing(std::vector<pcl::PointIndices>& cluster_indices, boost::shared_ptr<pcl::PointCloud<PointT>> lidar,
const uint32_t k_search, const uint32_t min_cluster_size, const uint32_t max_cluster_size, const uint32_t neighbour_number,
const float smoothness_threshold, const float curvature_threshold)
{
    ::boost::shared_ptr<::pcl::search::KdTree<PointT>> tree(new ::pcl::search::KdTree<PointT>);
    ::boost::shared_ptr<::pcl::PointCloud<NormalT>> normals(new ::pcl::PointCloud<NormalT>);
    ::pcl::NormalEstimation<PointT, NormalT> normal_estimator;
    normal_estimator.setSearchMethod(tree);
    normal_estimator.setInputCloud(lidar);
    normal_estimator.setKSearch(k_search);
    normal_estimator.compute(*normals);
    pcl::IndicesPtr indices(new std::vector<int>);
    ::pcl::RegionGrowing<PointT, NormalT> reg;
    reg.setMinClusterSize(min_cluster_size);
    reg.setMaxClusterSize(max_cluster_size);
    reg.setSearchMethod(tree);
    reg.setNumberOfNeighbours(neighbour_number);
    reg.setInputCloud(lidar);
    reg.setInputNormals(normals);
    reg.setSmoothnessThreshold(smoothness_threshold);
    reg.setCurvatureThreshold(curvature_threshold);
    reg.extract(cluster_indices);
}

template<typename PointT>
bool perception::enforceIntensitySimilarity(const PointT& point_a, const PointT& point_b, float squared_distance)
{
    if (std::abs(point_a.intensity-point_b.intensity)<5.0f)
        return (true);
    else
        return (false);    
}

template<typename PointT>
bool  perception::enforceCurvatureOrIntensitySimilarity(const PointT& point_a, const PointT& point_b, float squared_distance)
{
    Eigen::Map<const Eigen::Vector3f> point_a_normal = point_a.getNormalVector3fMap();
    Eigen::Map<const Eigen::Vector3f> point_b_normal = point_b.getNormalVector3fMap();
    if (std::abs(point_a.intensity-point_b.intensity)<5.0f)
        return (true);
    if (std::abs(point_a_normal.dot(point_b_normal))<0.05)
        return (true);
    return (false);
}

template<typename PointT>
bool perception::customRegionGrowing(const PointT& point_a, const PointT& point_b, float squared_distance)
{
    Eigen::Map<const Eigen::Vector3f> point_a_normal = point_a.getNormalVector3fMap();
    Eigen::Map<const Eigen::Vector3f> point_b_normal = point_b.getNormalVector3fMap();
    if (squared_distance < 10000)
    {
        if (std::abs(point_a.intensity - point_b.intensity) < 8.0f)
            return (true);
        if (std::abs(point_a_normal.dot(point_b_normal))<0.06)
            return (true);
    }
    else
    {
        if (std::abs(point_a.intensity-point_b.intensity)<3.0f)
            return (true);
    }
    return (false);
}

template<typename PointT, typename PointNT>
void perception::conditionalEculideanCluster(std::vector<pcl::PointIndices>& cluster_indices, boost::shared_ptr<pcl::PointCloud<PointT>> lidar,
const float search_radius, const float cluster_tolerance, const uint32_t min_cluster_size,
const uint32_t max_cluster_size)
{
    ::boost::shared_ptr<pcl::PointCloud<PointNT>> cloud_with_normals(new ::pcl::PointCloud<PointNT>);
    ::boost::shared_ptr<::pcl::search::KdTree<PointT>> search_tree(new ::pcl::search::KdTree<PointT>);
    ::pcl::IndicesClustersPtr small_clusters(new ::pcl::IndicesClusters), large_clusters(new ::pcl::IndicesClusters);
    ::pcl::copyPointCloud(*lidar, *cloud_with_normals);
    ::pcl::NormalEstimation<PointT, PointNT> ne;
    ne.setInputCloud(lidar);
    ne.setSearchMethod(search_tree);
    ne.setRadiusSearch(search_radius);
    ne.compute(*cloud_with_normals);
    ::pcl::ConditionalEuclideanClustering<PointNT> cec(true);
    cec.setInputCloud(cloud_with_normals);
    cec.setConditionFunction(&customRegionGrowing);
    cec.setClusterTolerance(cluster_tolerance);
    cec.setMinClusterSize(min_cluster_size);
    cec.setMaxClusterSize(max_cluster_size);
    cec.segment(cluster_indices);
    cec.getRemovedClusters(small_clusters, large_clusters);
}

template<typename PointT, typename T>
void perception::getObstaclesFeature(std::vector<perception::Obstacle_Feature_Point_Cloud<PointT, T>>& obstacles_feature,
std::vector<pcl::PointIndices>& cluster_indices, pcl::PointCloud<pcl::PointXYZI>::Ptr lidar, float factor_z)
{
    for (int i = 0; i < cluster_indices.size(); i++)
    {
        perception::Obstacle_Feature_Point_Cloud<PointT, T> obstacle_feature;
        obstacles_feature.push_back(obstacle_feature);
        obstacles_feature[i].getXYZmaxmin(cluster_indices[i], lidar, factor_z);
        obstacles_feature[i].getminRectangleArea(cluster_indices[i], lidar);
    }
}

template<typename PointT, typename T>
inline void perception::Obstacle_Feature_Point_Cloud<PointT, T>::setDCube(Point3D<T> dcube[8])
{
    for (size_t i = 0; i < 8; i++)
        dcube_[i] = dcube[i];
};

template<typename PointT, typename T>
inline void perception::Obstacle_Feature_Point_Cloud<PointT, T>::setCube(Cube<T> cube)
{
    cube_ = cube;
}

template<typename PointT, typename T>
void perception::Obstacle_Feature_Point_Cloud<PointT, T>::getXYZmaxmin(pcl::PointIndices& point_indices, pcl::PointCloud<pcl::PointXYZI>::Ptr lidar,  float factor_z)
{
    cube_.xmax_ = -DBL_MAX; cube_.ymax_ = -DBL_MAX;cube_. zmax_ = -DBL_MAX;
    cube_.xmin_ = DBL_MAX; cube_.ymin_ = DBL_MAX; cube_.zmin_ = DBL_MAX;
    for (std::vector<int>::const_iterator pit =point_indices.indices.begin();
    pit != point_indices.indices.end(); pit++)
    {
        float x = lidar->points[*pit].x, y = lidar->points[*pit].y, z = lidar->points[*pit].z;
        if (cube_.xmax_ < x)
            cube_.xmax_ = x;
        if (cube_.xmin_ > x)
            cube_.xmin_ = x;
        if (cube_.ymax_ < y)
            cube_.ymax_ = y;
        if (cube_.ymin_ > y)
            cube_.ymin_ = y;
        if (cube_.zmax_ < z)
            cube_.zmax_ = z;
        if (cube_.zmin_ > z)
            cube_.zmin_ = z;
    }
    cube_.zmax_ *= factor_z;
    cube_.zmin_ *= factor_z;
}

template<typename PointT, typename T>
T perception::Obstacle_Feature_Point_Cloud<PointT, T>::getRectangleArea(pcl::PointIndices& point_indices, pcl::PointCloud<pcl::PointXYZI>::Ptr lidar, perception::Point2D<T> rectangle[4], T theta)
{
    T xmin=DBL_MAX, ymin=DBL_MAX, xmax=-DBL_MAX, ymax=-DBL_MAX;
    for (std::vector<int>::const_iterator pit = point_indices.indices.begin();
    pit != point_indices.indices.end(); pit++)
    {
        Point2D<float> point_2d;
        point_2d.x_ = lidar->points[*pit].x*cos(-theta) + lidar->points[*pit].y*sin(-theta);
        point_2d.y_ = -lidar->points[*pit].x*sin(-theta) + lidar->points[*pit].y*cos(-theta);
        if (point_2d.x_ > xmax)
            xmax = point_2d.x_;
        if (point_2d.x_ < xmin)
            xmin = point_2d.x_;
        if (point_2d.y_ > ymax)
            ymax = point_2d.y_;
        if (point_2d.y_ < ymin)
            ymin = point_2d.y_;
    }
    rectangle[0].x_ = xmax*cos(theta) + ymax*sin(theta);
    rectangle[0].y_ = -xmax*sin(theta) + ymax*cos(theta);
    rectangle[1].x_ = xmin*cos(theta) + ymax*sin(theta);
    rectangle[1].y_ = -xmin*sin(theta) + ymax*cos(theta);
    rectangle[2].x_ = xmin*cos(theta) + ymin*sin(theta);
    rectangle[2].y_ = -xmin*sin(theta) + ymin*cos(theta);
    rectangle[3].x_ = xmax*cos(theta) + ymin*sin(theta);
    rectangle[3].y_ = -xmax*sin(theta) + ymin*cos(theta);
    T area = (xmax-xmin)*(ymax-ymin);
    return area;
}

template<typename PointT, typename T>
void perception::Obstacle_Feature_Point_Cloud<PointT, T>::getminRectangleArea(pcl::PointIndices& point_indices, pcl::PointCloud<pcl::PointXYZI>::Ptr lidar)
{
    T min_area = DBL_MAX;
    perception::Point2D<T> min_rectangle[4];
    for (float theta = 0; theta <= M_PI/2; theta+=M_PI/45)
    {
        perception::Point2D<T> rectangle[4];
        T area = getRectangleArea(point_indices, lidar, rectangle, theta);
        if (area < min_area)
        {
            min_area = area;
            for (int i=0; i<4; i++)
                min_rectangle[i] = rectangle[i];
        }
    }
    dcube_[0].x_ = min_rectangle[0].x_;
    dcube_[0].y_ = min_rectangle[0].y_;
    dcube_[0].z_ = cube_.zmax_;
    dcube_[1].x_ = min_rectangle[1].x_;
    dcube_[1].y_ = min_rectangle[1].y_;
    dcube_[1].z_ = cube_.zmax_;
    dcube_[2].x_ = min_rectangle[2].x_;
    dcube_[2].y_ = min_rectangle[2].y_;
    dcube_[2].z_ = cube_.zmax_;
    dcube_[3].x_ = min_rectangle[3].x_;
    dcube_[3].y_ = min_rectangle[3].y_;
    dcube_[3].z_ = cube_.zmax_;
    dcube_[4].x_ = min_rectangle[0].x_;
    dcube_[4].y_ = min_rectangle[0].y_;
    dcube_[4].z_ = cube_.zmin_;
    dcube_[5].x_ = min_rectangle[1].x_;
    dcube_[5].y_ = min_rectangle[1].y_;
    dcube_[5].z_ = cube_.zmin_;
    dcube_[6].x_ = min_rectangle[2].x_;
    dcube_[6].y_ = min_rectangle[2].y_;
    dcube_[6].z_ = cube_.zmin_;
    dcube_[7].x_ = min_rectangle[3].x_;
    dcube_[7].y_ = min_rectangle[3].y_;
    dcube_[7].z_ = cube_.zmin_;
}

#endif