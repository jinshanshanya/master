//Eigen
#include <Eigen/Dense>

// pcl
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>


namespace perception
{
    void pointcloudTransformationAffine3f(pcl::PointCloud<pcl::PointXYZI>& lidar, Eigen::Matrix4f& transformation_matrix,
    float translation_x = 0.0, float translation_y = 0.0, float translation_z = 0.0,
    float rotation_x = 0.0, float rotation_y = 0.0, float rotation_z = 0.0);
    void pointcloudTransformationAngleAxisTranslation3f(pcl::PointCloud<pcl::PointXYZI>& lidar, Eigen::Matrix4f& transformation_matrix,
    float translation_x = 0.0, float translation_y = 0.0, float translation_z = 0.0,
    float rotation_x = 0.0, float rotation_y = 0.0, float rotation_z = 0.0);
    void getXRotatedMatrix4f(Eigen::Matrix4f& rotated_matrix, double theta=0.0);
    void getYRotatedMatrix4f(Eigen::Matrix4f& rotated_matrix, double theta=0.0);
    void getZRotatedMatrix4f(Eigen::Matrix4f& rotated_matrix, double theta=0.0);
    void getTranslationMatrix4f(Eigen::Matrix4f& rotated_matrix, double x=0.0, double y=0.0, double z=0.0);
    void pointcloudTransformation(pcl::PointCloud<pcl::PointXYZI>& lidar, Eigen::Matrix4f& transformation_matrix,
    float translation_x = 0.0, float translation_y = 0.0, float translation_z = 0.0,
    float rotation_x = 0.0, float rotation_y = 0.0, float rotation_z = 0.0);
}