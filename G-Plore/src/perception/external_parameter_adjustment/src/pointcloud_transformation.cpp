#include "external_parameter_adjustment/pointcloud_transformation.h"

void perception::pointcloudTransformationAffine3f(pcl::PointCloud<pcl::PointXYZI>& lidar, Eigen::Matrix4f& transformation_matrix,
float translation_x, float translation_y, float translation_z,
float rotation_x, float rotation_y, float rotation_z)
{
    rotation_x = -rotation_x * M_PI / 180;
    rotation_y = -rotation_y * M_PI / 180;
    rotation_z = -rotation_z * M_PI / 180;
    translation_x = -translation_x;
    translation_y = -translation_y;
    translation_z = -translation_z;
    Eigen::Affine3f transform_affine3f = Eigen::Affine3f::Identity();
    transform_affine3f.translation() << translation_x, translation_y, translation_z;
     transform_affine3f.rotate(Eigen::AngleAxisf(rotation_x, Eigen::Vector3f::UnitX())
     *Eigen::AngleAxisf(rotation_y, Eigen::Vector3f::UnitY())
     *Eigen::AngleAxisf(rotation_z, Eigen::Vector3f::UnitZ()));
     transformation_matrix = transform_affine3f.matrix();
     pcl::transformPointCloud(lidar, lidar, transformation_matrix);
}

void perception::pointcloudTransformationAngleAxisTranslation3f(pcl::PointCloud<pcl::PointXYZI>& lidar, Eigen::Matrix4f& transformation_matrix,
float translation_x, float translation_y, float translation_z,
float rotation_x, float rotation_y, float rotation_z)
{
    rotation_x = -rotation_x * M_PI / 180;
    rotation_y = -rotation_y * M_PI / 180;
    rotation_z = -rotation_z * M_PI / 180;
    translation_x = -translation_x;
    translation_y = -translation_y;
    translation_z = -translation_z;
    Eigen::AngleAxisf angle_axis_x(rotation_x, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf angle_axis_y(rotation_y, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf angle_axis_z(rotation_z, Eigen::Vector3f::UnitZ());
    Eigen::Translation3f translation3f(translation_x, translation_y, translation_z);
    transformation_matrix = (translation3f * angle_axis_x * angle_axis_y * angle_axis_z).matrix();
    pcl::transformPointCloud(lidar, lidar, transformation_matrix);
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

void perception::pointcloudTransformation(pcl::PointCloud<pcl::PointXYZI>& lidar, Eigen::Matrix4f& transformation_matrix,
float translation_x, float translation_y, float translation_z,
float rotation_x, float rotation_y, float rotation_z)
{
    rotation_x = -rotation_x * M_PI / 180;
    rotation_y = -rotation_y * M_PI / 180;
    rotation_z = -rotation_z * M_PI / 180;
    translation_x = -translation_x;
    translation_y = -translation_y;
    translation_z = -translation_z;
    Eigen::Matrix4f translation_matrix, rotation_matrix_x, rotation_matrix_y, rotation_matrix_z;
    getXRotatedMatrix4f(rotation_matrix_x, rotation_x);
    getYRotatedMatrix4f(rotation_matrix_y, rotation_y);
    getZRotatedMatrix4f(rotation_matrix_z, rotation_z);
    getTranslationMatrix4f(translation_matrix, translation_x, translation_y, translation_z);
    transformation_matrix = translation_matrix * rotation_matrix_x * rotation_matrix_y * rotation_matrix_z;
    pcl::transformPointCloud(lidar, lidar , transformation_matrix);
}