#include "radar_external_parameter_adjustment/point_cloud_tools/eigen_transformation.h"

void global::perception::radar::eigenTransformationAffine3f(Eigen::Matrix4f& transformation_matrix,
float translation_x, float translation_y, float translation_z ,float rotation_x, float rotation_y, float rotation_z)
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
}
