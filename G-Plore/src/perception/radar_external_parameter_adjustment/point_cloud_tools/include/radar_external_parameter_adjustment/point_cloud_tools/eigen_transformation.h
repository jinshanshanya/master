#ifndef _eigen_transformation_
#define _eigen_transformation_

#include <Eigen/Dense>

namespace global {
namespace perception {
namespace radar {

void eigenTransformationAffine3f(Eigen::Matrix4f& transformation_matrix,
float translation_x = 0.0, float translation_y = 0.0, float translation_z = 0.0,
float rotation_x = 0.0, float rotation_y = 0.0, float rotation_z = 0.0);

}   // global
}   // perception
}   // radar

#endif

