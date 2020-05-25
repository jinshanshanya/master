#include <iostream>
#include <iomanip>
#include <Eigen/Dense>

int main(int argc, char** argv)
{
    Eigen::AngleAxisd V1(M_PI, Eigen::Vector3d(0,0,1));
    std::cout << "Rotation vector1: " << std::endl << V1.matrix() << std::endl;
    Eigen::Matrix3d matrix1 = Eigen::Matrix3d::Identity();
    std::cout << "matrix: " << std::endl << matrix1 << std::endl;
    Eigen::Vector3d vector1(1,1,1);
    std::cout << "vector1:" << std::endl << vector1 << std::endl;
    Eigen::AngleAxisd rotation_vector(M_PI/4, Eigen::Vector3d(0,0,1));
    Eigen::Matrix3d rotation_matrix = rotation_vector.toRotationMatrix();
    std::cout << "rotation matrix is:" << std::endl << std::fixed << std::setprecision(4) << rotation_matrix << std::endl;
    Eigen::Vector3d rotation_vector1 = rotation_vector * vector1;
    std::cout << "vector1 after rotation is: " << std::endl << rotation_vector1.transpose() << std::endl;
    return 0;
}