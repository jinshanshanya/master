#ifndef _SHOW_BOUNDING_BOX_
#define  _SHOW_BOUNDING_BOX_

// pcl
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

// point_cloud_tools
#include "perception_lidar/point_cloud_tools/base_data.h"

namespace perception
{
    template<typename T>
    void drawLine(Point3D<T> point1, Point3D<T> point2, pcl::PointCloud<pcl::PointXYZRGB>& bounding_box,
    uint8_t red=255, uint8_t green=255, uint8_t blue=255);

    template<typename T>
    void draw3DBoundingBox(Point3D<T> dcube[8], pcl::PointCloud<pcl::PointXYZRGB>& bounding_box,
    uint8_t red=255, uint8_t green=255, uint8_t blue=255);
}

template<typename T>
void perception::drawLine(Point3D<T> point1, Point3D<T> point2, pcl::PointCloud<pcl::PointXYZRGB>& bounding_box,
uint8_t red, uint8_t green, uint8_t blue)
{
    float r = sqrt(pow(point2.x_-point1.x_, 2) + pow(point2.y_-point1.y_, 2) + pow(point2.z_-point1.z_, 2));
    uint32_t rgb = ((uint32_t)red<<16 | (uint32_t)green<<8 | (uint32_t)blue);
    for (float i = 0; i < r; i+=0.05)
    {
        pcl::PointXYZRGB point;
        point.x = point1.x_ + i * (point2.x_-point1.x_) / r;
        point.y = point1.y_ + i * (point2.y_-point1.y_) / r;
        point.z = point1.z_ + i * (point2.z_-point1.z_) / r;
        point.rgb = *reinterpret_cast<float*>(&rgb);
        bounding_box.push_back(point);
        // std::cout << "i = " << i << std::endl;
    }
}

template<typename T>
void perception::draw3DBoundingBox(Point3D<T> dcube[8], pcl::PointCloud<pcl::PointXYZRGB>& bounding_box,
uint8_t red, uint8_t green, uint8_t blue)
{
    // std::cout << "point 1" << std::endl;
    perception::drawLine<T>(dcube[0], dcube[1], bounding_box, red, green, blue);
    // std::cout << "point 2" << std::endl;
    perception::drawLine<T>(dcube[1], dcube[2], bounding_box, red, green, blue);
    // std::cout << "point 3" << std::endl;
    perception::drawLine<T>(dcube[2], dcube[3], bounding_box, red, green, blue);
    // std::cout << "point 4" << std::endl;
    perception::drawLine<T>(dcube[3], dcube[0], bounding_box, red, green, blue);

    // std::cout << "point 5" << std::endl;
    perception::drawLine<T>(dcube[4], dcube[5], bounding_box, red, green, blue);
    // std::cout << "point 6" << std::endl;
    perception::drawLine<T>(dcube[5], dcube[6], bounding_box, red, green, blue);
    // std::cout << "point 7" << std::endl;
    perception::drawLine<T>(dcube[6], dcube[7], bounding_box, red, green, blue);
    // std::cout << "point 8" << std::endl;
    perception::drawLine<T>(dcube[7], dcube[4], bounding_box, red, green, blue);

    // std::cout << "point 9" << std::endl;
    perception::drawLine<T>(dcube[0], dcube[4], bounding_box, red, green, blue);
    // std::cout << "point 10" << std::endl;
    perception::drawLine<T>(dcube[1], dcube[5], bounding_box, red, green, blue);
    // std::cout << "point 11" << std::endl;
    perception::drawLine<T>(dcube[2], dcube[6], bounding_box, red, green, blue);
    // std::cout << "point 12" << std::endl;
    perception::drawLine<T>(dcube[3], dcube[7], bounding_box, red, green, blue);
}

#endif