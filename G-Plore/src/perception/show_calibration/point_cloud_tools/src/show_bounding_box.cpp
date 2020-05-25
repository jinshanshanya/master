#include "show_calibration/point_cloud_tools/show_bounding_box.h"

void global::perception::radar::drawLine(pcl::PointXYZ point1, pcl::PointXYZ point2, pcl::PointCloud<pcl::PointXYZRGB>& bounding_box,
uint8_t red, uint8_t green, uint8_t blue)
{
    float r = sqrt(pow(point2.x-point1.x, 2) + pow(point2.y-point1.y, 2) + pow(point2.z-point1.z, 2));
    uint32_t rgb = ((uint32_t)red<<16 | (uint32_t)green<<8 | (uint32_t)blue);
    for (float i = 0; i <= r; i+=0.05)
    {
        pcl::PointXYZRGB point;
        point.x = point1.x + i * (point2.x-point1.x) / r;
        point.y = point1.y + i * (point2.y-point1.y) / r;
        point.z = point1.z + i * (point2.z-point1.z) / r;
        point.rgb = *reinterpret_cast<float*>(&rgb);
        bounding_box.push_back(point);
        // std::cout << "i = " << i << std::endl;
    }
}

void global::perception::radar::draw3DBoundingBox(pcl::PointXYZ dcube[8], pcl::PointCloud<pcl::PointXYZRGB>& bounding_box,
uint8_t red, uint8_t green, uint8_t blue)
{
    // std::cout << "point 1" << std::endl;
    drawLine(dcube[0], dcube[1], bounding_box, red, green, blue);
    // std::cout << "point 2" << std::endl;
    drawLine(dcube[1], dcube[2], bounding_box, red, green, blue);
    // std::cout << "point 3" << std::endl;
    drawLine(dcube[2], dcube[3], bounding_box, red, green, blue);
    // std::cout << "point 4" << std::endl;
    drawLine(dcube[3], dcube[0], bounding_box, red, green, blue);

    // std::cout << "point 5" << std::endl;
    drawLine(dcube[4], dcube[5], bounding_box, red, green, blue);
    // std::cout << "point 6" << std::endl;
    drawLine(dcube[5], dcube[6], bounding_box, red, green, blue);
    // std::cout << "point 7" << std::endl;
    drawLine(dcube[6], dcube[7], bounding_box, red, green, blue);
    // std::cout << "point 8" << std::endl;
    drawLine(dcube[7], dcube[4], bounding_box, red, green, blue);

    // std::cout << "point 9" << std::endl;
    drawLine(dcube[0], dcube[4], bounding_box, red, green, blue);
    // std::cout << "point 10" << std::endl;
    drawLine(dcube[1], dcube[5], bounding_box, red, green, blue);
    // std::cout << "point 11" << std::endl;
    drawLine(dcube[2], dcube[6], bounding_box, red, green, blue);
    // std::cout << "point 12" << std::endl;
    drawLine(dcube[3], dcube[7], bounding_box, red, green, blue);
}