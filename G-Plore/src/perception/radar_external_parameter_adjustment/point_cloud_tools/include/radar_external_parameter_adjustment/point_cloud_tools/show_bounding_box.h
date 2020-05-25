#ifndef _SHOW_BOUNDING_BOX_
#define _SHOW_BOUNDING_BOX_

// pcl
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace global {
namespace perception {
namespace radar {

void drawLine(pcl::PointXYZ point1, pcl::PointXYZ point2, pcl::PointCloud<pcl::PointXYZRGB>& bounding_box,
uint8_t red=255, uint8_t green=255, uint8_t blue=255);

void draw3DBoundingBox(pcl::PointXYZ dcube[8], pcl::PointCloud<pcl::PointXYZRGB>& bounding_box,
uint8_t red=255, uint8_t green=255, uint8_t blue=255);


}   // global
}   // perception
}   // radar

#endif