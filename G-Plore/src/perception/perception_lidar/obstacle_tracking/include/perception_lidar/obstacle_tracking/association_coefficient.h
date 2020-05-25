#ifndef _ASSOCIATION_COEFFICIENT_
#define _ASSOCIATION_COEFFICIENT_

// point cloud tools
#include "perception_lidar/point_cloud_tools/base_data.h"

namespace perception
{
    template<typename T>
    T getIOU(Point2D<T> box1[2], Point2D<T> box2[2]);
}

template<typename T>
T perception::getIOU(Point2D<T> box0[2], Point2D<T> box1[2])
{
    // left down, right up
    T len_x_0 = box0[1].x_ - box0[0].x_;
    T len_y_0 = box0[1].y_ - box0[0].y_;
    T len_x_1 = box1[1].x_ - box1[0].x_;
    T len_y_1 = box1[1].y_ - box1[0].y_;
    T x_range = std::max(box0[1].x_, box1[1].x_) - std::min(box0[0].x_, box1[0].x_);
    T y_range = std::max(box0[1].y_, box1[1].y_) - std::min(box0[0].y_, box1[0].y_);
    T x_overlap = len_x_0 + len_x_1 - x_range;
    T y_overlap = len_y_0 + len_y_1 - y_range;
    if (x_overlap < 0 || y_overlap < 0)
        return (T)0;
    T area_intersection = x_overlap *  y_overlap;
    T area_union = len_x_0 * len_y_0 + len_x_1 * len_y_1 - area_intersection;
    T iou = area_intersection / area_union;
    return iou;
}

#endif
