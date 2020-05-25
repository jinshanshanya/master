#ifndef _RADAR_TOOLS_
#define _RADAR_TOOLS_

// c++
#include <float.h>

// Eigen
#include <Eigen/Dense>

// radar data
#include "perception_radar/radar_tools/radar_data.h"

namespace global {
namespace perception {
namespace radar {

class Radar_Object
{
    public:
        // constructor
        Radar_Object(){ };
        Radar_Object(const Rad_Obj& rad_obj, size_t offset,
        Eigen::Matrix4f& transform_matirx, const uint8_t&  radar_id);
        // destructor
        virtual ~Radar_Object(){ };
        // function
        inline float getxMax()
        {
            float xmax = bounding_box_[0].x_;
            for (size_t i=1; i<4; i++)
                if (xmax < bounding_box_[i].x_)
                    xmax = bounding_box_[i].x_;
            xmax_ = xmax;
            return xmax;
        };
        inline float getyMax()
        {
            float ymax = bounding_box_[0].y_;
            for (size_t i=1; i<4; i++)
                if (ymax < bounding_box_[i].y_)
                    ymax = bounding_box_[i].y_;
            ymax_ = ymax;
            return ymax;
        };
        inline float getxMin()
        {
            float xmin = bounding_box_[0].x_;
            for (size_t i=1; i<4; i++)
                if (xmin > bounding_box_[i].x_)
                    xmin = bounding_box_[i].x_;
            xmin_ = xmin;
            return xmin;
        };
        inline float getyMin()
        {
            float ymin = bounding_box_[0].y_;
            for (size_t i=1; i<4; i++)
                if (ymin > bounding_box_[i].y_)
                    ymin = bounding_box_[i].y_;
            ymin_ = ymin;
            return ymin;
        };
        // variable
        Point3D<float> radar_point_;
        Point3D<float> velocity_;
        Point3D<float> acceleration_;
        Point3D<float> radar_point_stdev_;
        Point3D<float> velocity_stdev_;
        Point3D<float> acceleration_stdev_;
        Angle3D<float> angle_;
        Angle3D<float> angle_stdev_;
        float length_;
        float width_;
        float height_;
        float rcs_;
        float xmin_, xmax_, ymin_, ymax_, zmin_, zmax_; 
        Radar_Obj_Type radar_obj_type_;
        Radar_Obj_Exist_Prob radar_obj_exist_prob_;
        Radar_Meas_State radar_meas_state_;
        Radar_DynProp radar_dyn_prop_;
        Point3D<float> bounding_box_[8];
        // uint8_t radar_id_;
        uint32_t object_id_;
        Eigen::Matrix4f transform_matrix_;
    protected:
        // function
        void extractObjectFeature();
        void transformFeature();
};

void removeDuplication(std::list<Radar_Object>& object_list, const float threshold);

float getIOU(Radar_Object& object1, Radar_Object& object2);

}   // namespace radar
}   // namespace perception
}   // namespace global

#endif