#include "perception_radar/radar_tools/radar_tools.h"

global::perception::radar::Radar_Object::Radar_Object(const Rad_Obj& rad_obj,
size_t offset, Eigen::Matrix4f& transform_matrix, const uint8_t&  radar_id):
radar_point_(rad_obj.Long_Pos[offset], rad_obj.Lat_Pos[offset], 0.0),
velocity_(rad_obj.Long_Vel[offset], rad_obj.Lat_Vel[offset], 0.0),
acceleration_(rad_obj.Long_Acc[offset], rad_obj.Lat_Acc[offset], 0.0),
radar_point_stdev_(rad_obj.Long_Pos_Stdev[offset], rad_obj.Lat_Pos_Stdev[offset], 0.0),
velocity_stdev_(rad_obj.Long_Vel_Stdev[offset], rad_obj.Lat_Vel_Stdev[offset], 0.0),
acceleration_stdev_(rad_obj.Long_Acc_Stdev[offset], rad_obj.Lat_Acc_Stdev[offset], 0.0),
angle_(0.0, 0.0, rad_obj.Orientation_Angle[offset]),
angle_stdev_(0.0, 0.0, rad_obj.Orientation_Stdev[offset]),
length_(rad_obj.Length[offset]),
width_(rad_obj.Width[offset]),
height_(2.0),
rcs_(rad_obj.RCS[offset]),
radar_obj_type_(rad_obj.Type[offset]),
radar_obj_exist_prob_(rad_obj.Prob_Exist[offset]),
radar_meas_state_(rad_obj.Meas_State[offset]),
radar_dyn_prop_(rad_obj.Dyn_Prop[offset]),
// radar_id_(radar_id),
object_id_((uint32_t)rad_obj.ID[offset]+256*(uint32_t)radar_id),
xmin_(rad_obj.Long_Pos[offset]),
xmax_(rad_obj.Long_Pos[offset] + rad_obj.Length[offset]),
ymin_(rad_obj.Lat_Pos[offset] - rad_obj.Width[offset] / 2),
ymax_(rad_obj.Lat_Pos[offset] + rad_obj.Width[offset] / 2),
zmin_(-1.0),
zmax_(1.0),
transform_matrix_(transform_matrix)
{

    // extract bounding box
    extractObjectFeature();
    // transform
    transformFeature();
};

void global::perception::radar::Radar_Object::extractObjectFeature()
{
    bounding_box_[0].x_ = xmax_;
    bounding_box_[0].y_ = ymax_;
    bounding_box_[0].z_ = zmax_;
    bounding_box_[1].x_ = xmin_;
    bounding_box_[1].y_ = ymax_;
    bounding_box_[1].z_ = zmax_;
    bounding_box_[2].x_ = xmin_;
    bounding_box_[2].y_ = ymin_;
    bounding_box_[2].z_ = zmax_;
    bounding_box_[3].x_ = xmax_;
    bounding_box_[3].y_ = ymin_;
    bounding_box_[3].z_ = zmax_;
    bounding_box_[4].x_ = xmax_;
    bounding_box_[4].y_ = ymax_;
    bounding_box_[4].z_ = zmin_;
    bounding_box_[5].x_ = xmin_;
    bounding_box_[5].y_ = ymax_;
    bounding_box_[5].z_ = zmin_;
    bounding_box_[6].x_ = xmin_;
    bounding_box_[6].y_ = ymin_;
    bounding_box_[6].z_ = zmin_;
    bounding_box_[7].x_ = xmax_;
    bounding_box_[7].y_ = ymin_;
    bounding_box_[7].z_ = zmin_;
}

 void global::perception::radar::Radar_Object::transformFeature()
 {
     radar_point_.transform(transform_matrix_);
     velocity_.transform(transform_matrix_);
     acceleration_.transform(transform_matrix_);
     for (size_t i=0; i<8; i++)
        bounding_box_[i].transform(transform_matrix_);
    radar_point_stdev_.transform(transform_matrix_);
    velocity_stdev_.transform(transform_matrix_);
    acceleration_stdev_.transform(transform_matrix_);
 }

void global::perception::radar::removeDuplication(std::list<Radar_Object>& object_list, const float threshold)
{
    for (std::list<Radar_Object>::iterator iter1 = object_list.begin();
    iter1 != object_list.end(); iter1++)
        for (std::list<Radar_Object>::iterator iter2 = iter1;
        iter2 != object_list.end(); )
        {
            if (iter1 != iter2)
            {
                float iou = getIOU(*iter1, *iter2);
                if (iou > threshold)
                    iter2 = object_list.erase(iter2);
                else
                    iter2++;
            }
            else
                iter2++;
        }
}

float global::perception::radar::getIOU(Radar_Object& object1, Radar_Object& object2)
{
    float x1min = object1.getxMin(), x1max = object1.getxMax(), y1min = object1.getyMin(), y1max = object1.getyMax();
    float x2min = object2.getxMin(), x2max = object2.getxMax(), y2min = object2.getyMin(), y2max = object2.getyMax();
    float len_x1 = x1max - x1min, len_y1 = y1max - y1min, len_x2 = x2max - x2min, len_y2 = y2max - y2min;
    float range_x = std::max(x1max, x2max) - std::min(x1min, x2min), range_y = std::max(y1max, y2max) - std::min(y1min, y2min);
    float overlap_x = len_x1 + len_x2 - range_x, overlap_y = len_y1 + len_y2 - range_y;
    if (overlap_x <=0 || overlap_y <=0)
        return 0.0;
    float area_intersection = overlap_x * overlap_y;
    float area_union = len_x1 * len_y1 + len_x2 * len_y2 - area_intersection;
    float iou = area_intersection / area_union;
    return iou;
}