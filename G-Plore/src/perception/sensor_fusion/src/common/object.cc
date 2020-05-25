

#include "common/object.h"

//#include "util/box2d.h"
#include "util/string_util.h"
//#include "modules/perception/common/perception_gflags.h"

namespace glb_auto_perception_sensorfusion
{

using Eigen::Vector3d;
// using glb_auto_perception_sensorfusion::util::Box2d;
// using glb_auto_perception_sensorfusion::util::Print;
// using glb_auto_perception_sensorfusion::util::StrCat;
// using glb_auto_perception_sensorfusion::util::Vec2d;

Object::Object()
{
  //cloud.reset(new pcl_util::PointCloud);
  type_probs.resize(static_cast<int>(ObjectType::MAX_OBJECT_TYPE), 0);
  position_uncertainty << 0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01;
  velocity_uncertainty << 0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01;
}

void Object::clone(const Object &rhs)
{
  *this = rhs;
  //pcl::copyPointCloud<pcl_util::Point, pcl_util::Point>(*(rhs.cloud), *cloud);
  radar_supplement = nullptr;
  if (rhs.radar_supplement != nullptr)
  {
    radar_supplement.reset(new RadarSupplement(*rhs.radar_supplement));
  }
  camera_supplement = nullptr;
  if (rhs.camera_supplement != nullptr)
  {
    camera_supplement.reset(new CameraSupplement());
    camera_supplement->clone(*(rhs.camera_supplement));
  }
}

// std::string Object::ToString() const
// {
//   // StrCat supports 9 arguments at most.
//   return StrCat(StrCat("Object[id: ", id,
//                        ", "
//                        "track_id: ",
//                        track_id,
//                        ", "
//                        "cloud_size: ",
//                        cloud->size(),
//                        ", "
//                        "direction: ",
//                        Print(direction.transpose()), ", "),
//                 StrCat("center: ", Print(center.transpose()),
//                        ", "
//                        "velocity: ",
//                        Print(velocity.transpose()),
//                        ", "
//                        "width: ",
//                        width,
//                        ", "
//                        "length: ",
//                        length, ", "),
//                 StrCat("height: ", height,
//                        ", "
//                        "polygon_size: ",
//                        polygon.size(),
//                        ", "
//                        "type: ",
//                        static_cast<int>(type),
//                        ", "
//                        "is_background: ",
//                        is_background),
//                 StrCat(", is_cipv: ", b_cipv, "]"));
// }

// Add 4 corners in the polygon
// void Object::AddFourCorners(PerceptionObstacle* pb_obj) const {
//   Box2d object_bounding_box = {{center(0), center(1)}, theta, length, width};
//   std::vector<Vec2d> corners;
//   object_bounding_box.GetAllCorners(&corners);

//   for (const auto& corner : corners) {
//     Point* p = pb_obj->add_polygon_point();
//     p->set_x(corner.x());
//     p->set_y(corner.y());
//     p->set_z(0.0);
//   }
//   ADEBUG << "PerceptionObstacle bounding box is : "
//          << object_bounding_box.DebugString();
// }

// void Object::Serialize(PerceptionObstacle* pb_obj) const {
//   CHECK(pb_obj != nullptr);
//   pb_obj->set_id(track_id);
//   pb_obj->set_theta(theta);

//   Point* obj_center = pb_obj->mutable_position();
//   obj_center->set_x(center(0));
//   obj_center->set_y(center(1));
//   obj_center->set_z(center(2));

//   Point* obj_velocity = pb_obj->mutable_velocity();
//   obj_velocity->set_x(velocity(0));
//   obj_velocity->set_y(velocity(1));
//   obj_velocity->set_z(velocity(2));

//   pb_obj->set_length(length);
//   pb_obj->set_width(width);
//   pb_obj->set_height(height);

//   if (polygon.size() /*pb_obs.polygon_point_size() */ >= 4) {
//     for (const auto& point : polygon.points) {
//       Point* p = pb_obj->add_polygon_point();
//       p->set_x(point.x);
//       p->set_y(point.y);
//       p->set_z(point.z);
//     }
//   } else {  // if polygon size is less than 4
//     // Generate polygon from center position, width, height
//     // and orientation of the object
//     AddFourCorners(pb_obj);
//   }

//   // always serialize point cloud
//   for (const auto& point : cloud->points) {
//     pb_obj->add_point_cloud(point.x);
//     pb_obj->add_point_cloud(point.y);
//     pb_obj->add_point_cloud(point.z);
//   }

//   pb_obj->set_confidence(score);
//   pb_obj->set_confidence_type(score_type);
//   pb_obj->set_tracking_time(tracking_time);
//   pb_obj->set_type(static_cast<PerceptionObstacle::Type>(type));
//   pb_obj->set_timestamp(latest_tracked_time);  // in seconds.
// }

// void Object::Deserialize(const PerceptionObstacle& pb_obs) {
//   track_id = pb_obs.id();
//   theta = pb_obs.theta();

//   center(0) = pb_obs.position().x();
//   center(1) = pb_obs.position().y();
//   center(2) = pb_obs.position().z();

//   velocity(0) = pb_obs.velocity().x();
//   velocity(1) = pb_obs.velocity().y();
//   velocity(2) = pb_obs.velocity().z();

//   length = pb_obs.length();
//   width = pb_obs.width();
//   height = pb_obs.height();

//   polygon.clear();
//   for (int idx = 0; idx < pb_obs.polygon_point_size(); ++idx) {
//     const auto& p = pb_obs.polygon_point(idx);
//     pcl_util::PointD point;
//     point.x = p.x();
//     point.y = p.y();
//     point.z = p.z();
//     polygon.push_back(point);
//   }

//   score = pb_obs.confidence();
//   score_type = pb_obs.confidence_type();
//   tracking_time = pb_obs.tracking_time();
//   latest_tracked_time = pb_obs.timestamp();
//   type = static_cast<ObjectType>(pb_obs.type());
// }

// std::string SensorObjects::ToString() const
// {
//   std::ostringstream oss;
//   oss << "sensor_type: " << GetSensorType(sensor_type)
//       << ", timestamp: " << timestamp
//       << ", sensor2world_pose:\n";
//   oss << sensor2world_pose << "\n, objects: " << objects.size() << " < ";
//   for (auto obj : objects)
//   {
//     oss << "\n"
//         << obj->ToString();
//   }
//   oss << " >]";
//   return oss.str();
// }
} // namespace glb_auto_perception_sensorfusion
