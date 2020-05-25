
#include "perception_yx/perception_yx.h"
#include "tf/transform_datatypes.h"
#include "tf_conversions/tf_eigen.h"
#include "util/log.h"
#include "common/types.h"
#include "common/radar.h"
#include "common/camera.h"
#include "common/lidar.h"
#include <sstream>
#include <thread>
#include <chrono>
#include <algorithm>

template <typename T>
std::string to_string_with_precision(const T a_value, const int n = 6)
{
  std::ostringstream out;
  out.precision(n);
  out << std::fixed << a_value;
  return out.str();
}
namespace perception_yx
{
using namespace glb_auto_perception_sensorfusion;

/**
 * @brief Construct a new PerceptionYX object
 * 
 * @param node ros node handle
 * @param priv_nh ros private node handle
 */
PerceptionYX::PerceptionYX(ros::NodeHandle node, ros::NodeHandle priv_nh)
{
  // Get parameters using private node handle
  std::string path_config_folder;
  priv_nh.param("path_config_folder", path_config_folder, std::string(""));
  AWARN<<"path_config_folder: "<<path_config_folder;
  //priv_nh.param("resize_factor", resize_factor_, 1.0);
  // sensor control
  priv_nh.param("use_front_left_lidar", use_front_left_lidar_, false);
  priv_nh.param("use_front_right_lidar", use_front_right_lidar_, false);
  priv_nh.param("use_rear_left_lidar", use_rear_left_lidar_, false);
  priv_nh.param("use_rear_right_lidar", use_rear_right_lidar_, false);
  priv_nh.param("use_cam", use_cam_, false);
  priv_nh.param("use_radar", use_radar_, false);
  // visualization setting
  priv_nh.param("vis_lidar_poly", vis_lidar_poly_, false);
  priv_nh.param("vis_cam_sphere", vis_cam_sphere_, false);
  priv_nh.param("vis_radar_sphere", vis_radar_sphere_, false);
  // frame id
  priv_nh.param("frame_id_cam", frame_id_cam_, std::string(""));
  priv_nh.param("frame_id_lidar", frame_id_lidar_, std::string(""));
  priv_nh.param("frame_id_radar", frame_id_radar_, std::string(""));
  priv_nh.param("frame_id_fusion", frame_id_fusion_, std::string(""));
  // log setting
  priv_nh.param("log_cam", log_cam_, false);
  priv_nh.param("log_radar", log_radar_, false);
  priv_nh.param("log_lidar", log_lidar_, false);
  priv_nh.param("log_fusion", log_fusion_, false);

  AWARN << "Sensor Front Left Lidar: "
        << (use_front_left_lidar_ ? "\033[1;32mEnabled\033[0m" : "\033[1;31mDisabled\033[0m")
        << ", Logging: "
        << (log_lidar_ ? "\033[1;32mTrue\033[0m" : "\033[1;31mFalse\033[0m");
  AWARN << "Sensor Front Right Lidar: "
        << (use_front_right_lidar_ ? "\033[1;32mEnabled\033[0m" : "\033[1;31mDisabled\033[0m")
        << ", Logging: "
        << (log_lidar_ ? "\033[1;32mTrue\033[0m" : "\033[1;31mFalse\033[0m");
  AWARN << "Sensor Rear Left Lidar: "
        << (use_rear_left_lidar_ ? "\033[1;32mEnabled\033[0m" : "\033[1;31mDisabled\033[0m")
        << ", Logging: "
        << (log_lidar_ ? "\033[1;32mTrue\033[0m" : "\033[1;31mFalse\033[0m");
  AWARN << "Sensor Rear Right Lidar: "
        << (use_rear_right_lidar_ ? "\033[1;32mEnabled\033[0m" : "\033[1;31mDisabled\033[0m")
        << ", Logging: "
        << (log_lidar_ ? "\033[1;32mTrue\033[0m" : "\033[1;31mFalse\033[0m");
  AWARN << "Sensor Radar: "
        << (use_radar_ ? "\033[1;32mEnabled\033[0m" : "\033[1;31mDisabled\033[0m")
        << ", Logging: "
        << (log_radar_ ? "\033[1;32mTrue\033[0m" : "\033[1;31mFalse\033[0m");
  AWARN << "Sensor Camera: "
        << (use_cam_ ? "\033[1;32mEnabled\033[0m" : "\033[1;31mDisabled\033[0m")
        << ", Logging: "
        << (log_cam_ ? "\033[1;32mTrue\033[0m" : "\033[1;31mFalse\033[0m");
  AWARN << "Fusion, Logging: "
        << (log_fusion_ ? "\033[1;32mTrue\033[0m" : "\033[1;31mFalse\033[0m");  
  AWARN << "frame_id_radar "
        << frame_id_radar_;
  AWARN << "vis_radar_sphere: "
        << (vis_radar_sphere_ ? "\033[1;32mTrue\033[0m" : "\033[1;31mFalse\033[0m"); 
  //AWARN << "Image Resize Factor: " << "\033[1;33m" << resize_factor_ << "\033[0m";

  // mutex
  //pthread_mutex_init(&proj_lock_, NULL);

  // Read in camera parameters
  //std::string file_path_camera_yaml = path_config_folder + "/config/calibration.yaml";
  //if (parseCameraCalibration(file_path_camera_yaml, resize_factor_) != 0) {
  //  AERROR << "failed to Parse camera calibration";
  //  return;
  //}

  // Initialize fusion
  if (!obstacle_fusion_.Init(path_config_folder, log_fusion_))
  {
    AERROR << "Failed to init ObstacleFusion";
    return;
  }
  //Initialize visualization marker vector
  //lidar bbox line list
  lidar_bbox_list_.header.frame_id = frame_id_lidar_;
  lidar_bbox_list_.ns = "lidar bbox line list";
  lidar_bbox_list_.action = visualization_msgs::Marker::ADD;
  lidar_bbox_list_.pose.orientation.w = 1.0;
  lidar_bbox_list_.id = 1;
  lidar_bbox_list_.type = visualization_msgs::Marker::LINE_LIST;
  // lidar_bbox_list_.lifetime = ros::Duration(1.0);
  lidar_bbox_list_.scale.x = 0.1; // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
  if (vis_lidar_poly_ == true) {
    lidar_bbox_list_.color.r = 1.0; 
    lidar_bbox_list_.color.g = 0.0; 
    lidar_bbox_list_.color.b = 0.0; 
    lidar_bbox_list_.color.a = 1.0; // red
  }
  // radar bbox line list
  radar_bbox_list_.header.frame_id = frame_id_radar_;
  radar_bbox_list_.ns = "radar bbox line list";
  radar_bbox_list_.action = visualization_msgs::Marker::ADD;
  radar_bbox_list_.pose.orientation.w = 1.0;
  radar_bbox_list_.id = 20;
  //radar_bbox_list_.lifetime = ros::Duration(1.0);
  if (vis_radar_sphere_ == true)
  {
    radar_bbox_list_.type = visualization_msgs::Marker::SPHERE_LIST;
    radar_bbox_list_.scale.x = 1.0;
    radar_bbox_list_.scale.y = 1.0;
    radar_bbox_list_.scale.z = 1.0;
  }
  else
  {
    radar_bbox_list_.type = visualization_msgs::Marker::LINE_LIST;
    radar_bbox_list_.scale.x = 0.1;
    radar_bbox_list_.color.r = 0.0;
    radar_bbox_list_.color.g = 0.0;
    radar_bbox_list_.color.b = 1.0; // blue
    radar_bbox_list_.color.a = 1.0;
  }

  // camera bbox line list
  camera_bbox_list_.header.frame_id = frame_id_cam_;
  camera_bbox_list_.ns = "camera bbox line list";
  camera_bbox_list_.action = visualization_msgs::Marker::ADD;
  camera_bbox_list_.pose.orientation.w = 1.0;
  camera_bbox_list_.id = 20;
  //camera_bbox_list_.lifetime = ros::Duration(1.0);
  if (vis_cam_sphere_ == true)
  {
    camera_bbox_list_.type = visualization_msgs::Marker::SPHERE_LIST;
    camera_bbox_list_.scale.x = 1.0;
    camera_bbox_list_.scale.y = 1.0;
    camera_bbox_list_.scale.z = 1.0;
  }
  else
  {
    camera_bbox_list_.type = visualization_msgs::Marker::LINE_LIST;
    camera_bbox_list_.scale.x = 0.1;
    camera_bbox_list_.color.r = 0.0;
    camera_bbox_list_.color.g = 1.0; //green
    camera_bbox_list_.color.b = 0.0; 
    camera_bbox_list_.color.a = 0.0;
  }
  // fusion bbox line list
  fusion_bbox_list_.header.frame_id = frame_id_fusion_;
  fusion_bbox_list_.ns = "fusion bbox line list";
  fusion_bbox_list_.action = visualization_msgs::Marker::ADD;
  fusion_bbox_list_.pose.orientation.w = 1.0;
  fusion_bbox_list_.id = 30;
  //fusion_bbox_list_.lifetime = ros::Duration(1.0);
  fusion_bbox_list_.type = visualization_msgs::Marker::LINE_LIST;
  fusion_bbox_list_.scale.x = 0.1;

  // Initialze SensorObjects
  lidar_fl_objects_.reset(new glb_auto_perception_sensorfusion::SensorObjects);
  lidar_fr_objects_.reset(new glb_auto_perception_sensorfusion::SensorObjects);
  lidar_rl_objects_.reset(new glb_auto_perception_sensorfusion::SensorObjects);
  lidar_rr_objects_.reset(new glb_auto_perception_sensorfusion::SensorObjects);
  camera_objects_.reset(new glb_auto_perception_sensorfusion::SensorObjects);
  radar_objects_.reset(new glb_auto_perception_sensorfusion::SensorObjects);

  // Pulisher: radar
  radar_bbox_publisher_ =
      node.advertise<visualization_msgs::Marker>("/radar_bbox_marker",1);
  radar_velocity_publisher_ =
      node.advertise<visualization_msgs::MarkerArray>("perception/output/radar_velocity_marker", 1);
  
  // Publisher: lidar
  lidar_bbox_publisher_ = node.advertise<visualization_msgs::Marker>("/lidar_bbox_marker",1);
  lidar_velocity_publisher_=node.advertise<visualization_msgs::MarkerArray>("/lidar_velocity_marker",1);

  // publisher :Camera
  camera_bbox_publisher_= 
    node.advertise<visualization_msgs::Marker>("/camera_bbox_marker",1);

  // Publisher: fusion
  fusion_bbox_publisher_ =
      node.advertise<visualization_msgs::Marker>("perception/output/fusion_bbox_marker", 1);
  fusion_velocity_publisher_ =
      node.advertise<visualization_msgs::MarkerArray>("perception/output/fusion_velocity_marker", 1);

  fusion_result_publisher_=node.advertise<fusion_msgs::FusionObjectList>("perception/output/fusion_result",1);


  std::string topic_name_lidar_fl_objs_in = "/perception/lidar/object_list";
  lidar_fl_message_ = node.subscribe<lidar_msgs::LidarObjectList>(topic_name_lidar_fl_objs_in, 1,
                                     boost::bind(&PerceptionYX::processLidarMessage, this,_1,glb_auto_perception_sensorfusion::SensorType::LeiShen_lidar_fl));
  // std::string topic_name_lidar_fr_objs_in = "/glb_auto/sensor/lidar_fr_objects";
  // lidar_fr_message_ = node.subscribe<lidar_msgs::LidarObjectList>(topic_name_lidar_fr_objs_in, 1,
  //                                    boost::bind(&PerceptionYX::processLidarMessage, this,_1,glb_auto_perception_sensorfusion::SensorType::LeiShen_lidar_fr));
  // std::string topic_name_lidar_rl_objs_in = "/glb_auto/sensor/lidar_rl_objects";
  // lidar_rl_message_ = node.subscribe<lidar_msgs::LidarObjectList>(topic_name_lidar_rl_objs_in, 1,
  //                                    boost::bind(&PerceptionYX::processLidarMessage, this,_1,glb_auto_perception_sensorfusion::SensorType::LeiShen_lidar_rl));
  // std::string topic_name_lidar_rr_objs_in = "/glb_auto/sensor/lidar_rr_objects";
  // lidar_rr_message_ = node.subscribe<lidar_msgs::LidarObjectList>(topic_name_lidar_rr_objs_in, 1,
  //                                    boost::bind(&PerceptionYX::processLidarMessage, this,_1,glb_auto_perception_sensorfusion::SensorType::LeiShen_lidar_rr));

  // std::string topic_name_camera_objs_in = "/CameraObjMsg";
  /* camera_message_ = node.subscribe(topic_name_camera_objs_in, 2,
                                   &PerceptionYX::processCameraMessage, this,
                                   ros::TransportHints().reliable().tcpNoDelay(true));
  */
  std::string topic_name_radar_objs_in = "/perception/radar_fusion/object_list";
  radar_conti_ = node.subscribe(topic_name_radar_objs_in, 10,
                                &PerceptionYX::processRadarMessage, this,
                                ros::TransportHints().reliable().tcpNoDelay(true));
  
  // boost::thread fusion(boost::bind(&PerceptionYX::processFusion,this));
}

PerceptionYX::~PerceptionYX() {}


void PerceptionYX::objToBBox(std::shared_ptr<glb_auto_perception_sensorfusion::Object> &obj, 
                             glb_auto_perception_sensorfusion::SensorType sensor_type) {
  if (vis_lidar_poly_ == true && is_lidar(sensor_type)) {  // show lidar objects with polygon
  } else if (vis_cam_sphere_ == true && is_camera(sensor_type)) {  // show camera objects as shpere
  } else if (vis_radar_sphere_ == true && is_radar(sensor_type)) { // show radar objects as shpere
    geometry_msgs::Point p;
    p.x = obj->center(0);
    p.y = obj->center(1);
    p.z = obj->center(2);
    radar_bbox_list_.points.push_back(p);
    std_msgs::ColorRGBA p_color;
    if (sensor_type == glb_auto_perception_sensorfusion::SensorType::Conti_MRR) {
      //p_color.a = obj->score;
      p_color.a = 1;
      p_color.r = 0.5;
      p_color.g = 0.8;
      p_color.b = 0.9;         // blue
    } else if (sensor_type == glb_auto_perception_sensorfusion::SensorType::Conti_MRR) {
      p_color.a = obj->score;  // sky blue
      p_color.r = 0.5;
      p_color.g = 0.8;
      p_color.b = 0.9;
    } 
    radar_bbox_list_.colors.push_back(p_color);
  } else {  
    // visualize as rectangles
    //Eigen::Vector3d bottom_quad[4];
    geometry_msgs::Point p_0;
    p_0.x = obj->bbox(0,0); p_0.y = obj->bbox(0,1); p_0.z = obj->bbox(0,2); 
    geometry_msgs::Point p_1;
    p_1.x = obj->bbox(1,0); p_1.y = obj->bbox(1,1); p_1.z = obj->bbox(1,2);
    geometry_msgs::Point p_2;
    p_2.x = obj->bbox(2,0); p_2.y = obj->bbox(2,1); p_2.z = obj->bbox(2,2);
    geometry_msgs::Point p_3;
    p_3.x = obj->bbox(3,0); p_3.y = obj->bbox(3,1); p_3.z = obj->bbox(3,2);

    geometry_msgs::Point p_4;
    p_4.x = obj->bbox(4,0); p_4.y = obj->bbox(4,1); p_4.z = obj->bbox(4,2); 
    geometry_msgs::Point p_5;
    p_5.x = obj->bbox(5,0); p_5.y = obj->bbox(5,1); p_5.z = obj->bbox(5,2);
    geometry_msgs::Point p_6;
    p_6.x = obj->bbox(6,0); p_6.y = obj->bbox(6,1); p_6.z = obj->bbox(6,2);
    geometry_msgs::Point p_7;
    p_7.x = obj->bbox(7,0); p_7.y = obj->bbox(7,1); p_7.z = obj->bbox(7,2);

    if (is_lidar(sensor_type)) {
      lidar_bbox_list_.points.push_back(p_0); lidar_bbox_list_.points.push_back(p_1);
      lidar_bbox_list_.points.push_back(p_1); lidar_bbox_list_.points.push_back(p_2);
      lidar_bbox_list_.points.push_back(p_2); lidar_bbox_list_.points.push_back(p_3);
      lidar_bbox_list_.points.push_back(p_3); lidar_bbox_list_.points.push_back(p_0);
      lidar_bbox_list_.points.push_back(p_4); lidar_bbox_list_.points.push_back(p_5);
      lidar_bbox_list_.points.push_back(p_5); lidar_bbox_list_.points.push_back(p_6);
      lidar_bbox_list_.points.push_back(p_6); lidar_bbox_list_.points.push_back(p_7);
      lidar_bbox_list_.points.push_back(p_7); lidar_bbox_list_.points.push_back(p_4);
      lidar_bbox_list_.points.push_back(p_0); lidar_bbox_list_.points.push_back(p_4);
      lidar_bbox_list_.points.push_back(p_1); lidar_bbox_list_.points.push_back(p_5);
      lidar_bbox_list_.points.push_back(p_2); lidar_bbox_list_.points.push_back(p_6);
      lidar_bbox_list_.points.push_back(p_3); lidar_bbox_list_.points.push_back(p_7);
    } else if (is_camera(sensor_type)) {
      camera_bbox_list_.points.push_back(p_0); camera_bbox_list_.points.push_back(p_1);
      camera_bbox_list_.points.push_back(p_1); camera_bbox_list_.points.push_back(p_2);
      camera_bbox_list_.points.push_back(p_2); camera_bbox_list_.points.push_back(p_3);
      camera_bbox_list_.points.push_back(p_3); camera_bbox_list_.points.push_back(p_0);
      camera_bbox_list_.points.push_back(p_4); camera_bbox_list_.points.push_back(p_5);
      camera_bbox_list_.points.push_back(p_5); camera_bbox_list_.points.push_back(p_6);
      camera_bbox_list_.points.push_back(p_6); camera_bbox_list_.points.push_back(p_7);
      camera_bbox_list_.points.push_back(p_7); camera_bbox_list_.points.push_back(p_4);
      camera_bbox_list_.points.push_back(p_0); camera_bbox_list_.points.push_back(p_4);
      camera_bbox_list_.points.push_back(p_1); camera_bbox_list_.points.push_back(p_5);
      camera_bbox_list_.points.push_back(p_2); camera_bbox_list_.points.push_back(p_6);
      camera_bbox_list_.points.push_back(p_3); camera_bbox_list_.points.push_back(p_7);
    } else if (is_radar(sensor_type)) {
      radar_bbox_list_.points.push_back(p_0); radar_bbox_list_.points.push_back(p_1);
      radar_bbox_list_.points.push_back(p_1); radar_bbox_list_.points.push_back(p_2);
      radar_bbox_list_.points.push_back(p_2); radar_bbox_list_.points.push_back(p_3);
      radar_bbox_list_.points.push_back(p_3); radar_bbox_list_.points.push_back(p_0);
      radar_bbox_list_.points.push_back(p_4); radar_bbox_list_.points.push_back(p_5);
      radar_bbox_list_.points.push_back(p_5); radar_bbox_list_.points.push_back(p_6);
      radar_bbox_list_.points.push_back(p_6); radar_bbox_list_.points.push_back(p_7);
      radar_bbox_list_.points.push_back(p_7); radar_bbox_list_.points.push_back(p_4);
      radar_bbox_list_.points.push_back(p_0); radar_bbox_list_.points.push_back(p_4);
      radar_bbox_list_.points.push_back(p_1); radar_bbox_list_.points.push_back(p_5);
      radar_bbox_list_.points.push_back(p_2); radar_bbox_list_.points.push_back(p_6);
      radar_bbox_list_.points.push_back(p_3); radar_bbox_list_.points.push_back(p_7);
    } else if (sensor_type == SensorType::FUSION) {
      fusion_bbox_list_.points.push_back(p_0); fusion_bbox_list_.points.push_back(p_1);
      fusion_bbox_list_.points.push_back(p_1); fusion_bbox_list_.points.push_back(p_2);
      fusion_bbox_list_.points.push_back(p_2); fusion_bbox_list_.points.push_back(p_3);
      fusion_bbox_list_.points.push_back(p_3); fusion_bbox_list_.points.push_back(p_0);
      fusion_bbox_list_.points.push_back(p_4); fusion_bbox_list_.points.push_back(p_5);
      fusion_bbox_list_.points.push_back(p_5); fusion_bbox_list_.points.push_back(p_6);
      fusion_bbox_list_.points.push_back(p_6); fusion_bbox_list_.points.push_back(p_7);
      fusion_bbox_list_.points.push_back(p_7); fusion_bbox_list_.points.push_back(p_4);
      fusion_bbox_list_.points.push_back(p_0); fusion_bbox_list_.points.push_back(p_4);
      fusion_bbox_list_.points.push_back(p_1); fusion_bbox_list_.points.push_back(p_5);
      fusion_bbox_list_.points.push_back(p_2); fusion_bbox_list_.points.push_back(p_6);
      fusion_bbox_list_.points.push_back(p_3); fusion_bbox_list_.points.push_back(p_7);
    }else {
      AERROR << "objToBBox: unsupported sensor type: " << GetSensorType(sensor_type);
      return;
    }
    // add colors
    std_msgs::ColorRGBA points_color;
    if (is_camera(sensor_type)) {
      points_color.a = 1.0;
      points_color.r = 0.0;
      points_color.g = 1.0; // radar objects are green
      points_color.b = 0.0;
    } else if (is_radar(sensor_type)) {
      points_color.a = 1.0;  // radar objects are blue
      points_color.r = 0.0;
      points_color.g = 0.0;
      points_color.b = 1.0;
    } else if (sensor_type == glb_auto_perception_sensorfusion::SensorType::FUSION) {
      if ((obj->fused_lidar && obj->fused_radar) || (obj->fused_lidar && obj->fused_cam)) {
        points_color.a = 1.0;  // matched fusion objects are lightcoral
        points_color.r = 0.9;
        points_color.g = 0.6;
        points_color.b = 0.6;
      } else {
        points_color.a = 1.0;  // unmatched fusion objects are firebrick
        points_color.r = 0.7;
        points_color.g = 0.2;
        points_color.b = 0.2;
      }
    } else {  // lidar objects 
      if (obj->type == glb_auto_perception_sensorfusion::ObjectType::VEHICLE) {
        points_color.a = 1.0;
        points_color.r = 1.0;  // vehicle is red
        points_color.g = 0.0;
        points_color.b = 0.0;
      } else if (obj->type == glb_auto_perception_sensorfusion::ObjectType::BICYCLE) {
        points_color.a = 1.0;
        points_color.r = 0.0;
        points_color.g = 1.0;  // bicycle is green
        points_color.b = 0.0;
      } else if (obj->type == glb_auto_perception_sensorfusion::ObjectType::PEDESTRIAN) {
        points_color.a = 1.0;
        points_color.r = 0.0;
        points_color.g = 0.0;
        points_color.b = 1.0;  // pedestrian is blue
      } else {
        points_color.a = 1.0;
        points_color.r = 1;  // unknown is grey
        points_color.g = 0.;
        points_color.b = 0.;
      }
    }
    for (int i = 0; i < 24; i++) {
      if (is_lidar(sensor_type)) {
        lidar_bbox_list_.colors.push_back(points_color);
      } else if (is_camera(sensor_type)) {
        camera_bbox_list_.colors.push_back(points_color);
      } else if (is_radar(sensor_type)) {
        radar_bbox_list_.colors.push_back(points_color);
      } else if (sensor_type == glb_auto_perception_sensorfusion::SensorType::FUSION) {
        fusion_bbox_list_.colors.push_back(points_color);
      } else {
        AERROR << "objToBBox: unsupported sensor type: " << GetSensorType(sensor_type);
      }
    }
  }
  return;
}

visualization_msgs::Marker PerceptionYX::objToMarker(std::shared_ptr<glb_auto_perception_sensorfusion::Object> &obj,
                                                     std::string &frame_id,
                                                     ros::Time &timestamp,
                                                     std::string &ns,
                                                     int32_t id,
                                                     int32_t type)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = timestamp;
  marker.ns = ns;
  marker.action = visualization_msgs::Marker::ADD;
  marker.id = id;
  marker.color.r = 1.0; // yellow
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;
  marker.lifetime = ros::Duration(0.1);
  marker.type = type;
  if (type == visualization_msgs::Marker::TEXT_VIEW_FACING)
  {
    marker.pose.position.x = obj->center(0);
    marker.pose.position.y = obj->center(1);
    marker.pose.position.z = obj->center(2) + 4.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    //marker.scale.x = 1;
    //marker.scale.y = 1;
    marker.scale.z = 1;
    marker.text = "ID: " + std::to_string(obj->track_id) + " Vel: " + to_string_with_precision(obj->velocity.norm(), 1)+ "\n"+
                  " lon_p: "+to_string_with_precision(obj->center(0),2)+" lat: "+to_string_with_precision(obj->center(1),2);
  }
  else if (type == visualization_msgs::Marker::ARROW)
  {
    double marker_arrow_length = 1.0;
    marker.scale.x = 0.3; //shaft diameter
    marker.scale.y = 0.3; //head diameter
    marker.scale.z = 0.5; // head length
    geometry_msgs::Point marker_p1;
    marker_p1.x = obj->center(0);
    marker_p1.y = obj->center(1);
    marker_p1.z = obj->center(2);
    geometry_msgs::Point marker_p2;
    // marker_p2.x = obj->center(0) + obj->direction(0) * marker_arrow_length * obj->velocity.norm();
    // marker_p2.y = obj->center(1) + obj->direction(1) * marker_arrow_length * obj->velocity.norm();
    // marker_p2.z = obj->center(2) + obj->direction(2) * marker_arrow_length * obj->velocity.norm();
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker_p2.x = obj->center(0) + obj->velocity(0);
    marker_p2.y = obj->center(1) + obj->velocity(1);
    marker_p2.z = obj->center(2);
    marker.points.push_back(marker_p1);
    marker.points.push_back(marker_p2);
  }
  return marker;
}

void PerceptionYX::processRadarMessage(const radar_msgs::RadarObjectListConstPtr &msg)
{
  if (!use_radar_) {
    AWARN_IF(log_radar_) << "Radar deactivated!";
    return;
  }

  AWARN_IF(log_radar_) << "\033[1;32m process Radar message \033[0m, timestamp: " << std::fixed 
                        << std::setprecision(12) << static_cast<double>(msg->header.stamp.sec);
  //   << " at time stamp: " << std::fixed << std::setprecision(12) << msg->header.stamp;

  AWARN_IF(log_radar_) << "size of Objects from radar is: " << +msg->ObjNum;
  //double radar_process_time = double(clock() - time_start) / CLOCKS_PER_SEC;
  //AWARN_IF(log_radar_) << "\033[1;33m" << "Radar processing takes: "
  //      << std::fixed << std::setprecision(6) << radar_process_time << "s at "
  //      << 1.0 / radar_process_time << "Hz" << "\033[0m";

  // radar bbox line list definition
  radar_bbox_list_.header.stamp = ros::Time::now();
  radar_bbox_list_.points.clear();
  radar_bbox_list_.colors.clear();
  radar_velocity_list_.markers.clear();
  int marker_count=200;
  //eatract objects from radar_message
  ros::Time current_time = ros::Time::now();
  double curr_time=static_cast<double>(current_time.sec)+static_cast<double>(current_time.nsec)/1.0e9;
  double data_time=static_cast<double>(msg->header.stamp.sec)+static_cast<double>(msg->header.stamp.nsec)/1.0e9;
  double time_diff=curr_time-data_time;
  ADEBUG <<"Radar Object time_diff : "<<time_diff; 
  int index = 0;
  int valid_nums=0;
  while (index < msg->ObjNum && index < msg->ObjectList.size())
  {
    if (!Radar::Filter(msg->ObjectList[index]))
    {
      ++valid_nums;
      std::shared_ptr<glb_auto_perception_sensorfusion::Object> pObj = std::make_shared<glb_auto_perception_sensorfusion::Object>();
      pObj->track_id = msg->ObjectList[index].ID;
      /* 
      pObj->center(0) = msg->ObjectList[index].Abs_Pos.x;
      pObj->center(1) = msg->ObjectList[index].Abs_Pos.y;
      //pObj->center(2)=0.5;
      pObj->anchor_point=pObj->center;
      pObj->velocity(0) = msg->ObjectList[index].Abs_Vel.x;
      pObj->velocity(1) = msg->ObjectList[index].Abs_Vel.y;
      pObj->position_uncertainty(0, 0) =msg->ObjectList[index].Abs_Pos_Stdev.x;
      pObj->position_uncertainty(1, 1) = msg->ObjectList[index].Abs_Pos_Stdev.y;
      pObj->position_uncertainty(2, 2) = msg->ObjectList[index].Abs_Pos_Stdev.z;
      pObj->velocity_uncertainty(0, 0) = msg->ObjectList[index].Abs_Vel_Stdev.x;
      pObj->velocity_uncertainty(1, 1) = msg->ObjectList[index].Abs_Vel_Stdev.y;
      pObj->velocity_uncertainty(2, 2) = msg->ObjectList[index].Abs_Vel_Stdev.z;
      pObj->theta = msg->ObjectList[index].Abs_Bbox_Angle.heading;
      */              
      pObj->center(0) = msg->ObjectList[index].Rel_Pos.x+time_diff*msg->ObjectList[index].Rel_Vel.x;
      pObj->center(1) = msg->ObjectList[index].Rel_Pos.y+time_diff*msg->ObjectList[index].Rel_Vel.y;
      pObj->center(2) = msg->ObjectList[index].Rel_Pos.z+time_diff*msg->ObjectList[index].Rel_Vel.z;
      pObj->anchor_point=pObj->center;
      pObj->velocity(0) = msg->ObjectList[index].Rel_Vel.x;
      pObj->velocity(1) = msg->ObjectList[index].Rel_Vel.y;
      pObj->velocity(2) = msg->ObjectList[index].Rel_Vel.z;
      pObj->position_uncertainty(0, 0) =msg->ObjectList[index].Rel_Pos_Stdev.x;
      pObj->position_uncertainty(1, 1) = msg->ObjectList[index].Rel_Pos_Stdev.y;
      pObj->position_uncertainty(2, 2) = msg->ObjectList[index].Rel_Pos_Stdev.z;
      pObj->velocity_uncertainty(0, 0) = msg->ObjectList[index].Rel_Vel_Stdev.x;
      pObj->velocity_uncertainty(1, 1) = msg->ObjectList[index].Rel_Vel_Stdev.y;
      pObj->velocity_uncertainty(2, 2) = msg->ObjectList[index].Rel_Vel_Stdev.z;
      pObj->theta = msg->ObjectList[index].Rel_Bbox_Angle.heading;
      pObj->length = msg->ObjectList[index].Length;
      pObj->width = msg->ObjectList[index].Width;
      pObj->height=1.0;
      //pObj->type =msg->ObjectList[index].Obj_Type;
      pObj->score=msg->ObjectList[index].Prob_Exist;
      if(nullptr==pObj->radar_supplement){
        pObj->radar_supplement.reset(new RadarSupplement());
      }
      pObj->radar_supplement->range=sqrt(pObj->center(0)*pObj->center(0)+pObj->center(1)*pObj->center(1));
      pObj->radar_supplement->angle=0.0;
      ADEBUG <<"Radar Object Information : " 
        <<"ID: "<<pObj->track_id<<" length = "<<pObj->length<<" width: "<<pObj->width<<" orientation angle : "<<pObj->theta \
        <<" position: "<<pObj->center(0)<<" , "<<pObj->center(1)<<" velocity: "<<pObj->velocity(0)<<" "<<pObj->velocity(1)
        <<" object type: "<<msg->ObjectList[index].Obj_Type
        <<" covarianect: "<<pObj->position_uncertainty(0, 0)<<" , "<<pObj->position_uncertainty(1, 1)<<" , "
        <<pObj->velocity_uncertainty(0, 0)<<" , "<<pObj->velocity_uncertainty(1, 1)
        <<" \033[1;32m RCS:" <<msg->ObjectList[index].RCS<<"\033[0m";
      //add bounding box
      objToBBox(pObj,glb_auto_perception_sensorfusion::SensorType::Conti_MRR);
      radar_objects_->objects.push_back(pObj);
    }
    ++ index;
  }
  //Publish visualization 
  radar_bbox_publisher_.publish(radar_bbox_list_);
  //
  radar_objects_->sensor_type = glb_auto_perception_sensorfusion::SensorType::Conti_MRR;
  radar_objects_->sensor_id = GetSensorType(radar_objects_->sensor_type);
  //radar_objects_->timestamp = static_cast<double>(msg->header.stamp.sec)+static_cast<double>(msg->header.stamp.nsec)/1.0e9;
  radar_objects_->timestamp = curr_time;
  radar_objects_->sensor2world_pose = Eigen::Matrix4d::Identity();

  //fusion
  AWARN_IF(log_radar_) << "size of valid Objects from radar is: " << valid_nums;
  processFusion(radar_objects_);
  radar_objects_->objects.clear();
  // auto data_manager =glb_auto_perception_sensorfusion::SortedSensorFramePoolManager::getInstance();
  // data_manager->addNewFrame(radar_objects_);
  //radar_objects_.reset(new glb_auto_perception_sensorfusion::SensorObjects);
  return;
}


/*void PerceptionYX::processCameraMessage(const CameraMsg::VisObjsConstPtr &msg)
{
  if (!use_cam_) {
    AWARN_IF(log_cam_) << "camera deactivated!";
    return;
  }
  AWARN_IF(log_cam_) << "\033[1;32mprocess Camera data\033[0m, timestamp: " << std::fixed 
                    << std::setprecision(12)<<static_cast<double>(msg->stamp)/1e9;
  //     << " at time stamp: " << std::fixed << std::setprecision(12) << msg->header.stamp;

  AWARN_IF(log_cam_) << "size of Objects from camera is: " << +msg->obsNum;
  camera_objects_->sensor_type = glb_auto_perception_sensorfusion::SensorType::CAMERA;
 // camera bbox line list definition
  camera_bbox_list_.header.stamp = ros::Time::now();
  camera_bbox_list_.points.clear();
  camera_bbox_list_.colors.clear();
  //camera_velocity_list_.markers.clear();
  int marker_count=200;
  //eatract objects from camera_message
  int index = 0;
  while (index < msg->visObjs.size()&&index<msg->obsNum)
  {
    if(!camera::Filter(msg->visObjs[index])){
      std::shared_ptr<glb_auto_perception_sensorfusion::Object> pObj = std::make_shared<glb_auto_perception_sensorfusion::Object>();
      pObj->track_id = msg->visObjs[index].ID;
      pObj->center(0) = msg->visObjs[index].Long_Pos;
      pObj->center(1) = msg->visObjs[index].Lat_Pos;
      pObj->center(2)=1.0;
      pObj->anchor_point=pObj->center;
      pObj->velocity(0) = msg->visObjs[index].Long_Vel;
      pObj->velocity(1) = msg->visObjs[index].Lat_Vel;
      
      pObj->position_uncertainty(0, 0) = 10; //TODO:\Attention hard code
      pObj->position_uncertainty(1, 1) = 10;
      pObj->velocity_uncertainty(0, 0) = 10;
      pObj->velocity_uncertainty(1, 1) = 10;

      pObj->type=camera::GetCameraObjType(static_cast<Obs_Type>(msg->visObjs[index].Obs_Classif));
      if(pObj->type== ObjectType::PEDESTRIAN || pObj->type==ObjectType::UNKNOWN){
        pObj->length = 1.7;
        pObj->width = 1;
        pObj->height=1.0;
      }else
      {
        pObj->height = msg->visObjs[index].Height;
        pObj->width = msg->visObjs[index].Width;
        pObj->length = pObj->width;
      }
      pObj->theta = atan2(msg->visObjs[index].Lat_Pos,msg->visObjs[index].Long_Pos);

      //add bounding box
      objToBBox(pObj,glb_auto_perception_sensorfusion::SensorType::CAMERA);
    
      ADEBUG <<"Camera Object Information : "<<"ID: "<<pObj->track_id <<" length = "<<pObj->length<<" width: "<<pObj->width<<" orientation angle : "<<pObj->theta \
            <<" position: "<<pObj->center(0)<<" , "<<pObj->center(1)<<" velocity: "<<pObj->velocity(0)<<" "<<pObj->velocity(1)
            <<" type: " <<camera::GetCameraObjTypeName(static_cast<Obs_Type>(msg->visObjs[index].Obs_Classif));
      camera_objects_->objects.emplace_back(pObj);

    }
    ++ index;
  }
  //Publish visualization 
  camera_bbox_publisher_.publish(camera_bbox_list_);
  //camera_objects_->objects = camera_objects;
  camera_objects_->sensor_type = glb_auto_perception_sensorfusion::SensorType::CAMERA;
  camera_objects_->sensor_id = GetSensorType(camera_objects_->sensor_type);
  camera_objects_->timestamp = msg->stamp/(1e9);
  camera_objects_->sensor2world_pose = Eigen::Matrix4d::Identity();

  // fusion
  processFusion(camera_objects_);
  camera_objects_->objects.clear();
}
*/


void PerceptionYX::processLidarMessage(const lidar_msgs::LidarObjectListConstPtr &msg,const glb_auto_perception_sensorfusion::SensorType sensor)
{
  AWARN_IF(log_lidar_) << "\033[1;32mprocess Lidar data at time stamp: " << std::fixed << std::setprecision(12) 
                       << static_cast<double>(msg->header.stamp.sec)+static_cast<double>(msg->header.stamp.nsec)/1.0e9;
  if (!use_front_left_lidar_ && sensor == glb_auto_perception_sensorfusion::SensorType::LeiShen_lidar_fl) {
    AWARN_IF(log_lidar_) << "front_left_lidar deactivated!";
    return;
  }
  if (!use_front_right_lidar_ && sensor == glb_auto_perception_sensorfusion::SensorType::LeiShen_lidar_fr) {
    AWARN_IF(log_lidar_) << "front_right_lidar deactivated!";
    return;
  }
  if (!use_rear_left_lidar_ && sensor == glb_auto_perception_sensorfusion::SensorType::LeiShen_lidar_rl) {
    AWARN_IF(log_lidar_) << "rear_left_lidar deactivated!";
    return;
  }
  if (!use_rear_right_lidar_ && sensor == glb_auto_perception_sensorfusion::SensorType::LeiShen_lidar_rr) {
    AWARN_IF(log_lidar_) << "rear_right_lidar deactivated!";
    return;
  }

  AWARN_IF(log_lidar_) << "\033[1;32m process Lidar message \033[0m, frame_id: " << msg->header.frame_id;
  //   << " at time stamp: " << std::fixed << std::setprecision(12) << msg->header.stamp;

  AWARN_IF(log_lidar_) << "size of Objects from lidar is: " << msg->ObjNum;
  //double radar_process_time = double(clock() - time_start) / CLOCKS_PER_SEC;
  //AWARN_IF(log_radar_) << "\033[1;33m" << "Radar processing takes: "
  //      << std::fixed << std::setprecision(6) << radar_process_time << "s at "
  //      << 1.0 / radar_process_time << "Hz" << "\033[0m";

  // lidar bbox line list definition
  
  lidar_bbox_list_.header.stamp = ros::Time();
  lidar_bbox_list_.points.clear();
  lidar_bbox_list_.colors.clear();
  lidar_velocity_list_.markers.clear();
  int marker_count=200;
  //eatract objects from lidar_message
  ros::Time current_time = ros::Time::now();
  double curr_time=static_cast<double>(current_time.sec)+static_cast<double>(current_time.nsec)/1.0e9;
  ADEBUG <<"calc ros curr_time : "<<static_cast<double>(current_time.sec)<<"  "<<static_cast<double>(current_time.nsec)/1.0e9;
  double data_time=static_cast<double>(msg->header.stamp.sec)+static_cast<double>(msg->header.stamp.nsec)/1.0e9;
  ADEBUG <<"calc Object time : "<<static_cast<double>(msg->header.stamp.sec)<<"  "<<static_cast<double>(msg->header.stamp.nsec)/1.0e9;
  double time_diff=curr_time-data_time;
  ADEBUG <<"Lidar Object time_diff : "<<time_diff;
  for(size_t index=0; index< msg->ObjectList.size();++index)
  {
    if (!Lidar::Filter(msg->ObjectList[index])){
      std::shared_ptr<glb_auto_perception_sensorfusion::Object> pObj = std::make_shared<glb_auto_perception_sensorfusion::Object>();
      pObj->track_id = msg->ObjectList[index].ID;
      /*
      pObj->center(0) = msg->ObjectList[index].Abs_Pos.x;
      pObj->center(1) = msg->ObjectList[index].Abs_Pos.y;
      pObj->center(2) = msg->ObjectList[index].Abs_Pos.z;

      pObj->velocity(0) = msg->ObjectList[index].Abs_Vel.x;
      pObj->velocity(1) = msg->ObjectList[index].Abs_Vel.y;
      pObj->velocity(2) = msg->ObjectList[index].Abs_Vel.z;

      pObj->position_uncertainty(0, 0) = msg->ObjectList[index].Abs_Pos_Stdev.x;
      pObj->position_uncertainty(1, 1) = msg->ObjectList[index].Abs_Pos_Stdev.y;
      pObj->position_uncertainty(2, 2) = msg->ObjectList[index].Abs_Pos_Stdev.z;

      pObj->velocity_uncertainty(0, 0) = msg->ObjectList[index].Abs_Vel_Stdev.x;
      pObj->velocity_uncertainty(1, 1) = msg->ObjectList[index].Abs_Vel_Stdev.y;
      pObj->velocity_uncertainty(2, 2) = msg->ObjectList[index].Abs_Vel_Stdev.z;
  
      pObj->theta = msg->ObjectList[index].Abs_Bbox_Angle.heading;

      pObj->length = msg->ObjectList[index].Length;
      pObj->width = msg->ObjectList[index].Width;
      pObj->bbox<<msg->ObjectList[index].Abs_Bbox[0].x,msg->ObjectList[index].Abs_Bbox[0].y,msg->ObjectList[index].Abs_Bbox[0].z,
                  msg->ObjectList[index].Abs_Bbox[1].x,msg->ObjectList[index].Abs_Bbox[1].y,msg->ObjectList[index].Abs_Bbox[1].z,
                  msg->ObjectList[index].Abs_Bbox[2].x,msg->ObjectList[index].Abs_Bbox[2].y,msg->ObjectList[index].Abs_Bbox[2].z,
                  msg->ObjectList[index].Abs_Bbox[3].x,msg->ObjectList[index].Abs_Bbox[3].y,msg->ObjectList[index].Abs_Bbox[3].z,
                  msg->ObjectList[index].Abs_Bbox[4].x,msg->ObjectList[index].Abs_Bbox[4].y,msg->ObjectList[index].Abs_Bbox[4].z,
                  msg->ObjectList[index].Abs_Bbox[5].x,msg->ObjectList[index].Abs_Bbox[5].y,msg->ObjectList[index].Abs_Bbox[5].z,
                  msg->ObjectList[index].Abs_Bbox[6].x,msg->ObjectList[index].Abs_Bbox[6].y,msg->ObjectList[index].Abs_Bbox[6].z,
                  msg->ObjectList[index].Abs_Bbox[7].x,msg->ObjectList[index].Abs_Bbox[7].y,msg->ObjectList[index].Abs_Bbox[7].z;
      */
      pObj->center(0) = msg->ObjectList[index].Rel_Pos.x+time_diff*msg->ObjectList[index].Rel_Vel.x;
      pObj->center(1) = msg->ObjectList[index].Rel_Pos.y+time_diff*msg->ObjectList[index].Rel_Vel.y;
      pObj->center(2) = msg->ObjectList[index].Rel_Pos.z+time_diff*msg->ObjectList[index].Rel_Vel.z;

      pObj->velocity(0) = msg->ObjectList[index].Rel_Vel.x;
      pObj->velocity(1) = msg->ObjectList[index].Rel_Vel.y;
      pObj->velocity(2) = msg->ObjectList[index].Rel_Vel.z;

      pObj->position_uncertainty(0, 0) = msg->ObjectList[index].Rel_Pos_Stdev.x;
      pObj->position_uncertainty(1, 1) = msg->ObjectList[index].Rel_Pos_Stdev.y;
      pObj->position_uncertainty(2, 2) = msg->ObjectList[index].Rel_Pos_Stdev.z;

      pObj->velocity_uncertainty(0, 0) = msg->ObjectList[index].Rel_Vel_Stdev.x;
      pObj->velocity_uncertainty(1, 1) = msg->ObjectList[index].Rel_Vel_Stdev.y;
      pObj->velocity_uncertainty(2, 2) = msg->ObjectList[index].Rel_Vel_Stdev.z;
  
      pObj->theta = msg->ObjectList[index].Rel_Bbox_Angle.heading;

      //pObj->length = msg->ObjectList[index].Length;
      // pObj->width = msg->ObjectList[index].Width;
      pObj->length = std::max(msg->ObjectList[index].Rel_Range.xMax-msg->ObjectList[index].Rel_Range.xMin,
                            msg->ObjectList[index].Rel_Range.yMax-msg->ObjectList[index].Rel_Range.yMin);
      pObj->width = std::min(msg->ObjectList[index].Rel_Range.xMax-msg->ObjectList[index].Rel_Range.xMin,
                            msg->ObjectList[index].Rel_Range.yMax-msg->ObjectList[index].Rel_Range.yMin);
      pObj->height = msg->ObjectList[index].Rel_Range.zMax-msg->ObjectList[index].Rel_Range.zMin;

      //pObj->type = msg->ObjectList[index].Obj_Type;

      pObj->bbox<<msg->ObjectList[index].Rel_Bbox[0].x,msg->ObjectList[index].Rel_Bbox[0].y,msg->ObjectList[index].Rel_Bbox[0].z,
                  msg->ObjectList[index].Rel_Bbox[1].x,msg->ObjectList[index].Rel_Bbox[1].y,msg->ObjectList[index].Rel_Bbox[1].z,
                  msg->ObjectList[index].Rel_Bbox[2].x,msg->ObjectList[index].Rel_Bbox[2].y,msg->ObjectList[index].Rel_Bbox[2].z,
                  msg->ObjectList[index].Rel_Bbox[3].x,msg->ObjectList[index].Rel_Bbox[3].y,msg->ObjectList[index].Rel_Bbox[3].z,
                  msg->ObjectList[index].Rel_Bbox[4].x,msg->ObjectList[index].Rel_Bbox[4].y,msg->ObjectList[index].Rel_Bbox[4].z,
                  msg->ObjectList[index].Rel_Bbox[5].x,msg->ObjectList[index].Rel_Bbox[5].y,msg->ObjectList[index].Rel_Bbox[5].z,
                  msg->ObjectList[index].Rel_Bbox[6].x,msg->ObjectList[index].Rel_Bbox[6].y,msg->ObjectList[index].Rel_Bbox[6].z,
                  msg->ObjectList[index].Rel_Bbox[7].x,msg->ObjectList[index].Rel_Bbox[7].y,msg->ObjectList[index].Rel_Bbox[7].z;
      ADEBUG <<"Lidar Object Information : " 
        <<"ID: "<<pObj->track_id<<" length = "<<pObj->length<<" width: "<<pObj->width<<" orientation angle : "<<pObj->theta \
        <<" position: "<<pObj->center(0)<<" , "<<pObj->center(1)<<" velocity: "<<pObj->velocity(0)<<" "<<pObj->velocity(1)
        <<" object type: "<<msg->ObjectList[index].Obj_Type
        <<" covarianect: "<<pObj->position_uncertainty(0, 0)<<" , "<<pObj->position_uncertainty(1, 1)
        <<"xMax: "<<msg->ObjectList[index].Rel_Range.xMax
        <<"xMin: "<<msg->ObjectList[index].Rel_Range.xMin;
      //add bounding box
      objToBBox(pObj,glb_auto_perception_sensorfusion::SensorType::SutengJuChuang);

      lidar_fl_objects_->objects.push_back(pObj);
    }
  }
  // //Publish visualization 
  lidar_bbox_publisher_.publish(lidar_bbox_list_);

  lidar_fl_objects_->sensor_type = glb_auto_perception_sensorfusion::SensorType::SutengJuChuang;
  lidar_fl_objects_->sensor_id = GetSensorType(radar_objects_->sensor_type);
  //lidar_fl_objects_->timestamp = static_cast<double>(msg->header.stamp.sec)+static_cast<double>(msg->header.stamp.nsec)/1.0e9;
  lidar_fl_objects_->timestamp = curr_time;
  lidar_fl_objects_->sensor2world_pose = Eigen::Matrix4d::Identity();
  // fusion
  processFusion(lidar_fl_objects_);
  lidar_fl_objects_->objects.clear();
  // auto data_manager =glb_auto_perception_sensorfusion::SortedSensorFramePoolManager::getInstance();
  // data_manager->addNewFrame(lidar_fl_objects_);
  // lidar_fl_objects_.reset(new glb_auto_perception_sensorfusion::SensorObjects);
  return;
}

void PerceptionYX::processFusion(
    std::shared_ptr<glb_auto_perception_sensorfusion::SensorObjects> &in_sensor_objects)
{
  AWARN_IF(log_fusion_) << "\033[1;32mprocess Fusion\033[0m,"
                        << " at time stamp: " << std::fixed << std::setprecision(12) << in_sensor_objects->timestamp;

  // fused objects
  std::vector<std::shared_ptr<glb_auto_perception_sensorfusion::Object>> fused_objects;

  // process fusion
  double time_start = clock();
  obstacle_fusion_.Process(in_sensor_objects, &fused_objects);

  double pub_time=clock();
  // publish fusion results
  fusion_pub_mutex_.lock();
  publishFusionMarker(fused_objects, in_sensor_objects->timestamp);
  publishFusionResult(fused_objects, in_sensor_objects->timestamp);
  fusion_pub_mutex_.unlock();
  // calculate time
  fused_objects.clear();
  double pub_process_time = double(clock() - pub_time) / CLOCKS_PER_SEC;
  AWARN_IF(log_fusion_) << "\033[1;33m"
                        << "publish processing takes: "
                        << std::fixed << std::setprecision(6) << pub_process_time << "s at "
                        << 1.0 / pub_process_time << "Hz"
                        << "\033[0m";
  double fusion_process_time = double(clock() - time_start) / CLOCKS_PER_SEC;
  AWARN_IF(log_fusion_) << "\033[1;33m"
                        << "Fusion processing takes: "
                        << std::fixed << std::setprecision(6) << fusion_process_time << "s at "
                        << 1.0 / fusion_process_time << "Hz"
                        << "\033[0m";
  return;
}

void PerceptionYX::processFusion(){

  auto data_manager =glb_auto_perception_sensorfusion::SortedSensorFramePoolManager::getInstance();
  while (true)
  {
        
    std::shared_ptr<SensorObjects> in_sensor_objects=data_manager->getNewestFrame();
    if(nullptr!=in_sensor_objects){
      AWARN_IF(log_fusion_) << "\033[1;32mprocess Fusion\033[0m,"
                            << " at time stamp: " << std::fixed << std::setprecision(12) << in_sensor_objects->timestamp;
      // fused objects
      std::vector<std::shared_ptr<glb_auto_perception_sensorfusion::Object>> fused_objects;

      // process fusion
      double time_start = clock();
      obstacle_fusion_.Process(in_sensor_objects, &fused_objects);

      // publish fusion results
      publishFusionMarker(fused_objects, in_sensor_objects->timestamp);
      publishFusionResult(fused_objects, in_sensor_objects->timestamp);

      // calculate time
      double fusion_process_time = double(clock() - time_start) / CLOCKS_PER_SEC;
      AWARN_IF(log_fusion_) << "size of result" <<fused_objects.size() <<"\033[1;33m"
                            << "Fusion processing takes: "
                            << std::fixed << std::setprecision(6) << fusion_process_time << "s at "
                            << 1.0 / fusion_process_time << "Hz"
                            << "\033[0m";
    
    }

    std::this_thread::sleep_for(std::chrono:: milliseconds (50));
  }
 return;
}
void PerceptionYX::publishFusionMarker(
    std::vector<std::shared_ptr<glb_auto_perception_sensorfusion::Object>> &fused_objects,
    double timestamp)
{
  // fusion bbox line list definition
  ros::Time stamp = ros::Time::now();
  fusion_bbox_list_.header.stamp = stamp;
  fusion_bbox_list_.points.clear();
  fusion_bbox_list_.colors.clear();
  fusion_velocity_list_.markers.clear();

  // build bounding boxes / velocity markers
  int marker_count = 300;
  double marker_arrow_length = 1.0;
  for (auto& obj : fused_objects)
  {
    // add bounding box
    objToBBox(obj, glb_auto_perception_sensorfusion::SensorType::FUSION);
    // add velocity marker
    // if (obj->velocity.norm() < 0.2) {  // skip close to static objects
    //   continue;
    // }
    // AWARN_IF(log_fusion_) << "fusion moving obj track id: " << obj->track_id;
    // AWARN_IF(log_fusion_) << "fusion moving obj velocity: " << obj->velocity;
    // velocity text
    std::string ns = "fusion velocity text list";
    visualization_msgs::Marker velocity_text_marker = objToMarker(obj,
                                                                  frame_id_fusion_, stamp, ns,
                                                                  marker_count, visualization_msgs::Marker::TEXT_VIEW_FACING);
    fusion_velocity_list_.markers.push_back(velocity_text_marker);
    marker_count++;
    // velocity arrow
    visualization_msgs::Marker velocity_arrow_marker = objToMarker(obj,
                                                                   frame_id_fusion_, stamp, ns,
                                                                   marker_count, visualization_msgs::Marker::ARROW);
    fusion_velocity_list_.markers.push_back(velocity_arrow_marker);
    marker_count++;
  }

  // publish bounding boxes / velocity
  fusion_bbox_publisher_.publish(fusion_bbox_list_);
  fusion_velocity_publisher_.publish(fusion_velocity_list_);

  return;
}

void PerceptionYX::publishFusionResult(
    std::vector<std::shared_ptr<glb_auto_perception_sensorfusion::Object>> &fused_objects,
    double timestamp){
  fusion_result_.header.stamp.sec=static_cast<int>(timestamp);
  fusion_result_.header.stamp.nsec=static_cast<int>((timestamp-fusion_result_.header.stamp.sec)*10e9);
  fusion_result_.ObjectList.clear();
  int numbers=0;
  fusion_msgs::FusionObject fobj;
  for (const auto& obj : fused_objects)
  {
    fobj.ID=obj->id;

    fobj.Rel_Pos.x=obj->center(0);
    fobj.Rel_Pos.y=obj->center(1);
    fobj.Rel_Pos.z=obj->center(2);

    fobj.Rel_Vel.x=obj->velocity(0);
    fobj.Rel_Vel.y=obj->velocity(1);
    fobj.Rel_Vel.z=obj->velocity(2);

    fobj.Rel_Bbox_Angle.heading=obj->theta;
    fobj.Length=obj->length;
    fobj.Width=obj->width;
    fobj.Height=obj->height;
    fobj.Obj_Type=static_cast<uint8_t>(obj->type);
    fusion_result_.ObjectList.emplace_back(fobj);
    ++numbers;
  }
  fusion_result_.ObjNum=numbers;
  fusion_result_publisher_.publish(fusion_result_);
  return;
}

bool PerceptionYX::getSensorTrans(const double query_time, Eigen::Matrix4d *trans,
                                  std::string parent_frame_id, std::string child_frame_id)
{
  if (!trans)
  {
    AERROR << "failed to get trans, the trans ptr can not be nullptr";
    return false;
  }

  ros::Time query_stamp(query_time);
  // const double kTf2BuffSize = FLAGS_tf2_buff_in_ms / 1000.0;
  std::string err_msg;
  if (!tf2_listener_.canTransform(parent_frame_id,
                                  child_frame_id,
                                  query_stamp,
                                  //  ros::Time(0),
                                  &err_msg))
  {
    AERROR << "Cannot transform frame: " << parent_frame_id
           << " to frame " << child_frame_id
           << " , err: " << err_msg
           << ". Frames: " << tf2_listener_.allFramesAsString();
    return false;
  }

  tf::StampedTransform stamped_transform;
  geometry_msgs::TransformStamped transform_stamped;
  try
  {
    tf2_listener_.lookupTransform(parent_frame_id,
                                  child_frame_id, query_stamp, stamped_transform);
  }
  catch (tf::TransformException &ex)
  {
    AERROR << "Exception: " << ex.what();
    return false;
  }
  Eigen::Affine3d affine_3d;
  tf::transformTFToEigen(stamped_transform, affine_3d);
  *trans = affine_3d.matrix();
  // AWARN << "get " << parent_frame_id << " to "
  //         << child_frame_id << " trans: \n" << *trans;
  return true;
}

} // namespace perception_yx
