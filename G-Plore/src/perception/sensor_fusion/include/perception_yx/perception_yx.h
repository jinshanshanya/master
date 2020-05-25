
#ifndef _PERCEPTION_YX_H_
#define _PERCEPTION_YX_H_

#include "common/object.h"
#include "perception_yx/obstacle_fusion.h"
#include "data_manager/data_manager.h"


#include"fusion_msgs/FusionObjectList.h"
#include "lidar_msgs/LidarObjectList.h"
#include "radar_msgs/RadarObjectList.h"

#include "ros/ros.h"
#include "ros/time.h"
#include "ros/publisher.h"
#include "ros/subscriber.h"

#include "common/types.h"
#include "tf/transform_listener.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <Eigen/StdVector>
#include "yaml-cpp/yaml.h"
#include <pthread.h>
#include <mutex>


namespace perception_yx {

class CameraCalibration {
 public:
  //  intrinsic
  //cv::Mat cameraK;
  //cv::Mat cameraD;
  //  extrinsic
  std::vector<double> cameraT;  //  x y z
  std::vector<double> cameraR;  //  q0(w) q1(x) q2(y) q3(z)
};

class PerceptionYX
{
public:

  /** Constructor
   *
   *  @param node NodeHandle of this instance
   *  @param private_nh private NodeHandle of this instance
   */
  PerceptionYX(ros::NodeHandle node, ros::NodeHandle private_nh);
  ~PerceptionYX();

  /** callback to process message input
   *
   *  @param scan vector of input 3D data points
   *  @param stamp time stamp of data
   *  @param frame_id data frame of reference
   */
  void processLidarMessage(const lidar_msgs::LidarObjectListConstPtr& msg ,const glb_auto_perception_sensorfusion::SensorType sensor);

  //void processCameraMessage(const CameraMsg::VisObjsConstPtr& msg);

  void processRadarMessage(const radar_msgs::RadarObjectListConstPtr& msg);

  int parseCameraCalibration(const std::string contents, double resize_factor);

private:

  static unsigned char CLIPVALUE(int val) {
    // Old method (if)
    val = val < 0 ? 0 : val;
    return val > 255 ? 255 : val;

    // New method (array)
    // return uchar_clipping_table[val + clipping_table_offset];
  }


  void processFusion(
          std::shared_ptr<glb_auto_perception_sensorfusion::SensorObjects> &in_sensor_objects);
    
  void processFusion();

  void publishFusionMarker(std::vector<std::shared_ptr<glb_auto_perception_sensorfusion::Object>> &fused_objects, 
                      double timestamp);

  void publishFusionResult(std::vector<std::shared_ptr<glb_auto_perception_sensorfusion::Object>> &fused_objects, 
                      double timestamp);
  bool getSensorTrans(const double query_time, Eigen::Matrix4d* trans, 
                      std::string parent_frame_id, std::string child_frame_id);

  visualization_msgs::Marker objToMarker(std::shared_ptr<glb_auto_perception_sensorfusion::Object> &obj,
                                                     std::string &frame_id,
                                                     ros::Time &timestamp,
                                                     std::string &ns,
                                                     int32_t id,
                                                     int32_t type);
  void objToBBox(std::shared_ptr<glb_auto_perception_sensorfusion::Object> &obj, 
                                 glb_auto_perception_sensorfusion::SensorType sensor_type);
  tf::TransformListener tf2_listener_;

  // Sensor control
  bool use_front_left_lidar_ = false;
  bool use_front_right_lidar_ = false;
  bool use_rear_left_lidar_ = false;
  bool use_rear_right_lidar_ = false;
  bool use_cam_ = false;
  bool use_radar_ = false;

  // Parameters for logging
  bool log_cam_;
  bool log_radar_;
  bool log_lidar_;
  //bool log_front_left_lidar_;
  //bool log_front_right_lidar_;
  //bool log_rear_left_lidar_;
  //bool log_rear_right_lidar_;
  bool log_fusion_;

  // Frame IDs
  std::string frame_id_cam_;
  std::string frame_id_lidar_;
  std::string frame_id_radar_;
  std::string frame_id_fusion_;
  
  // Parameters for cameras
  CameraCalibration calibs_[2];

  double extrinsic_[2][6];



  // Parameters for projection
  //pthread_mutex_t proj_lock_;

  // Markers for visualization
  visualization_msgs::Marker lidar_bbox_list_;               // id: 1
  visualization_msgs::MarkerArray lidar_velocity_list_;      // id: 100+
  visualization_msgs::Marker camera_bbox_list_;            // id: 10+
  visualization_msgs::Marker radar_bbox_list_;               // id: 20
  visualization_msgs::MarkerArray radar_velocity_list_;      // id: 200+
  
  visualization_msgs::Marker fusion_bbox_list_;              // id: 30

  visualization_msgs::MarkerArray fusion_velocity_list_;      // id: 300+

  fusion_msgs::FusionObjectList fusion_result_;

  bool vis_lidar_poly_;
  bool vis_cam_sphere_;
  bool vis_radar_sphere_;

  // Subscriber
  ros::Subscriber lidar_fl_message_;
  ros::Subscriber lidar_fr_message_;
  ros::Subscriber lidar_rl_message_;
  ros::Subscriber lidar_rr_message_;
  ros::Subscriber camera_message_;
  ros::Subscriber radar_conti_;

  // Markers publisher
  ros::Publisher fusion_bbox_publisher_;
  ros::Publisher fusion_velocity_publisher_;
  ros::Publisher lidar_bbox_publisher_;
  ros::Publisher lidar_velocity_publisher_; 
  ros::Publisher camera_bbox_publisher_;
  ros::Publisher radar_bbox_publisher_;
  ros::Publisher radar_velocity_publisher_;

  // Result publisher
  ros::Publisher fusion_result_publisher_;

  // obstacle detection
  /* std::unique_ptr<glb_auto_perception_sensorfusion::LidarProcess> lidar_fl_process_;
  std::unique_ptr<glb_auto_perception_sensorfusion::LidarProcess> lidar_fr_process_;
  std::unique_ptr<glb_auto_perception_sensorfusion::LidarProcess> lidar_rl_process_;
  std::unique_ptr<glb_auto_perception_sensorfusion::LidarProcess> lidar_rr_process_;
  std::unique_ptr<glb_auto_perception_sensorfusion::CameraProcess> camera_process_;
  std::unique_ptr<glb_auto_perception_sensorfusion::ModestRadarDetector> radar_process_; */

  // tracked objects
  std::shared_ptr<glb_auto_perception_sensorfusion::SensorObjects> lidar_fl_objects_;
  std::shared_ptr<glb_auto_perception_sensorfusion::SensorObjects> lidar_fr_objects_;
  std::shared_ptr<glb_auto_perception_sensorfusion::SensorObjects> lidar_rl_objects_;
  std::shared_ptr<glb_auto_perception_sensorfusion::SensorObjects> lidar_rr_objects_;
  std::shared_ptr<glb_auto_perception_sensorfusion::SensorObjects> camera_objects_;
  std::shared_ptr<glb_auto_perception_sensorfusion::SensorObjects> radar_objects_;

  // obstacle fusion
  std::mutex fusion_pub_mutex_;
  glb_auto_perception_sensorfusion::ObstacleFusion obstacle_fusion_;
};

} // namespace velodyne_perception_yx

#endif
