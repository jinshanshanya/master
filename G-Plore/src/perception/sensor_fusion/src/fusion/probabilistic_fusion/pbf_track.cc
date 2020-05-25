/******************************************************************************
 * Copyright 2017 The glb_auto Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "fusion/probabilistic_fusion/pbf_track.h"

#include <algorithm>
#include <vector>
#include <numeric>
#include <cfloat>
#include "boost/format.hpp"


//#include "util/geometry_util.h"
#include "common/types.h"
#include "common/perception_gflags.h"

#include "fusion/probabilistic_fusion/pbf_base_track_object_matcher.h"
#include "fusion/probabilistic_fusion/pbf_imf_fusion.h"
#include "fusion/probabilistic_fusion/pbf_kalman_motion_fusion.h"
#include "fusion/probabilistic_fusion/pbf_sensor_manager.h"

namespace glb_auto_perception_sensorfusion {

/*class PbfTrack*/
int PbfTrack::s_track_idx_ = 0;
double PbfTrack::s_max_lidar_invisible_period_ = 0.25;
double PbfTrack::s_max_radar_invisible_period_ = 0.15;
double PbfTrack::s_max_camera_invisible_period_ = 0.25;
double PbfTrack::s_max_radar_confident_angle_ = 20;
double PbfTrack::s_min_radar_confident_distance_ = 40;

bool PbfTrack::s_publish_if_has_lidar_ = true;
bool PbfTrack::s_publish_if_has_radar_ = true;

PbfTrack::MotionFusionMethod PbfTrack::s_motion_fusion_method_ =
    PbfTrack::MotionFusionMethod::PBF_KALMAN;

// bba manager ptr
const BBAManager *PbfTrack::_s_classify_manager_ptr =
    &BBAManager::instance("classify");

std::map<SensorType, double> PbfTrack::_s_sensor_factors = {{SensorType::VELODYNE_64, 0.5},
                                                            {SensorType::CAMERA, 0.95},
                                                            {SensorType::PANDORA_HESAI40, 0.5},
                                                            {SensorType::PANDORA_CAMERA, 0.95}
                                                           };

std::map<SensorType, double> PbfTrack::_s_sensor_factors_for_unknown = {
    {SensorType::VELODYNE_64, 0.5}, {SensorType::CAMERA, 0.95},
    {SensorType::PANDORA_HESAI40, 0.5}, {SensorType::PANDORA_CAMERA, 0.95}
};

PbfTrack::PbfTrack(std::shared_ptr<PbfSensorObject> obj, bool log_fusion)
    : _fused_bba(_s_classify_manager_ptr) {
  // id
  idx_ = GetNextTrackId();

  // log
  log_fusion_ = log_fusion;
  if (s_motion_fusion_method_ == MotionFusionMethod::PBF_KALMAN) {
    motion_fusion_.reset(new PbfKalmanMotionFusion());
  } else {
    motion_fusion_.reset(new PbfIMFFusion());
    // if (idx_ == 6) {
    //   std::dynamic_pointer_cast<PbfIMFFusion>(motion_fusion_)->setLogFusionIMF(log_fusion_);
    // }
  }

  invisible_in_lidar_ = true;
  invisible_in_radar_ = true;
  invisible_in_camera_ = true;
  SensorType sensor_type = obj->sensor_type;
  std::string sensor_id = obj->sensor_id;
  if (is_lidar(sensor_type)) {
    lidar_objects_[sensor_id] = obj;
    motion_fusion_->Initialize(obj);
    invisible_in_lidar_ = false;
  } else if (is_radar(sensor_type)) {
    radar_objects_[sensor_id] = obj;
    motion_fusion_->Initialize(obj);
    invisible_in_radar_ = false;
  } else if (is_camera(sensor_type)) {
    camera_objects_[sensor_id] = obj;
    motion_fusion_->Initialize(obj);
    invisible_in_camera_ = false;
  } else {
    AERROR << "Unsupported sensor type : " << static_cast<int>(sensor_type)
           << ", sensor id : " << sensor_id;
  }

  motion_fusion_->setLastFuseTS(obj->timestamp);
  fused_timestamp_ = obj->timestamp;
  fused_object_.reset(new PbfSensorObject());
  fused_object_->clone(*obj);

  // PerformClassFusion(obj);

  age_ = 0;
  invisible_period_ = 0;
  tracking_period_ = 0.0;
  is_dead_ = false;
}

void PbfTrack::SetMotionFusionMethod(const std::string &motion_fusion_method) {
  if (motion_fusion_method == "PbfKalmanMotionFusion") {
    s_motion_fusion_method_ = MotionFusionMethod::PBF_KALMAN;
  } else if (motion_fusion_method == "PbfIMFFusion") {
    s_motion_fusion_method_ = MotionFusionMethod::PBF_IMF;
  } else {
    AERROR << "Unknown motion fusion method.";
  }
}

void PbfTrack::SetFusedLidarStatus(bool fused_lidar) {
  fused_object_->object->fused_lidar = fused_lidar;
  return;
}

void PbfTrack::SetFusedRadarStatus(bool fused_radar) {
  fused_object_->object->fused_radar = fused_radar;
  return;
}

void PbfTrack::SetFusedCamStatus(bool fused_cam) {
  fused_object_->object->fused_cam = fused_cam;
  return;
}

PbfTrack::~PbfTrack() {}

void PbfTrack::UpdateWithSensorObject(std::shared_ptr<PbfSensorObject> obj,
                                      double match_dist) {
  const SensorType &sensor_type = obj->sensor_type;
  const std::string sensor_id = obj->sensor_id;
  AWARN_IF(log_fusion_) << ".......... update WITH sensor object" 
        << ", track id " << GetTrackId()
        << ", sensor id " << sensor_id
        << ", type " << GetObjectName(obj->object->type);
  if (FLAGS_async_fusion) {
    PerformMotionFusionAsync(obj);
    // std::cerr << "PBFIMF:track id is: " << GetTrackId() << std::endl;
  } else {
    PerformMotionFusion(obj);
  }

  // Hard fix divergent radar tracked object
  double distance = (obj->object->center - GetFusedObject()->object->center).head(2).norm();
  if (is_radar(sensor_type) && distance > 4.0) {  //TODO: \attention ,hard code ,need to imprpve
    radar_objects_.erase(sensor_id);
    is_dead_ = (lidar_objects_.empty() && radar_objects_.empty() &&
              camera_objects_.empty());
    if (is_dead_) {
      return;
    }
  }

  // update last observation
  if (is_lidar(sensor_type)) {
    lidar_objects_[sensor_id] = obj;
    invisible_in_lidar_ = false;
  } else if (is_radar(sensor_type)) {
    radar_objects_[sensor_id] = obj;
    invisible_in_radar_ = false;
  } else if (is_camera(sensor_type)) {
    camera_objects_[sensor_id] = obj;
    invisible_in_camera_ = false;
  }
  
  double timestamp = obj->timestamp;
  UpdateMeasurementsLifeWithMeasurement(&lidar_objects_, sensor_id, timestamp,
                                        s_max_lidar_invisible_period_);
  UpdateMeasurementsLifeWithMeasurement(&radar_objects_, sensor_id, timestamp,
                                        s_max_radar_invisible_period_);
  UpdateMeasurementsLifeWithMeasurement(&camera_objects_, sensor_id, timestamp,
                                        s_max_camera_invisible_period_);

  invisible_period_ = 0;
  tracking_period_ += obj->timestamp - fused_timestamp_;
  // AWARN << "... update interval: " << obj->timestamp - fused_timestamp_
  //   << " at freq: " << 1.0 / (obj->timestamp - fused_timestamp_);
  fused_timestamp_ = obj->timestamp;
  fused_object_->timestamp = obj->timestamp;

  // AWARN_IF(log_fusion_) << "ASSIGNED track id: " << GetTrackId() 
  //                       << " lidar_objects_.size(): " << lidar_objects_.size()
  //                       << " radar_objects_.size(): " << radar_objects_.size();
                        // << " camera_objects_.size(): " << camera_objects_.size();

  // class fusion
  if (!FLAGS_use_navigation_mode) {
    // AWARN_IF(log_fusion_) << "class fusion performed";
    PerformClassFusion(obj);
  } else {
    // AWARN_IF(log_fusion_) << "class fusion skipped in navigation mode";
  }
  
  return;
}

bool PbfTrack::IsInCameraView(const Eigen::Vector3d &center, double timestamp) {
  // // read transformation matrix from calibration config manager
  // CalibrationConfigManager *calibration_config_manager =
  //     Singleton<CalibrationConfigManager>::get();

  // const CameraCalibrationPtr camera_calibration =
  //     calibration_config_manager->get_camera_calibration();

  // const CameraDistortPtr camera_distort =
  //     camera_calibration->get_camera_model();
  // Eigen::Matrix4d camera_trans;

  // if (!GetCameraTrans(timestamp, &camera_trans)) {
  //   AERROR << "failed to get trans at timestamp: " << timestamp;
  //   return false;
  // }

  // Eigen::Matrix4d world_to_camera = camera_trans.inverse();

  // return camera_distort->is_in_view(world_to_camera, center);
}

void PbfTrack::UpdateWithoutSensorObject(const SensorType &sensor_type,
                                         const std::string &sensor_id,
                                         double min_match_dist,
                                         double timestamp) {
  AINFO << "......... update WITHOUT sensor object with track id " << GetTrackId()
        << " with sensor id " << sensor_id;

  const Eigen::Vector3d &center = fused_object_->object->center;

  if (motion_fusion_ == nullptr) {
    AERROR << "Skip update becuase motion_fusion_ is nullptr.";
    return;
  }

  AINFO << "Before UpdateMeasurementsLifeWithoutMeasurement : "
    << " size of lidar_objects_: " << lidar_objects_.size()
    << " size of radar_objects_: " << radar_objects_.size()
    << " size of camera_objects_: " << camera_objects_.size();

  UpdateMeasurementsLifeWithoutMeasurement(
      &lidar_objects_, sensor_id, timestamp, s_max_lidar_invisible_period_,
      &invisible_in_lidar_);
  UpdateMeasurementsLifeWithoutMeasurement(
      &radar_objects_, sensor_id, timestamp, s_max_radar_invisible_period_,
      &invisible_in_radar_);
  UpdateMeasurementsLifeWithoutMeasurement(
      &camera_objects_, sensor_id, timestamp, s_max_camera_invisible_period_,
      &invisible_in_camera_);

  AINFO << "After UpdateMeasurementsLifeWithoutMeasurement : "
    << " size of lidar_objects_: " << lidar_objects_.size()
    << " size of radar_objects_: " << radar_objects_.size()
    << " size of camera_objects_: " << camera_objects_.size();

  // decide if object is dead
  is_dead_ = (lidar_objects_.empty() && radar_objects_.empty() &&
              camera_objects_.empty());

  // AWARN_IF(log_fusion_) << "UNASSIGNED track id: " << GetTrackId() 
  //                       << "lidar_objects_.size(): " << lidar_objects_.size()
  //                       << "radar_objects_.size(): " << radar_objects_.size();
                        // << "camera_objects_.size(): " << camera_objects_.size();
  
  if (!is_dead_) {
    double time_diff = (timestamp - fused_timestamp_)<=0?0:(timestamp - fused_timestamp_);
    //@attention: 考虑拿到之前时间的数据，所以此处特殊处理，待
    motion_fusion_->UpdateWithoutObject(time_diff);  // update belief_anchor_point_
    if (FLAGS_use_navigation_mode) {
      PerformMotionCompensation(fused_object_, timestamp);  // predict obj properties
    }
    invisible_period_ = timestamp - fused_timestamp_;
  }

  // class fusion, only recalculate type if in the field of view of camera
  if (!is_dead_ && !FLAGS_use_navigation_mode && is_camera(sensor_type) &&
      //IsInCameraView(center, timestamp) &&
      fused_object_->object->type != ObjectType::VEHICLE) {
    std::map<uint64_t, double> fp_bba_map = {
        {DSTInitiator::OTHERS_UNMOVABLE, 0.8},
        {DSTInitiator::OTHERS_MOVABLE, 0.2}};

    BBA fp_bba(_s_classify_manager_ptr);
    fp_bba.set_bba(fp_bba_map);
    _fused_bba = _fused_bba + fp_bba * GetUnknownReliablityFactor(sensor_type);
    AINFO << "camera without sensor for object id " << fused_object_->object->id
          << "with type " << GetObjectName(fused_object_->object->type);
    DecideObjectType();
    AINFO << "camera without after type fusion " << fused_object_->object->id
          << "with type " << GetObjectName(fused_object_->object->type);
  }
}

int PbfTrack::GetTrackId() const { return idx_; }

std::shared_ptr<PbfSensorObject> PbfTrack::GetFusedObject() {
  return fused_object_;
}

double PbfTrack::GetFusedTimestamp() const { return fused_timestamp_; }

std::shared_ptr<PbfSensorObject> PbfTrack::GetLidarObject(
    const std::string &sensor_id) {
  std::shared_ptr<PbfSensorObject> obj = nullptr;
  auto it = lidar_objects_.find(sensor_id);
  if (it != lidar_objects_.end()) {
    obj = it->second;
  }
  return obj;
}

std::shared_ptr<PbfSensorObject> PbfTrack::GetRadarObject(
    const std::string &sensor_id) {
  std::shared_ptr<PbfSensorObject> obj = nullptr;
  auto it = radar_objects_.find(sensor_id);
  if (it != radar_objects_.end()) {
    obj = it->second;
  }
  return obj;
}

std::shared_ptr<PbfSensorObject> PbfTrack::GetCameraObject(
    const std::string &sensor_id) {
  std::shared_ptr<PbfSensorObject> obj = nullptr;
  auto it = camera_objects_.find(sensor_id);
  if (it != camera_objects_.end()) {
    obj = it->second;
  }
  return obj;
}

void PbfTrack::PerformMotionFusionAsync(std::shared_ptr<PbfSensorObject> obj) {
  if (motion_fusion_ == nullptr) {
    AERROR << "Skip motion fusion becuase motion_fusion_ is nullptr.";
    return;
  }
  ADEBUG << "perform motion fusion asynchrounously!";
  const SensorType &sensor_type = obj->sensor_type;

  //double current_time = ros::Time::now().toSec();
  // AWARN_IF(log_fusion_) << std::fixed << std::setprecision(15)
  //       << "last fuse ts " << motion_fusion_->getLastFuseTS()
  //       << ", obj timestamp " << obj->timestamp 
  //       << ", time diff " << obj->timestamp - motion_fusion_->getLastFuseTS();

  // for low cost, we only consider radar and camera fusion for now
  if (is_camera(sensor_type) || is_radar(sensor_type) ||
      is_lidar(sensor_type)) {
    if (is_camera(sensor_type)) {
      AINFO << "camera sensor in async fusion";
    }
    if (is_radar(sensor_type)) {
      AINFO << "radar sensor in async fusion";
    }
    if (is_lidar(sensor_type)) {
      AINFO << "lidar sensor in async fusion";
    }

    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();

    // set current fuse time as current observation time
    double current_time = obj->timestamp; 
    motion_fusion_->setCurrentFuseTS(current_time);

    if (motion_fusion_->Initialized()) {
      double time_diff = motion_fusion_->getFuseTimeDiff();
      motion_fusion_->UpdateWithObject(obj, time_diff);
    } else {
      motion_fusion_->Initialize(obj);
      current_time = obj->timestamp;  // for initialize, we set fusion time to
                                      // sensor timestamp
    }
    Eigen::Vector3d anchor_point;
    motion_fusion_->GetState(&anchor_point, &velocity);
    fused_object_->object->velocity = velocity;
    fused_object_->object->anchor_point = anchor_point;
    fused_object_->object->center = anchor_point;
    if (is_lidar(sensor_type) || 
          (is_camera(sensor_type) && !fused_object_->object->fused_lidar)) {
      fused_object_->object->theta = obj->object->theta;
      fused_object_->object->direction = obj->object->direction;
      fused_object_->object->length = obj->object->length;
      fused_object_->object->width = obj->object->width;
      fused_object_->object->height = obj->object->height;
    }
    // updated by arrival time of sensor object
    fused_object_->timestamp = current_time;
    // AWARN_IF(log_fusion_) << "fused object in pbftrack is " << fused_object_->object->ToString();
    motion_fusion_->setLastFuseTS(current_time);
  }
}

void PbfTrack::PerformMotionFusion(std::shared_ptr<PbfSensorObject> obj) {
  if (motion_fusion_ == nullptr) {
    AERROR << "Skip motion fusion becuase motion_fusion_ is nullptr.";
    return;
  }
  const SensorType &sensor_type = obj->sensor_type;
  double time_diff = obj->timestamp - fused_object_->timestamp;

  std::shared_ptr<PbfSensorObject> lidar_object = GetLatestLidarObject();
  std::shared_ptr<PbfSensorObject> radar_object = GetLatestRadarObject();

  if (FLAGS_use_navigation_mode) {
    if (is_camera(sensor_type) || is_radar(sensor_type)) {
      if (motion_fusion_->Initialized()) {
        motion_fusion_->UpdateWithObject(obj, time_diff);
        Eigen::Vector3d anchor_point;
        Eigen::Vector3d velocity;
        // use radar position and velocity
        if (is_radar(sensor_type)) {
          motion_fusion_->SetState(obj->object->center, obj->object->velocity);
        }
        motion_fusion_->GetState(&anchor_point, &velocity);
        fused_object_->object->velocity = velocity;
        fused_object_->object->anchor_point = anchor_point;
        fused_object_->object->center = anchor_point;

        if (is_camera(sensor_type)) {
          fused_object_->object->theta = obj->object->theta;
          fused_object_->object->direction = obj->object->direction;
        }
      } else {
        if (is_camera(sensor_type)) {
          motion_fusion_->Initialize(obj);
        }
      }
    }
  } else {
    if (is_lidar(sensor_type) || is_camera(sensor_type)) {
      fused_object_->clone(*obj);
      if (motion_fusion_->Initialized() &&
          (lidar_object != nullptr || radar_object != nullptr)) {
        motion_fusion_->UpdateWithObject(obj, time_diff);
        Eigen::Vector3d anchor_point;
        Eigen::Vector3d velocity;
        motion_fusion_->GetState(&anchor_point, &velocity);
        fused_object_->object->velocity = velocity;
      } else {
        motion_fusion_->Initialize(obj);
      }
    } else if (is_radar(sensor_type)) {
      if (motion_fusion_->Initialized() &&
          (lidar_object != nullptr || radar_object != nullptr)) {
        Eigen::Vector3d pre_anchor_point;
        Eigen::Vector3d pre_velocity;
        motion_fusion_->GetState(&pre_anchor_point, &pre_velocity);
        motion_fusion_->UpdateWithObject(obj, time_diff);
        Eigen::Vector3d anchor_point;
        Eigen::Vector3d velocity;
        motion_fusion_->GetState(&anchor_point, &velocity);
        if (lidar_object == nullptr) {
          PbfSensorObject fused_obj_bk;
          fused_obj_bk.clone(*fused_object_);
          fused_object_->clone(*obj);
          fused_object_->object->velocity = velocity;
        } else {
          // if has lidar, use lidar shape
          Eigen::Vector3d translation = anchor_point - pre_anchor_point;
          fused_object_->object->anchor_point = anchor_point;
          fused_object_->object->center += translation;
          //for (auto point : fused_object_->object->polygon.points) {
            //point.x += translation[0];
           // point.y += translation[1];
           // point.z += translation[2];
          //}
          fused_object_->object->velocity = velocity;
        }
      } else {
        AERROR << "Something must be wrong.";
      }
    } else {
      AERROR << "Unsupport sensor type " << static_cast<int>(sensor_type);
      return;
    }
  }
}

void PbfTrack::PerformMotionCompensation(std::shared_ptr<PbfSensorObject> obj,
                                         double timestamp) {
  double time_diff = timestamp - obj->timestamp;
  if (time_diff < 0) {
    AWARN_IF(log_fusion_) << "target timestamp is smaller than previous timestamp";
    return;
  }

  Eigen::Vector3d offset = obj->object->velocity * time_diff;
  obj->object->center += offset;
  obj->object->anchor_point += offset;

  /* PolygonDType &polygon = obj->object->polygon;
  for (size_t i = 0; i < polygon.size(); i++) {
    polygon.points[i].x += offset[0];
    polygon.points[i].y += offset[1];
    polygon.points[i].z += offset[2];
  } */

  /* pcl_util::PointCloudPtr cloud = obj->object->cloud;
  for (size_t i = 0; i < cloud->size(); i++) {
    cloud->points[i].x += offset[0];
    cloud->points[i].y += offset[1];
    cloud->points[i].z += offset[2];
  } */

  obj->timestamp = timestamp;
  obj->object->latest_tracked_time = timestamp;
  obj->object->tracking_time += time_diff;
}

int PbfTrack::GetNextTrackId() {
  int ret_track_id = s_track_idx_;
  if (s_track_idx_ == INT_MAX) {
    s_track_idx_ = 0;
  } else {
    ++s_track_idx_;
  }
  return ret_track_id;
}

bool PbfTrack::AbleToPublish() {
  return true;

  if (FLAGS_use_navigation_mode && !camera_objects_.empty()) {
    return true;
  }

  ADEBUG << s_publish_if_has_lidar_ << " " << invisible_in_lidar_ << " "
         << lidar_objects_.size();
  double invisible_period_threshold = 0.001;
  if (invisible_period_ > invisible_period_threshold && invisible_in_lidar_ &&
      invisible_in_radar_) {
    ADEBUG << "unable_to_publish: invisible " << invisible_period_;
    return false;
  }

  if (s_publish_if_has_lidar_ && !invisible_in_lidar_ &&
      !lidar_objects_.empty()) {
    return true;
  }

  std::shared_ptr<PbfSensorObject> radar_object = GetLatestRadarObject();
  if (s_publish_if_has_radar_ && !invisible_in_radar_ &&
      radar_object != nullptr) {
    if (radar_object->sensor_type == SensorType::RADAR) {
      if (radar_object->object->radar_supplement->range >
              s_min_radar_confident_distance_ &&
          radar_object->object->radar_supplement->angle <
              s_max_radar_confident_angle_) {
        if (fused_object_->object->velocity.dot(
                fused_object_->object->direction) < 0.3) {
          fused_object_->object->velocity.setZero();
          return false;
        }
        return true;
      }
    }
  }
  return false;
}

void PbfTrack::UpdateMeasurementsLifeWithMeasurement(
    std::map<std::string, std::shared_ptr<PbfSensorObject>> *objects,
    const std::string &sensor_id, double timestamp, double max_invisible_time) {
  for (auto it = objects->begin(); it != objects->end();) {
    if (sensor_id != it->first) {
      double period = timestamp - it->second->timestamp;
      if (period > max_invisible_time) {
        it->second = nullptr;
        it = objects->erase(it);
      } else {
        ++it;
      }
    } else {
      ++it;
    }
  }
}

void PbfTrack::UpdateMeasurementsLifeWithoutMeasurement(
    std::map<std::string, std::shared_ptr<PbfSensorObject>> *objects,
    const std::string &sensor_id, double timestamp, double max_invisible_time,
    bool *invisible_state) {
  *invisible_state = true;
  for (auto it = objects->begin(); it != objects->end();) {
    // if (sensor_id == it->first) {
    //   it->second = nullptr;
    //   it = objects->erase(it);
    // } else {
      double period = timestamp - it->second->timestamp;
      if (period > max_invisible_time) {
        it->second = nullptr;
        it = objects->erase(it);
      } else {
        *invisible_state = false;
        ++it;
      }
    // }
  }
}

std::shared_ptr<PbfSensorObject> PbfTrack::GetLatestLidarObject() {
  std::shared_ptr<PbfSensorObject> lidar_object;
  for (auto it = lidar_objects_.begin(); it != lidar_objects_.end(); ++it) {
    if (lidar_object == nullptr) {
      lidar_object = it->second;
    } else if (lidar_object->timestamp < it->second->timestamp) {
      lidar_object = it->second;
    }
  }
  return lidar_object;
}

std::shared_ptr<PbfSensorObject> PbfTrack::GetLatestRadarObject() {
  std::shared_ptr<PbfSensorObject> radar_object;
  for (auto it = radar_objects_.begin(); it != radar_objects_.end(); ++it) {
    if (radar_object == nullptr) {
      radar_object = it->second;
    } else if (radar_object->timestamp < it->second->timestamp) {
      radar_object = it->second;
    }
  }
  return radar_object;
}

std::shared_ptr<PbfSensorObject> PbfTrack::GetLatestCameraObject() {
  std::shared_ptr<PbfSensorObject> camera_object;
  for (auto it = camera_objects_.begin(); it != camera_objects_.end(); ++it) {
    if (camera_object == nullptr) {
      camera_object = it->second;
    } else if (camera_object->timestamp < it->second->timestamp) {
      camera_object = it->second;
    }
  }
  return camera_object;
}

std::shared_ptr<PbfSensorObject> PbfTrack::GetSensorObject(
    const SensorType &sensor_type, const std::string &sensor_id) {
  if (is_lidar(sensor_type)) {
    return GetLidarObject(sensor_id);
  } else if (is_radar(sensor_type)) {
    return GetRadarObject(sensor_id);
  } else if (is_camera(sensor_type)) {
    return GetCameraObject(sensor_id);
  } else {
    AERROR << "Unsupported sensor type.";
    return nullptr;
  }
}

// @brief use dst evidence theory to do the class type fusion
void PbfTrack::PerformClassFusion(std::shared_ptr<PbfSensorObject> obj) {
  if (obj->object->type == ObjectType::VEHICLE) {
    AINFO << "disable type fusion for vehicle type";
    return;
  }

  AINFO << "perform class fusion type is " << GetObjectName(obj->object->type);
  BBA sensor_obj_bba = TypeProbsToBba(obj->object->type_probs);
  // AINFO<<"sensor obj bba information " << sensor_obj_bba.print_bba();
  // AINFO<<"sensor reliability factor " <<
  // GetReliabilityFactor(obj->sensor_type);
  _fused_bba =
      _fused_bba + sensor_obj_bba * GetReliabilityFactor(obj->sensor_type);
  DecideObjectType();
}

void PbfTrack::DecideObjectType() {
  const DSTInitiator &dst_initiator = DSTInitiator::instance();
  const BBAManager &classify_manager = *_s_classify_manager_ptr;
  const std::vector<double> &fused_bba_vec = _fused_bba.get_bba_vec();
  auto max_iter = std::max_element(fused_bba_vec.begin(), fused_bba_vec.end());
  size_t max_hyp_ind = max_iter - fused_bba_vec.begin();
  uint64_t max_hyp = classify_manager.ind_to_fod_subset(max_hyp_ind);
  if (max_hyp == DSTInitiator::OTHERS_MOVABLE) {
    AERROR << "max hyp is UNKNOWN_MOVABLE" << _fused_bba.print_bba();
  }
  fused_object_->object->type =
      static_cast<ObjectType>(dst_initiator.hyp_to_typ(max_hyp));
  AINFO << "used object type is " << GetObjectName(fused_object_->object->type);
}

BBA PbfTrack::TypeProbsToBba(const std::vector<float> &type_probs) {
  const auto &classify_manager = *_s_classify_manager_ptr;
  const auto &dst_initiator = DSTInitiator::instance();
  CHECK(classify_manager.initialized());
  BBA res_bba(&classify_manager);
  double type_probs_sum =
      std::accumulate(type_probs.begin(), type_probs.end(), 0.0);
  if (type_probs_sum < DBL_MIN) {
    AWARN << "the sum of types probability equal 0.0";
    return res_bba;
  }
  if (type_probs.size() > (int)ObjectType::UNKNOWN_UNMOVABLE &&
      type_probs[static_cast<int>(ObjectType::UNKNOWN_UNMOVABLE)] > 0.0f) {
    ADEBUG << "unknonw_unmovable prob = "
           << type_probs[static_cast<int>(ObjectType::UNKNOWN_UNMOVABLE)]
          << " > 0.0f";
  }
  std::map<uint64_t, double> res_bba_map;
  for (size_t i = 0; i < type_probs.size(); ++i) {
    size_t typ = i;
    uint64_t hyp = 0;
    if (!dst_initiator.typ_to_hyp(typ, &hyp)) {
      continue;
    }
    res_bba_map[hyp] = type_probs[typ];
  }
  // for support camera probs detection prob
  // if (type_probs_sum < 1.0) {
  //     auto find_res = res_bba_map.find(DSTInitiator::OTHERS);
  //     if (find_res == res_bba_map.end()) {
  //         res_bba_map[DSTInitiator::OTHERS] = 1 - type_probs_sum;
  //     } else {
  //         find_res->second += (1 - type_probs_sum);
  //     }
  // }
  CHECK(res_bba.set_bba(res_bba_map));
  return res_bba;
}

double PbfTrack::GetReliabilityFactor(SensorType sensor_type) {
  auto find_res = _s_sensor_factors.find(sensor_type);
  if (find_res == _s_sensor_factors.end()) {
    AWARN << "the sensor type: " << (int)sensor_type
          << " is not supported by class fusion";
    return 0.0;
  }

  return find_res->second;
}

double PbfTrack::GetUnknownReliablityFactor(SensorType sensor_type) {
  auto find_res = _s_sensor_factors_for_unknown.find(sensor_type);
  if (find_res == _s_sensor_factors_for_unknown.end()) {
    AWARN << "the sensor type: " << (int)sensor_type
          << " is not supported by class fusion";
    return 0.0;
  }
  return find_res->second;
}

}  // namespace glb_auto
