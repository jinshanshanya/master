/******************************************************************************
 * Copyright 2017 The glb_auto Authors. All Rights Reserved.
 *****************************************************************************/

#include "common/types.h"

namespace glb_auto_perception_sensorfusion {

std::string GetObjectName(const ObjectType& obj_type) {
  std::string obj_name;
  switch (obj_type) {
    case ObjectType::UNKNOWN:
      obj_name = "unknown";
      break;
    case ObjectType::UNKNOWN_MOVABLE:
      obj_name = "unknown_movable";
      break;
    case ObjectType::UNKNOWN_UNMOVABLE:
      obj_name = "unknown_unmovable";
      break;
    case ObjectType::PEDESTRIAN:
      obj_name = "pedestrian";
      break;
    case ObjectType::BICYCLE:
      obj_name = "bicycle";
      break;
    case ObjectType::VEHICLE:
      obj_name = "vehicle";
      break;
    default:
      obj_name = "error";
      break;
  }
  return obj_name;
}

std::string GetSensorType(SensorType sensor_type) {
  switch (sensor_type) {
    case SensorType::VELODYNE_64:
      return "velodyne_64";
    case SensorType::VELODYNE_16:
      return "velodyne_16";
    case SensorType::RADAR:
      return "radar";
    case SensorType::CAMERA:
      return "camera";
    case SensorType::ULTRASONIC:
      return "ultrasonic";
    case SensorType::PANDORA_HESAI40:
      return "pandora_hesai40";
    case SensorType::PANDORA_CAMERA:
      return "pandora_camera";
    case SensorType::FUJITSU_ESR:
      return "fujitsu_esr";
    case SensorType::BOSCH_MRR:
      return "bosch_mrr";
    case SensorType::FUSION:
      return "fusion";
    case SensorType::IBEO:
      return "ibeo";
    case SensorType::DELPHI_ESR:
      return "delphi_esr";
    case SensorType::DELPHI_SRR:
      return "delphi_srr";
    case SensorType::UNKNOWN_SENSOR_TYPE:
      return "unknown_sensor_type";
    case SensorType::LeiShen_lidar_fl:
      return "LeiShen_lidar_fl";
    case SensorType::LeiShen_lidar_fr:
      return "LeiShen_lidar_fr";
    case SensorType::LeiShen_lidar_rl:
      return "LeiShen_lidar_rl";
    case SensorType::LeiShen_lidar_rr:
      return "LeiShen_lidar_rr";
    case SensorType::Conti_MRR:
      return "Conti_MRR";
    case SensorType::SutengJuChuang:
      return "SutengJuChuang";
  }
  return "";
}

bool is_lidar(SensorType sensor_type) {
  return (sensor_type == SensorType::LeiShen_lidar_fl||
          sensor_type == SensorType::LeiShen_lidar_fr||
          sensor_type == SensorType::LeiShen_lidar_rl||
          sensor_type == SensorType::LeiShen_lidar_rr||
          sensor_type == SensorType::VELODYNE_64 ||
          sensor_type == SensorType::VELODYNE_16 ||
          sensor_type == SensorType::PANDORA_HESAI40 ||
          sensor_type == SensorType::SutengJuChuang||
          sensor_type == SensorType::IBEO);
}

bool is_radar(SensorType sensor_type) {
  return (sensor_type == SensorType::Conti_MRR||
          sensor_type == SensorType::RADAR ||
          sensor_type == SensorType::FUJITSU_ESR ||
          sensor_type == SensorType::BOSCH_MRR ||
          sensor_type == SensorType::DELPHI_ESR ||
          sensor_type == SensorType::DELPHI_SRR);
}

bool is_camera(SensorType sensor_type) {
  return (sensor_type == SensorType::CAMERA ||
          sensor_type == SensorType::PANDORA_CAMERA);
}

}  // namespace glb_auto
