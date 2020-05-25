/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *****************************************************************************/

#ifndef _GLB_AUTO_SENSORFUSION_COMMON_TYPES_H_
#define _GLB_AUTO_SENSORFUSION_COMMON_TYPES_H_

#include <string>

//#include "common/pcl_types.h"

namespace glb_auto_perception_sensorfusion {

enum class ObjectType {
  UNKNOWN = 0,
  UNKNOWN_MOVABLE = 1,
  UNKNOWN_UNMOVABLE = 2,
  PEDESTRIAN = 3,
  BICYCLE = 4,
  VEHICLE = 5,
  MAX_OBJECT_TYPE = 6,
};

enum InternalObjectType {
  INT_BACKGROUND = 0,
  INT_SMALLMOT = 1,
  INT_PEDESTRIAN = 2,
  INT_NONMOT = 3,
  INT_BIGMOT = 4,
  INT_UNKNOWN = 5,
  INT_MAX_OBJECT_TYPE = 6,
};

enum class SensorType {
  VELODYNE_64 = 0,
  VELODYNE_16 = 1,
  RADAR = 2,
  CAMERA = 3,
  ULTRASONIC = 4,
  PANDORA_HESAI40 = 5,
  PANDORA_CAMERA = 6,
  FUJITSU_ESR = 7,
  BOSCH_MRR = 8,
  FUSION = 9,
  IBEO = 10,
  DELPHI_ESR = 11,
  DELPHI_SRR = 12,
  Conti_MRR = 13 ,
  LeiShen_lidar_fr=14,
  LeiShen_lidar_fl=15,
  LeiShen_lidar_rr=16,
  LeiShen_lidar_rl=17,
  SutengJuChuang=18,
  UNKNOWN_SENSOR_TYPE,
};

//typedef glb_auto_perception_sensorfusion::pcl_util::PointCloud PolygonType;
//typedef glb_auto_perception_sensorfusion::pcl_util::PointDCloud PolygonDType;

using SeqId = uint32_t;

std::string GetObjectName(const ObjectType& obj_type);

std::string GetSensorType(SensorType sensor_type);

bool is_lidar(SensorType sensor_type);
bool is_radar(SensorType sensor_type);
bool is_camera(SensorType sensor_type);

}  // namespace perception

#endif  // MODULES_PERCEPTION_OBSTACLE_BASE_TYPES_H_
