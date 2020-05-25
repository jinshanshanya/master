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

#ifndef _GLB_AUTO_SENSORFUSION_FUSION_PROBABILISTIC_FUSION_PBF_SENSOR_H_
#define _GLB_AUTO_SENSORFUSION_FUSION_PROBABILISTIC_FUSION_PBF_SENSOR_H_

#include <deque>
#include <string>
#include <vector>

#include "util/log.h"
#include "common/object.h"
#include "fusion/probabilistic_fusion/pbf_sensor_object.h"

namespace glb_auto_perception_sensorfusion {

class PbfSensor {
 public:
  explicit PbfSensor(const std::string &sensor_id, const SensorType &type);
  ~PbfSensor();

  /*
   * @brief query frames whose time stamp is in range (latest_fused_time_stamp_,
   * time_stamp); update latest_query_timestamp_ by time_stamp.
   */
  void QueryLatestFrames(double time_stamp,
                         std::vector<PbfSensorFramePtr> *frames);

  /*
   * @brief query latest frame whose time stamp is in range
   * (latest_fused_time_stamp_, time_stamp]; update latest_query_timestamp_ by
   * time_stamp
   */
  PbfSensorFramePtr QueryLatestFrame(const double time_stamp);

  /**@brief add a frame objects*/
  void AddFrame(const SensorObjects &frame);

  /**@brief query pose at time_stamp, return false if not found*/
  bool GetPose(const double time_stamp, const double time_range,
               Eigen::Matrix4d *pose);

  static void SetMaxCachedFrameNumber(const int number) {
    s_max_cached_frame_number_ = number;
  }

 protected:
  /**@brief cached frames in FIFO*/
  std::deque<PbfSensorFramePtr> frames_;

  std::string sensor_id_;
  SensorType sensor_type_;

  /**@brief max size of frames_*/
  static size_t s_max_cached_frame_number_;

  double latest_query_timestamp_ = 0.0;

 private:
  PbfSensor();
};

}  // namespace glb_auto

#endif  // MODULES_PERCEPTION_OBSTACLE_FUSION_PROBABILISTIC_FUSION_PBF_SENSOR_H_
