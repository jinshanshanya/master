/******************************************************************************
 * Copyright 2017 The glb_auto Authors. All Rights Reserved.
 *****************************************************************************/

#ifndef _GLB_AUTO_SENSORFUSION_FUSION_ASYNC_FUSION_ASYNC_FUSION_H_
#define _GLB_AUTO_SENSORFUSION_FUSION_ASYNC_FUSION_ASYNC_FUSION_H_

#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "fusion/proto/async_fusion_config.pb.h"

#include "common/object.h"
#include "fusion/interface/base_fusion.h"
#include "fusion/probabilistic_fusion/pbf_base_track_object_matcher.h"
#include "fusion/probabilistic_fusion/pbf_hm_track_object_matcher.h"
#include "fusion/probabilistic_fusion/pbf_track_manager.h"

namespace glb_auto_perception_sensorfusion {

class AsyncFusion : public BaseFusion {
 public:
  AsyncFusion() = default;
  ~AsyncFusion();

  virtual bool Init();

  bool Init(bool log_fusion);

  bool Init(bool log_fusion, std::string &file_path_config);

  // @brief: fuse objects from multi sensors(64-lidar, 16-lidar, radar...)
  // @param [in]: multi sensor objects.
  // @param [out]: fused objects.
  // @return true if fuse successfully, otherwise return false
  virtual bool Fuse(const std::vector<SensorObjects> &multi_sensor_objects,
                    std::vector<std::shared_ptr<Object>> *fused_objects,
                    FusionOptions *options);

  virtual std::string name() const;

 protected:
  void FuseFrame(PbfSensorFramePtr frame);

  /**@brief create new tracks for objects not assigned to current tracks*/
  void CreateNewTracks(
      const std::vector<std::shared_ptr<PbfSensorObject>> &sensor_objects,
      const std::vector<int> &unassigned_ids, 
      const SensorType &sensor_type);

  /**@brief update current tracks with matched objects*/
  void UpdateAssignedTracks(
      const std::vector<std::shared_ptr<PbfSensorObject>> &sensor_objects,
      const std::vector<std::pair<int, int>> &assignments,
      const std::vector<double> &track_objects_dist,
      std::vector<PbfTrackPtr> const *tracks);

  /**@brief update current tracks which cannot find matched objects*/
  void UpdateUnassignedTracks(const std::vector<int> &unassigned_tracks,
                              const std::vector<double> &track_object_dist,
                              const SensorType &sensor_type,
                              const std::string &sensor_id,
                              const double timestamp,
                              std::vector<PbfTrackPtr> *tracks);

  void CollectFusedObjects(double timestamp,
                           std::vector<std::shared_ptr<Object>> *fused_objects);

  void DecomposeFrameObjects(
      const std::vector<std::shared_ptr<PbfSensorObject>> &frame_objects,
      std::vector<std::shared_ptr<PbfSensorObject>> *foreground_objects,
      std::vector<std::shared_ptr<PbfSensorObject>> *background_objects);

  void FuseForegroundObjects(
      const Eigen::Vector3d &ref_point, const SensorType &sensor_type,
      const std::string &sensor_id, const double timestamp,
      std::vector<std::shared_ptr<PbfSensorObject>> *foreground_objects);

  PbfSensorFramePtr ConstructFrame(const SensorObjects &obj);

 protected:
  bool log_fusion_ = false;
  
  bool started_ = false;
  std::unique_ptr<PbfBaseTrackObjectMatcher> matcher_;
  PbfTrackManager *track_manager_ = nullptr;
  std::mutex fusion_mutex_;

  async_fusion_config::ModelConfigs config_;

};

}  // namespace glb_auto

#endif  // MODULES_PERCEPTION_OBSTACLE_FUSION_ASYNC_FUSION_ASYNC_FUSION_H_
