/******************************************************************************
 * Copyright 2017 The glb_auto Authors. All Rights Reserved.
 *****************************************************************************/

#ifndef _GLB_AUTO_SENSORFUSION_FUSION_PROBABILISTIC_FUSION_PBF_BASE_MOTION_FUSION_H_  // NOLINT
#define _GLB_AUTO_SENSORFUSION_FUSION_PROBABILISTIC_FUSION_PBF_BASE_MOTION_FUSION_H_  // NOLINT

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "fusion/probabilistic_fusion/pbf_sensor_object.h"

namespace glb_auto_perception_sensorfusion {

class PbfBaseMotionFusion {
 public:
  PbfBaseMotionFusion() : name_("PbfBaseMotionFusion") {}
  virtual ~PbfBaseMotionFusion() = default;

  // @brief initialize the state of filter
  // @params[IN] anchor_point: initial anchor point for filtering
  // @params[IN] velocity: initial velocity for filtering
  // @return nothing
  virtual void Initialize(const Eigen::Vector3d& anchor_point,
                          const Eigen::Vector3d& velocity) = 0;

  // @brief initialize the state of the filter
  // @params[IN] new_object: initial object for filtering
  // @return nothing
  virtual void Initialize(
      const std::shared_ptr<PbfSensorObject> new_object) = 0;

  // @brief predict the state of filter
  // @params[OUT] anchor_point:  predicted anchor point for filtering
  // @params[OUT] velocity: predicted velocity
  // @params[IN] time_diff: time interval from last update
  // @return nothing
  virtual void Predict(Eigen::Vector3d* anchor_point, Eigen::Vector3d* velocity,
                       const double time_diff) = 0;

  // @brief update with measurements
  // @params[IN] new_object: new object for current update
  // @params[IN] time_diff: time interval from last update;
  // @return nothing
  virtual void UpdateWithObject(
      const std::shared_ptr<PbfSensorObject> new_object,
      const double time_diff) = 0;

  // @brief update without measurements
  // @params[IN] time_diff: time interval from last update
  // @return nothing
  virtual void UpdateWithoutObject(const double time_diff) = 0;

  // @brief get current state of the filter
  // @params[OUT] anchor_point: current anchor_point
  // @params[OUT] velocity: current velocity
  // @return nothing
  virtual void GetState(Eigen::Vector3d* anchor_point,
                        Eigen::Vector3d* velocity) = 0;

  // @brief set current state of the filter
  // @params[IN] anchor_point: updated anchor_point
  // @params[IN] velocity: updated velocity
  // @return nothing
  virtual void SetState(const Eigen::Vector3d& anchor_point,
                        const Eigen::Vector3d& velocity) = 0;

  void setCurrentFuseTS(const double ts) { fuse_timestamp = ts; }

  void setLastFuseTS(const double ts) { last_fuse_timestamp = ts; }

  double getLastFuseTS() { return last_fuse_timestamp; }

  double getCurrentFuseTS() { return fuse_timestamp; }

  double getFuseTimeDiff() { return (fuse_timestamp - last_fuse_timestamp); }

  std::string name() { return name_; }

  // @brief check if filter has been initialized
  // @return initialization status
  bool Initialized() const { return initialized_; }

 protected:
  std::string name_;
  bool initialized_ = false;

  // last fusion time
  double last_fuse_timestamp = 0.0;
  double fuse_timestamp = 0.0;
};

}  // namespace glb_auto

/* clang-format off */
#endif  // MODULES_PERCEPTION_OBSTACLE_FUSION_PROBABILISTIC_FUSION_PBF_BASE_MOTION_FUSION_H_ // NOLINT
/* clang-format on */
