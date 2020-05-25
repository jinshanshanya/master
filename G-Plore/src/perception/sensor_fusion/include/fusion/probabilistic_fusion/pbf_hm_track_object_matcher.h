/******************************************************************************
 * Copyright 2017 The glb_auto Authors. All Rights Reserved.
 *****************************************************************************/

#ifndef _GLB_AUTO_SENSORFUSION_FUSION_PBF_PBF_HM_TRACK_OBJECT_MATCHER_H_
#define _GLB_AUTO_SENSORFUSION_FUSION_PBF_PBF_HM_TRACK_OBJECT_MATCHER_H_

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "util/graph_util.h"
#include "util/hungarian_bigraph_matcher.h"
#include "fusion/probabilistic_fusion/pbf_base_track_object_matcher.h"
#include "fusion/probabilistic_fusion/pbf_sensor_object.h"
#include "fusion/probabilistic_fusion/pbf_track.h"

namespace glb_auto_perception_sensorfusion {

class PbfHmTrackObjectMatcher : public PbfBaseTrackObjectMatcher {
 public:
  PbfHmTrackObjectMatcher() = default;
  virtual ~PbfHmTrackObjectMatcher() = default;

  bool Match(
      const std::vector<PbfTrackPtr> &fusion_tracks,
      const std::vector<std::shared_ptr<PbfSensorObject>> &sensor_objects,
      const TrackObjectMatcherOptions &options,
      std::vector<std::pair<int, int>> *assignments,
      std::vector<int> *unassigned_fusion_tracks,
      std::vector<int> *unassigned_sensor_tracks,
      std::vector<double> *track2measurements_dist,
      std::vector<double> *measurement2track_dist) override;

  bool Init() override;

  std::string name() const override;

 protected:
  void ComputeAssociationMat(
      const std::vector<PbfTrackPtr> &fusion_tracks,
      const std::vector<std::shared_ptr<PbfSensorObject>> &sensor_objects,
      const std::vector<int> &unassigned_fusion_tracks,
      const std::vector<int> &unassigned_sensor_objects,
      const Eigen::Vector3d &ref_point,
      const Eigen::Matrix4d &sensor_world_pose,
      std::vector<std::vector<double>> *association_mat);
  bool HmAssign(const std::vector<std::vector<double>> &association_mat,
                std::vector<std::pair<int, int>> *assignments,
                std::vector<int> *unassigned_fusion_tracks,
                std::vector<int> *unassigned_sensor_objects);
  void MinimizeAssignment(
      const std::vector<std::vector<double>> &association_mat,
      std::vector<int> *ref_idx, std::vector<int> *new_idx);
  void ComputeConnectedComponents(
      const std::vector<std::vector<double>> &association_mat,
      const float connected_threshold,
      std::vector<std::vector<int>> *track_components,
      std::vector<std::vector<int>> *obj_components);

};

}  // namespace glb_auto

#endif  // MODULES_PERCEPTION_OBSTACLE_FUSION_PBF_PBF_HM_TRACK_OBJECT_MATCHER_H_
