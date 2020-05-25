/******************************************************************************
 * Copyright 2017 The glb_auto Authors. All Rights Reserved.
 *****************************************************************************/

#include "config_manager/calibration_config_manager.h"

#include <cmath>

#include "Eigen/Eigen"
#include "gflags/gflags.h"
#include "yaml-cpp/yaml.h"

#include "util/log.h"

namespace glb_auto_perception_sensorfusion {

// @brief load transformation_matrix from file
bool load_transformation_matrix_from_file(
    const std::string &file_name, Eigen::Matrix4d *transformation_matrix, int cam_idx) {
  try {
    YAML::Node config = YAML::LoadFile(file_name);
    std::string cam_idx_str = std::to_string(cam_idx);
    if (!config) {
      AWARN << "Open TransformationMatrix File:" << file_name << " failed.";
      return false;
    }
    if (!config[cam_idx_str]["transform"]) {
      AWARN << "Open TransformationMatrix File:" << file_name
            << " has no transform.";
      return false;
    }
    // fill translation
    if (config[cam_idx_str]["transform"]["translation"]) {
      (*transformation_matrix)(0, 3) =
          config[cam_idx_str]["transform"]["translation"]["x"].as<double>();
      (*transformation_matrix)(1, 3) =
          config[cam_idx_str]["transform"]["translation"]["y"].as<double>();
      (*transformation_matrix)(2, 3) =
          config[cam_idx_str]["transform"]["translation"]["z"].as<double>();
    } else {
      AWARN << "TransformationMatrix File:" << file_name
            << " has no transform:translation.";
      return false;
    }
    // fill rotation
    if (config[cam_idx_str]["transform"]["rotation"]) {
      double qx = config[cam_idx_str]["transform"]["rotation"]["x"].as<double>();
      double qy = config[cam_idx_str]["transform"]["rotation"]["y"].as<double>();
      double qz = config[cam_idx_str]["transform"]["rotation"]["z"].as<double>();
      double qw = config[cam_idx_str]["transform"]["rotation"]["w"].as<double>();
      Eigen::Quaternion<double> rotation(qw, qx, qy, qz);
      (*transformation_matrix).block<3, 3>(0, 0) = rotation.toRotationMatrix();
    } else {
      AWARN << "TransformationMatrix File:" << file_name
            << " has no transform:rotation.";
      return false;
    }
  } catch (const YAML::Exception &e) {
    AERROR << file_name << " load failed. error:" << e.what();
    AERROR << "Please ensure param file is exist or format is correct";
    return false;
  }

  // fill trivial elements
  for (int i = 0; i < 3; i++) {
    (*transformation_matrix)(3, i) = 0.0;
  }
  (*transformation_matrix)(3, 3) = 1.0;
  return true;
}

bool load_matrix4d_from_file(const std::string &file_name,
                             const std::string &key, Eigen::Matrix4d *matrix,
                             int cam_idx) {
  try {
    YAML::Node config = YAML::LoadFile(file_name);
    if (!config) {
      AWARN << "Open Matrix File:" << file_name << " failed.";
      return false;
    }
    if (!config[key]) {
      AWARN << "Matrix File:" << file_name << " has no key:" << key;
      return false;
    }

    (*matrix)(0, 0) = config[key]["0"]["0"].as<double>();
    (*matrix)(0, 1) = config[key]["0"]["1"].as<double>();
    (*matrix)(0, 2) = config[key]["0"]["2"].as<double>();
    (*matrix)(0, 3) = config[key]["0"]["3"].as<double>();
    (*matrix)(1, 0) = config[key]["1"]["0"].as<double>();
    (*matrix)(1, 1) = config[key]["1"]["1"].as<double>();
    (*matrix)(1, 2) = config[key]["1"]["2"].as<double>();
    (*matrix)(1, 3) = config[key]["1"]["3"].as<double>();
    (*matrix)(2, 0) = config[key]["2"]["0"].as<double>();
    (*matrix)(2, 1) = config[key]["2"]["1"].as<double>();
    (*matrix)(2, 2) = config[key]["2"]["2"].as<double>();
    (*matrix)(2, 3) = config[key]["2"]["3"].as<double>();
    (*matrix)(3, 0) = config[key]["3"]["0"].as<double>();
    (*matrix)(3, 1) = config[key]["3"]["1"].as<double>();
    (*matrix)(3, 2) = config[key]["3"]["2"].as<double>();
    (*matrix)(3, 3) = config[key]["3"]["3"].as<double>();
  } catch (const YAML::Exception &e) {
    AERROR << file_name << " load failed. error:" << e.what();
    AERROR << "Please ensure param file is exist or format is correct";
    return false;
  }

  return true;
}

bool CameraCoeffient::init(const std::string &camera_type,
                           const std::string &camera_extrinsic_file_name,
                           const std::string &camera_intrinsic_file_name,
                           int cam_idx) {
  camera_type_str = camera_type;

  try {
    if (!init_camera_extrinsic_matrix(camera_extrinsic_file_name, cam_idx)) {
      AERROR << camera_type_str << " init failed. lidar to camera matrix file:"
             << camera_extrinsic_file_name;
      return false;
    }

    if (!init_camera_intrinsic_matrix_and_distort_params(
            camera_intrinsic_file_name, cam_idx)) {
      AERROR << camera_type_str << " init failed. camera intrinsic matrix file:"
             << camera_intrinsic_file_name;
      return false;
    }
  } catch (const YAML::Exception &e) {
    AERROR << camera_type_str << " init failed. error:" << e.what();
    AERROR << "Please ensure param file is exist or format is correct";
    return false;
  }
  return true;
}

bool CameraCoeffient::init_camera_extrinsic_matrix(
    const std::string &file_name, int cam_idx) {
  if (!load_transformation_matrix_from_file(file_name, &camera_extrinsic, cam_idx) &&
      !load_matrix4d_from_file(file_name, "T", &camera_extrinsic, cam_idx)) {
    AERROR << "Load camera_extrinsic matrix file failed. file:" << file_name;
    return false;
  }
  AINFO << camera_type_str
        << " camera_extrinsic matrix is:" << camera_extrinsic;
  return true;
}

bool CameraCoeffient::init_camera_intrinsic_matrix_and_distort_params(
    const std::string &file_name, int cam_idx) {
  YAML::Node config = YAML::LoadFile(file_name);
  std::string cam_idx_str = std::to_string(cam_idx);

  if (!config["resize_factor"]) {
    AERROR << "load resize factor failed. file_name:" << file_name;
    return false;
  }
  double resize_factor = config["resize_factor"].as<double>();

  if (config[cam_idx_str]["K"]) {
    // TODO: make resize_factor a variable
    camera_intrinsic(0, 0) = config[cam_idx_str]["K"][0].as<double>() 
                              * resize_factor;
    camera_intrinsic(0, 1) = config[cam_idx_str]["K"][1].as<double>() 
                              * resize_factor;
    camera_intrinsic(0, 2) = config[cam_idx_str]["K"][2].as<double>() 
                              * resize_factor;
    camera_intrinsic(0, 3) = 0.0;
    camera_intrinsic(1, 0) = config[cam_idx_str]["K"][3].as<double>() 
                              * resize_factor;
    camera_intrinsic(1, 1) = config[cam_idx_str]["K"][4].as<double>() 
                              * resize_factor;
    camera_intrinsic(1, 2) = config[cam_idx_str]["K"][5].as<double>() 
                              * resize_factor;
    camera_intrinsic(1, 3) = 0.0;
    camera_intrinsic(2, 0) = config[cam_idx_str]["K"][6].as<double>();
    camera_intrinsic(2, 1) = config[cam_idx_str]["K"][7].as<double>();
    camera_intrinsic(2, 2) = config[cam_idx_str]["K"][8].as<double>();
    camera_intrinsic(2, 3) = 0.0;
  } else {
    AERROR << "load lidar camera intrinsic failed. file_name:" << file_name;
    return false;
  }

  if (config[cam_idx_str]["D"]) {
    for (size_t i = 0; i < 5; ++i) {
      distort_params[i] = config[cam_idx_str]["D"][i].as<double>();
    }
  } else {
    AERROR << "load camera distortion coefficients failed. file_name:"
           << file_name;
    return false;
  }

  if (config[cam_idx_str]["height"]) {
    image_height = config[cam_idx_str]["height"].as<size_t>();
  } else {
    AERROR << "load image height from camera intrinsic failed. file:"
           << file_name;
    return false;
  }
  if (config[cam_idx_str]["width"]) {
    image_width = config[cam_idx_str]["width"].as<size_t>();
  } else {
    AERROR << "Load image width from camera intrinsic failed. file:"
           << file_name;
    return false;
  }

  AINFO << camera_type_str << " camera intrinsic is:" << camera_intrinsic
        << " distort_params is:" << distort_params << " height:" << image_height
        << " width:" << image_width;
  return true;
}

CalibrationConfigManager::CalibrationConfigManager() {
    // : camera_calibration_(new CameraCalibration()) {
  camera_calibration_.resize(2);
  camera_extrinsic_path_ = FLAGS_camera_extrinsics_file;
  camera_intrinsic_path_ = FLAGS_camera_intrinsics_file;
  init();
}

bool CalibrationConfigManager::init() {
  MutexLock lock(&mutex_);
  return init_internal();
}

bool CalibrationConfigManager::reset() {
  MutexLock lock(&mutex_);
  inited_ = false;
  return init_internal();
}

CalibrationConfigManager::~CalibrationConfigManager() {}

bool CalibrationConfigManager::init_internal() {
  if (inited_) {
    return true;
  }
  AINFO << "camera_calibration_ size: " << camera_calibration_.size();
  for (int i = 0; i < camera_calibration_.size(); i++) {
    AINFO << "loading camera_calibration_ of idx: " << i;
    camera_calibration_[i].reset(new CameraCalibration());
    if (!camera_calibration_[i]->init(camera_intrinsic_path_,
                                 camera_extrinsic_path_,
                                 i)) {
      AERROR << "init intrinsics failure: " << camera_intrinsic_path_ << " "
            << camera_extrinsic_path_;
      return false;
    }
  }

  AINFO << "finish to load Calibration Configs.";

  inited_ = true;
  return inited_;
}

CameraCalibration::CameraCalibration()
    : _camera2car_pose(new Eigen::Matrix<double, 4, 4>()),
      _car2camera_pose(new Eigen::Matrix<double, 4, 4>())
      //undistort_handler_(new ImageGpuPreprocessHandler()),
      //camera_model_(new CameraDistort<double>()) 
      {}

CameraCalibration::~CameraCalibration() {}

bool CameraCalibration::init(const std::string &intrinsic_path,
                             const std::string &extrinsic_path,
                             int cam_idx) {
  if (!camera_coefficient_.init("", extrinsic_path, intrinsic_path, cam_idx)) {
    AERROR << "init camera coefficient failed";
    return false;
  }

  camera_intrinsic_ = camera_coefficient_.camera_intrinsic;
  image_height_ = camera_coefficient_.image_height;
  image_width_ = camera_coefficient_.image_width;
  *_camera2car_pose = camera_coefficient_.camera_extrinsic;
  *_car2camera_pose = _camera2car_pose->inverse();

  // TODO(later): BUGGY. Crash
  // if (!init_undistortion(intrinsic_path)) {
  //   AERROR << "init undistortion failed";
  //   return false;
  // }

  //init_camera_model();
  calculate_homographic();
  AINFO << "Successfully loading intrinsic and extrinsic";
  return true;
}

void CameraCalibration::calculate_homographic() {
  std::lock_guard<std::mutex> lock(adj_mtx_);
  auto camera_intrinsic_inverse = camera_intrinsic_.block(0, 0, 3, 3).inverse();
  auto car2camera_3_4 = (*_car2camera_pose).block(0, 0, 3, 4);
  Eigen::Matrix3d camera_2car_stripped;
  camera_2car_stripped.col(0) = car2camera_3_4.col(0);
  camera_2car_stripped.col(1) = car2camera_3_4.col(1);
  camera_2car_stripped.col(2) = car2camera_3_4.col(3);
  homography_mat_ = camera_2car_stripped.inverse() * camera_intrinsic_inverse;
  homography_mat_inverse_ = homography_mat_.inverse();
}

/* void CameraCalibration::init_camera_model() {
  camera_model_->set(camera_intrinsic_.block(0, 0, 3, 3), image_width_,
                     image_height_);
} */

/* bool CameraCalibration::init_undistortion(const std::string &intrinsics_path) {
  AINFO << "Loading intrinsics: " << intrinsics_path;
  int err =
      undistort_handler_->init(intrinsics_path, 0);
      //undistort_handler_->init(intrinsics_path, FLAGS_obs_camera_detector_gpu);

  if (err != 0) {
    AERROR << "Undistortion initialization failed with error code: " << err;
    return false;
  }
  undistort_handler_->set_device(0);
  //undistort_handler_->set_device(FLAGS_obs_camera_detector_gpu);
  return true;
} */

}  // namespace glb_auto
