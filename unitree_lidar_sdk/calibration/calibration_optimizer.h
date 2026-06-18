// Copyright 2026 MapMindAI Inc. All rights reserved.

#ifndef UNITREE_LIDAR_SDK_CALIBRATION_OPTIMIZER_H_
#define UNITREE_LIDAR_SDK_CALIBRATION_OPTIMIZER_H_

#include <limits>
#include <string>
#include <vector>

#include "unitree_lidar_sdk/calibration/plane_extractor.h"
#include "unitree_lidar_sdk/calibration/replayer_common.h"

namespace calibration {

struct CalibrationParameters {
  std::vector<float> range_coefficients_m;
  std::vector<float> alpha_theta_coefficients_rad;
};

struct ResidualSummary {
  int total_points = 0;
  int assigned_points = 0;
  double mean_abs_error_m = std::numeric_limits<double>::infinity();
  double rms_error_m = std::numeric_limits<double>::infinity();
  double max_abs_error_m = 0.0;
  std::vector<int> assigned_per_plane;
};

struct CalibrationOptimizationConfig {
  bool enabled = true;
  std::string range_model_candidates = "constant,linear,quadratic";
  int iterations = 6;
  double range_step_m = 0.01;
  bool optimize_range_coefficients = true;
  double alpha_step_rad = 0.0005;
  bool optimize_alpha_theta_coefficients = true;
  double regularization = 1e-4;
  double assignment_threshold_m = 0.08;
  int min_required_assigned_points = 400;
};

struct CalibrationSolution {
  std::string model_name = "none";
  CalibrationParameters parameters;
  ResidualSummary residuals;
  double objective = std::numeric_limits<double>::infinity();
  UniLidarCalibration calibration;
};

UniLidarCalibration BuildCalibrationFromParameters(const CalibrationParameters& parameters);
ResidualSummary EvaluateResiduals(const ReplayFrame& frame, const std::vector<PlaneModel>& planes,
                                  double assignment_threshold_m);
CalibrationSolution OptimizeCalibration(const ReplayFrame& merged_frame,
                                        const std::vector<PlaneModel>& planes,
                                        const CalibrationOptimizationConfig& config);
std::string VectorSummary(const std::vector<float>& values, int max_items = 8);

}  // namespace calibration

#endif  // UNITREE_LIDAR_SDK_CALIBRATION_OPTIMIZER_H_
