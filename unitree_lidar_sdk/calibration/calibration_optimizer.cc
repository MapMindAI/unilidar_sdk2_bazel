// Copyright 2026 MapMindAI Inc. All rights reserved.

#include "unitree_lidar_sdk/calibration/calibration_optimizer.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#include <utility>

#include <glog/logging.h>

namespace calibration {
namespace {

struct RangeModelCandidate {
  std::string name;
  int degree = 0;
};

std::vector<std::string> SplitCsv(const std::string& input) {
  std::vector<std::string> tokens;
  std::stringstream stream(input);
  std::string token;
  while (std::getline(stream, token, ',')) {
    token.erase(std::remove_if(token.begin(), token.end(), ::isspace), token.end());
    if (!token.empty()) {
      tokens.push_back(token);
    }
  }
  return tokens;
}

std::vector<RangeModelCandidate> ParseRangeModelCandidates(const std::string& csv) {
  std::vector<RangeModelCandidate> models;
  for (const std::string& token : SplitCsv(csv)) {
    if (token == "constant") {
      models.push_back({"constant", 0});
    } else if (token == "linear") {
      models.push_back({"linear", 1});
    } else if (token == "quadratic") {
      models.push_back({"quadratic", 2});
    } else {
      LOG(WARNING) << "Ignoring unknown range model candidate: " << token;
    }
  }
  if (models.empty()) {
    models.push_back({"constant", 0});
  }
  return models;
}

float EvaluateRangeModel(const std::vector<float>& coefficients, float alpha) {
  float correction = 0.0f;
  float basis = 1.0f;
  for (float coefficient : coefficients) {
    correction += coefficient * basis;
    basis *= alpha;
  }
  return correction;
}

UniLidarCalibration MakeCalibrationFromRawParameters(
    const std::vector<float>& range_coefficients,
    const std::vector<float>& alpha_theta_coefficients) {
  UniLidarCalibration calibration;
  calibration.enabled = true;
  calibration.delta_range_alpha_fcn =
      [range_coefficients](float alpha) { return EvaluateRangeModel(range_coefficients, alpha); };
  calibration.delta_alpha_theta_fcn =
      [alpha_theta_coefficients](float theta) { return EvaluateRangeModel(alpha_theta_coefficients, theta); };
  return calibration;
}

double RegularizationPenalty(const std::vector<float>& range_coefficients,
                             const std::vector<float>& alpha_theta_coefficients,
                             double regularization) {
  double penalty = 0.0;
  for (float coefficient : range_coefficients) {
    penalty += coefficient * coefficient;
  }
  for (float offset : alpha_theta_coefficients) {
    penalty += offset * offset;
  }
  return regularization * penalty;
}

}  // namespace

UniLidarCalibration BuildCalibrationFromParameters(const CalibrationParameters& parameters) {
  return MakeCalibrationFromRawParameters(parameters.range_coefficients_m,
                                          parameters.alpha_theta_coefficients_rad);
}

ResidualSummary EvaluateResiduals(const ReplayFrame& frame, const std::vector<PlaneModel>& planes,
                                  double assignment_threshold_m) {
  ResidualSummary summary;
  summary.total_points = static_cast<int>(frame.points.size());
  summary.assigned_per_plane.assign(planes.size(), 0);
  if (planes.empty() || frame.points.empty()) {
    return summary;
  }

  double sum_abs = 0.0;
  double sum_sq = 0.0;
  for (const CloudPoint& point : frame.points) {
    double best = assignment_threshold_m;
    int best_plane = -1;
    for (size_t plane_index = 0; plane_index < planes.size(); ++plane_index) {
      const double residual =
          std::abs(planes[plane_index].normal.dot(point.xyz) + planes[plane_index].d);
      if (residual < best) {
        best = residual;
        best_plane = static_cast<int>(plane_index);
      }
    }
    if (best_plane < 0) {
      continue;
    }
    ++summary.assigned_points;
    ++summary.assigned_per_plane[best_plane];
    sum_abs += best;
    sum_sq += best * best;
    summary.max_abs_error_m = std::max(summary.max_abs_error_m, best);
  }

  if (summary.assigned_points > 0) {
    summary.mean_abs_error_m = sum_abs / static_cast<double>(summary.assigned_points);
    summary.rms_error_m = std::sqrt(sum_sq / static_cast<double>(summary.assigned_points));
  }
  return summary;
}

CalibrationSolution OptimizeCalibration(const ReplayFrame& merged_frame,
                                        const std::vector<PlaneModel>& planes,
                                        const CalibrationOptimizationConfig& config) {
  CalibrationSolution best_solution;
  ReplayFrame candidate_frame = merged_frame;
  const std::vector<RangeModelCandidate> models =
      ParseRangeModelCandidates(config.range_model_candidates);

  for (const RangeModelCandidate& model : models) {
    std::vector<float> range_coefficients(static_cast<size_t>(model.degree + 1), 0.0f);
    std::vector<float> alpha_theta_coefficients = {0.0f, 0.0f};
    double range_step = config.range_step_m;
    double alpha_step = config.alpha_step_rad;

    auto evaluate_current = [&](ResidualSummary* residuals_out) {
      const UniLidarCalibration calibration = MakeCalibrationFromRawParameters(
          range_coefficients, alpha_theta_coefficients);
      RebuildFramePoints(calibration, &candidate_frame);
      ResidualSummary residuals =
          EvaluateResiduals(candidate_frame, planes, config.assignment_threshold_m);
      if (residuals_out != nullptr) {
        *residuals_out = residuals;
      }
      if (residuals.assigned_points < config.min_required_assigned_points) {
        return std::numeric_limits<double>::infinity();
      }
      return residuals.rms_error_m +
             RegularizationPenalty(range_coefficients, alpha_theta_coefficients, config.regularization);
    };

    ResidualSummary current_residuals;
    double current_objective = evaluate_current(&current_residuals);
    LOG(INFO) << "Calibration model=" << model.name
              << " initial_rms=" << current_residuals.rms_error_m
              << " assigned=" << current_residuals.assigned_points;

    for (int iteration = 0; iteration < config.iterations; ++iteration) {
      bool improved = false;

      if (config.optimize_range_coefficients) {
        for (size_t coefficient_index = 0; coefficient_index < range_coefficients.size();
             ++coefficient_index) {
          const float original = range_coefficients[coefficient_index];
          for (const float delta :
               {static_cast<float>(-range_step), static_cast<float>(range_step)}) {
            range_coefficients[coefficient_index] = original + delta;
            ResidualSummary trial_residuals;
            const double trial_objective = evaluate_current(&trial_residuals);
            if (trial_objective < current_objective) {
              current_objective = trial_objective;
              current_residuals = trial_residuals;
              improved = true;
            } else {
              range_coefficients[coefficient_index] = original;
            }
          }
        }
      }

      if (config.optimize_alpha_theta_coefficients) {
        for (size_t coefficient_index = 0; coefficient_index < alpha_theta_coefficients.size();
             ++coefficient_index) {
          const float original = alpha_theta_coefficients[coefficient_index];
          for (const float delta :
               {static_cast<float>(-alpha_step), static_cast<float>(alpha_step)}) {
            alpha_theta_coefficients[coefficient_index] = original + delta;
            ResidualSummary trial_residuals;
            const double trial_objective = evaluate_current(&trial_residuals);
            if (trial_objective < current_objective) {
              current_objective = trial_objective;
              current_residuals = trial_residuals;
              improved = true;
            } else {
              alpha_theta_coefficients[coefficient_index] = original;
            }
          }
        }
      }

      LOG(INFO) << "Calibration model=" << model.name << " iter=" << iteration
                << " objective=" << current_objective
                << " rms=" << current_residuals.rms_error_m
                << " assigned=" << current_residuals.assigned_points;
      if (!improved) {
        range_step *= 0.5;
        alpha_step *= 0.5;
      }
    }

    CalibrationSolution candidate_solution;
    candidate_solution.model_name = model.name;
    candidate_solution.parameters.range_coefficients_m = range_coefficients;
    candidate_solution.parameters.alpha_theta_coefficients_rad = alpha_theta_coefficients;
    candidate_solution.objective = current_objective;
    candidate_solution.residuals = current_residuals;
    candidate_solution.calibration = BuildCalibrationFromParameters(candidate_solution.parameters);

    if (candidate_solution.objective < best_solution.objective) {
      best_solution = std::move(candidate_solution);
    }
  }

  return best_solution;
}

std::string VectorSummary(const std::vector<float>& values, int max_items) {
  std::ostringstream stream;
  stream << "[";
  for (size_t i = 0; i < values.size() && static_cast<int>(i) < max_items; ++i) {
    if (i > 0) {
      stream << ", ";
    }
    stream << std::fixed << std::setprecision(6) << values[i];
  }
  if (static_cast<int>(values.size()) > max_items) {
    stream << ", ...";
  }
  stream << "]";
  return stream.str();
}

}  // namespace calibration
