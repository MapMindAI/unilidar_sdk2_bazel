// Copyright 2026 MapMindAI Inc. All rights reserved.

#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

#include <glog/logging.h>

#include "gflags/gflags.h"
#include "unitree_lidar_sdk/calibration/calibration_optimizer.h"
#include "unitree_lidar_sdk/calibration/plane_extractor.h"
#include "unitree_lidar_sdk/calibration/replayer_common.h"
#include "unitree_lidar_sdk/calibration/replayer_viewer.h"

DEFINE_string(input_path, "/tmp/unitree_lidar_packets.bin", "Input raw packet recording file.");
DEFINE_int32(accumulate_rings, 50, "Number of packets to accumulate into one displayed cloud.");
DEFINE_int32(window_width, 1600, "Viewer width.");
DEFINE_int32(window_height, 900, "Viewer height.");
DEFINE_double(play_hz, 5.0, "Autoplay rate in clouds per second.");
DEFINE_double(point_size, 2.0, "Point size in the viewer.");
DEFINE_double(merged_point_size, 1.0, "Point size for the merged initial-frame cloud.");
DEFINE_double(min_range_m, 0.0, "Additional minimum range filter in meters.");
DEFINE_double(max_range_m, 100.0, "Additional maximum range filter in meters.");
DEFINE_int32(merge_beginning_frames, 10,
             "Merge the first N replay frames into one static point cloud and overlay it.");
DEFINE_bool(orthographic_camera, true, "Use an orthographic camera in the Pangolin viewer.");
DEFINE_double(orthographic_extent, 50.0,
              "Half extent of the orthographic camera frustum in viewer units.");

DEFINE_bool(extract_planes, true, "Extract planes from the merged beginning cloud.");
DEFINE_int32(max_planes, 3, "Maximum number of planes to extract from the merged cloud.");
DEFINE_double(plane_inlier_threshold_m, 0.2,
              "Point-to-plane distance threshold used for plane extraction and assignment.");
DEFINE_int32(plane_ransac_iterations, 600, "RANSAC iterations per extracted plane.");
DEFINE_int32(plane_min_inliers, 400, "Minimum inlier count required to accept an extracted plane.");
DEFINE_int32(plane_detection_sample_limit, 25000,
             "Maximum number of merged points used for plane detection.");
DEFINE_double(plane_min_extent_m, 0.75, "Minimum plane span along each in-plane axis.");

DEFINE_bool(show_planes, true, "Overlay extracted plane rectangles.");
DEFINE_bool(show_plane_inliers, true, "Overlay merged-cloud plane inlier points.");

DEFINE_bool(optimize_calibration, true, "Optimize calibration against extracted planes.");
DEFINE_string(range_model_candidates, "constant,linear,quadratic",
              "Comma-separated range correction models: constant, linear, quadratic.");
DEFINE_int32(calibration_iterations, 6, "Coordinate-descent iterations per model family.");
DEFINE_double(calibration_range_step_m, 0.01, "Initial step size for range correction coefficients.");
DEFINE_bool(optimize_range_coefficients, true,
            "Optimize delta_range_alpha_fcn coefficients during calibration search.");
DEFINE_double(calibration_alpha_step_rad, 0.0005,
              "Initial step size for each per-ring alpha offset.");
DEFINE_bool(optimize_ring_alpha_offsets, true,
            "Also optimize per-ring alpha offsets in delta_alpha_min.");
DEFINE_double(calibration_regularization, 1e-4,
              "L2 regularization applied to the optimized parameters.");
DEFINE_double(calibration_assignment_threshold_m, 0.08,
              "Maximum point-to-plane distance counted in the calibration objective.");

namespace third_party {
namespace {

calibration::PlaneExtractionConfig MakePlaneExtractionConfig() {
  calibration::PlaneExtractionConfig config;
  config.enabled = FLAGS_extract_planes;
  config.max_planes = FLAGS_max_planes;
  config.inlier_threshold_m = FLAGS_plane_inlier_threshold_m;
  config.ransac_iterations = FLAGS_plane_ransac_iterations;
  config.min_inliers = FLAGS_plane_min_inliers;
  config.detection_sample_limit = FLAGS_plane_detection_sample_limit;
  config.min_extent_m = FLAGS_plane_min_extent_m;
  return config;
}

calibration::CalibrationOptimizationConfig MakeCalibrationOptimizationConfig(
    int min_required_assigned_points) {
  calibration::CalibrationOptimizationConfig config;
  config.enabled = FLAGS_optimize_calibration;
  config.range_model_candidates = FLAGS_range_model_candidates;
  config.iterations = FLAGS_calibration_iterations;
  config.range_step_m = FLAGS_calibration_range_step_m;
  config.optimize_range_coefficients = FLAGS_optimize_range_coefficients;
  config.alpha_step_rad = FLAGS_calibration_alpha_step_rad;
  config.optimize_ring_alpha_offsets = FLAGS_optimize_ring_alpha_offsets;
  config.regularization = FLAGS_calibration_regularization;
  config.assignment_threshold_m = FLAGS_calibration_assignment_threshold_m;
  config.min_required_assigned_points = min_required_assigned_points;
  return config;
}

calibration::ViewerConfig MakeViewerConfig() {
  calibration::ViewerConfig config;
  config.window_width = FLAGS_window_width;
  config.window_height = FLAGS_window_height;
  config.play_hz = FLAGS_play_hz;
  config.point_size = FLAGS_point_size;
  config.merged_point_size = FLAGS_merged_point_size;
  config.orthographic_camera = FLAGS_orthographic_camera;
  config.orthographic_extent = FLAGS_orthographic_extent;
  config.show_planes = FLAGS_show_planes;
  config.show_plane_inliers = FLAGS_show_plane_inliers;
  return config;
}

}  // namespace

int Run(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  CHECK(!FLAGS_input_path.empty()) << "input_path is required.";

  const std::vector<calibration::RecordedPacket> packets = calibration::LoadPackets(FLAGS_input_path);
  CHECK(!packets.empty()) << "No packets found in " << FLAGS_input_path;
  LOG(INFO) << "Loaded " << packets.size() << " raw lidar packets from " << FLAGS_input_path;

  std::vector<calibration::ReplayFrame> frames = calibration::BuildReplayFrames(
      packets, FLAGS_accumulate_rings, FLAGS_min_range_m, FLAGS_max_range_m);
  CHECK(!frames.empty()) << "No replay frames prepared.";
  LOG(INFO) << "Prepared " << frames.size() << " replay frames using accumulate_rings="
            << std::max(1, FLAGS_accumulate_rings);

  calibration::UniLidarCalibration active_calibration;
  active_calibration.enabled = false;
  for (calibration::ReplayFrame& frame : frames) {
    calibration::RebuildFramePoints(active_calibration, &frame);
  }

  std::unique_ptr<calibration::ReplayFrame> merged_beginning_frame;
  std::vector<calibration::PlaneModel> planes;
  const calibration::PlaneExtractionConfig plane_config = MakePlaneExtractionConfig();
  const calibration::CalibrationOptimizationConfig calibration_config =
      MakeCalibrationOptimizationConfig(plane_config.min_inliers);

  if (FLAGS_merge_beginning_frames > 0) {
    merged_beginning_frame =
        std::make_unique<calibration::ReplayFrame>(
            calibration::BuildMergedBeginningFrame(frames, FLAGS_merge_beginning_frames));
    calibration::RebuildFramePoints(active_calibration, merged_beginning_frame.get());
    LOG(INFO) << "Merged first "
              << std::min(static_cast<int>(frames.size()), std::max(0, FLAGS_merge_beginning_frames))
              << " frames into " << merged_beginning_frame->points.size() << " background points.";

    planes = calibration::DetectPlanes(*merged_beginning_frame, plane_config);
    calibration::LogPlaneSummary(planes);

    if (!planes.empty()) {
      const calibration::ResidualSummary baseline = calibration::EvaluateResiduals(
          *merged_beginning_frame, planes, calibration_config.assignment_threshold_m);
      LOG(INFO) << "Baseline residuals: assigned=" << baseline.assigned_points << "/"
                << baseline.total_points << " mean_abs=" << baseline.mean_abs_error_m
                << " rms=" << baseline.rms_error_m << " max=" << baseline.max_abs_error_m;

      if (calibration_config.enabled) {
        const calibration::CalibrationSolution solution = calibration::OptimizeCalibration(
            *merged_beginning_frame, planes, std::max(1, FLAGS_accumulate_rings), calibration_config);
        if (std::isfinite(solution.objective)) {
          LOG(INFO) << "Selected calibration model=" << solution.model_name
                    << " coeff_m=" << calibration::VectorSummary(solution.range_coefficients_m)
                    << " alpha_offsets_rad=" << calibration::VectorSummary(solution.delta_alpha_min)
                    << " rms=" << solution.residuals.rms_error_m
                    << " mean_abs=" << solution.residuals.mean_abs_error_m
                    << " assigned=" << solution.residuals.assigned_points;
          active_calibration = solution.calibration;
          active_calibration.enabled = true;
          for (calibration::ReplayFrame& frame : frames) {
            calibration::RebuildFramePoints(active_calibration, &frame);
          }
          calibration::RebuildFramePoints(active_calibration, merged_beginning_frame.get());
        } else {
          LOG(WARNING) << "Calibration optimization failed to find a finite solution.";
        }
      }
    } else {
      LOG(WARNING) << "No planes extracted from merged cloud. Calibration optimization skipped.";
    }
  }

  calibration::RunViewer(
      frames, merged_beginning_frame.get(),
      merged_beginning_frame == nullptr
          ? 0
          : std::min(static_cast<int>(frames.size()), std::max(0, FLAGS_merge_beginning_frames)),
      planes, MakeViewerConfig());
  return 0;
}

}  // namespace third_party

int main(int argc, char** argv) { return third_party::Run(argc, argv); }
