// Copyright 2026 MapMindAI Inc. All rights reserved.

#ifndef UNITREE_LIDAR_SDK_REPLAYER_VIEWER_H_
#define UNITREE_LIDAR_SDK_REPLAYER_VIEWER_H_

#include <vector>

#include "unitree_lidar_sdk/calibration/calibration_optimizer.h"
#include "unitree_lidar_sdk/calibration/plane_extractor.h"
#include "unitree_lidar_sdk/calibration/replayer_common.h"

namespace calibration {

struct ViewerConfig {
  int window_width = 1600;
  int window_height = 900;
  double play_hz = 5.0;
  double point_size = 2.0;
  double merged_point_size = 1.0;
  bool orthographic_camera = true;
  double orthographic_extent = 50.0;
  bool show_planes = true;
  bool show_plane_inliers = true;
  CalibrationParameters initial_calibration_parameters;
};

void RunViewer(const std::vector<ReplayFrame>& frames, const ReplayFrame* merged_beginning_frame,
               int merged_frame_count, const std::vector<PlaneModel>& planes,
               const ViewerConfig& config);

}  // namespace calibration

#endif  // UNITREE_LIDAR_SDK_REPLAYER_VIEWER_H_
