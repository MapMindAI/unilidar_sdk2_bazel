// Copyright 2026 MapMindAI Inc. All rights reserved.

#ifndef UNITREE_LIDAR_SDK_REPLAYER_COMMON_H_
#define UNITREE_LIDAR_SDK_REPLAYER_COMMON_H_

#include <cstdint>
#include <fstream>
#include <functional>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "unitree_lidar_sdk/include/unitree_lidar_protocol.h"
#include "unitree_lidar_sdk/raw_packet_file.h"

namespace calibration {

struct RecordedPacket {
  third_party::RawPacketRecordHeader record_header{};
  unilidar_sdk2::LidarPointDataPacket packet{};
};

struct PointSample {
  int ring_index = 0;
  int sample_index = 0;
  Eigen::Vector3f color = Eigen::Vector3f::Ones();
  uint16_t raw_range = 0;
  float alpha_base = 0.0f;
  float theta = 0.0f;
  float range_scale = 0.0f;
  float range_bias = 0.0f;
  float packet_range_min = 0.0f;
  float packet_range_max = 0.0f;
  float a_axis_dist = 0.0f;
  float b_axis_dist = 0.0f;
  float cos_beta_sin_xi = 0.0f;
  float sin_beta_cos_xi = 0.0f;
  float sin_beta_sin_xi = 0.0f;
  float cos_beta_cos_xi = 0.0f;
  float cos_xi = 0.0f;
};

struct CloudPoint {
  Eigen::Vector3f xyz = Eigen::Vector3f::Zero();
  Eigen::Vector3f color = Eigen::Vector3f::Ones();
  float alpha = 0.0f;
  float range_m = 0.0f;
  int ring_index = 0;
};

struct ReplayFrame {
  std::vector<PointSample> samples;
  std::vector<CloudPoint> points;
  uint32_t first_sequence = 0;
  uint32_t last_sequence = 0;
  uint64_t first_host_timestamp_ns = 0;
  uint64_t last_host_timestamp_ns = 0;
  int packet_count = 0;
};

struct UniLidarCalibration {
  bool enabled = false;
  std::function<float(float /*alpha*/)> delta_range_alpha_fcn = [](float) { return 0.0f; };
  std::function<float(float /*theta*/)> delta_alpha_theta_fcn = [](float) { return 0.0f; };
};

template <typename T>
bool ReadStruct(std::ifstream* input, T* value) {
  input->read(reinterpret_cast<char*>(value), sizeof(T));
  return input->good();
}

Eigen::Vector3f ColorForRing(int ring, int max_rings);
bool ComputePointPosition(const PointSample& sample, const UniLidarCalibration& calibration,
                          CloudPoint* point);
void AppendPacketSamples(const unilidar_sdk2::LidarPointDataPacket& packet, int ring_index,
                         int max_rings, float min_range_m, float max_range_m,
                         std::vector<PointSample>* samples);
void RebuildFramePoints(const UniLidarCalibration& calibration, ReplayFrame* frame);
std::vector<RecordedPacket> LoadPackets(const std::string& path);
std::vector<ReplayFrame> BuildReplayFrames(const std::vector<RecordedPacket>& packets,
                                           int accumulate_rings, float min_range_m,
                                           float max_range_m);
ReplayFrame BuildMergedBeginningFrame(const std::vector<ReplayFrame>& frames, int merge_count);

}  // namespace calibration

#endif  // UNITREE_LIDAR_SDK_REPLAYER_COMMON_H_
