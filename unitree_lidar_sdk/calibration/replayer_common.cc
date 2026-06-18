// Copyright 2026 MapMindAI Inc. All rights reserved.

#include "unitree_lidar_sdk/calibration/replayer_common.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <fstream>
#include <utility>

#include <glog/logging.h>

namespace calibration {

Eigen::Vector3f ColorForRing(int ring, int max_rings) {
  static const std::array<Eigen::Vector3f, 8> kPalette = {
      Eigen::Vector3f(1.0f, 0.25f, 0.25f), Eigen::Vector3f(1.0f, 0.7f, 0.2f),
      Eigen::Vector3f(0.95f, 0.95f, 0.2f), Eigen::Vector3f(0.25f, 0.95f, 0.25f),
      Eigen::Vector3f(0.2f, 0.85f, 1.0f),  Eigen::Vector3f(0.35f, 0.45f, 1.0f),
      Eigen::Vector3f(0.85f, 0.3f, 1.0f),  Eigen::Vector3f(1.0f, 0.35f, 0.7f),
  };
  if (max_rings <= 0) {
    return kPalette[0];
  }
  return kPalette[static_cast<size_t>(ring % max_rings) % kPalette.size()];
}

Eigen::Vector3f ColorForTheta(float theta) {
  constexpr float kTwoPi = 2.0f * static_cast<float>(M_PI);
  float wrapped = std::fmod(theta, kTwoPi);
  if (wrapped < 0.0f) {
    wrapped += kTwoPi;
  }
  const float t = wrapped / kTwoPi;
  if (t < 1.0f / 6.0f) {
    const float u = t * 6.0f;
    return Eigen::Vector3f(1.0f, u, 0.0f);
  }
  if (t < 2.0f / 6.0f) {
    const float u = (t - 1.0f / 6.0f) * 6.0f;
    return Eigen::Vector3f(1.0f - u, 1.0f, 0.0f);
  }
  if (t < 3.0f / 6.0f) {
    const float u = (t - 2.0f / 6.0f) * 6.0f;
    return Eigen::Vector3f(0.0f, 1.0f, u);
  }
  if (t < 4.0f / 6.0f) {
    const float u = (t - 3.0f / 6.0f) * 6.0f;
    return Eigen::Vector3f(0.0f, 1.0f - u, 1.0f);
  }
  if (t < 5.0f / 6.0f) {
    const float u = (t - 4.0f / 6.0f) * 6.0f;
    return Eigen::Vector3f(u, 0.0f, 1.0f);
  }
  const float u = (t - 5.0f / 6.0f) * 6.0f;
  return Eigen::Vector3f(1.0f, 0.0f, 1.0f - u);
}

bool ComputePointPosition(const PointSample& sample, const UniLidarCalibration& calibration,
                          CloudPoint* point) {
  CHECK(point != nullptr);
  if (sample.raw_range < 1) {
    return false;
  }

  const float alpha = sample.alpha_base +
                      (calibration.enabled ? calibration.delta_alpha_theta_fcn(sample.theta) : 0.0f);
  const float base_range =
      sample.range_scale * (static_cast<float>(sample.raw_range) + sample.range_bias);
  const float range =
      base_range + (calibration.enabled ? calibration.delta_range_alpha_fcn(alpha) : 0.0f);
  if (range < sample.packet_range_min || range > sample.packet_range_max) {
    return false;
  }

  const float sin_alpha = std::sin(alpha);
  const float cos_alpha = std::cos(alpha);
  const float sin_theta = std::sin(sample.theta);
  const float cos_theta = std::cos(sample.theta);

  const float a = (-sample.cos_beta_sin_xi + sample.sin_beta_cos_xi * sin_alpha) * range +
                  sample.b_axis_dist;
  const float b = cos_alpha * sample.cos_xi * range;
  const float c = (sample.sin_beta_sin_xi + sample.cos_beta_cos_xi * sin_alpha) * range;

  point->xyz = Eigen::Vector3f(cos_theta * a - sin_theta * b, sin_theta * a + cos_theta * b,
                               c + sample.a_axis_dist);
  point->color = sample.color;
  point->alpha = alpha;
  point->range_m = range;
  point->ring_index = sample.ring_index;
  return true;
}

void AppendPacketSamples(const unilidar_sdk2::LidarPointDataPacket& packet, int ring_index,
                         int max_rings, float min_range_m, float max_range_m,
                         std::vector<PointSample>* samples) {
  CHECK(samples != nullptr);
  const int num_of_points = static_cast<int>(packet.data.point_num);
  const float sin_beta = std::sin(packet.data.param.beta_angle);
  const float cos_beta = std::cos(packet.data.param.beta_angle);
  const float sin_xi = std::sin(packet.data.param.xi_angle);
  const float cos_xi = std::cos(packet.data.param.xi_angle);
  const float alpha_step = packet.data.angle_increment;
  const float theta_step = packet.data.com_horizontal_angle_step;
  const float angle_bias = packet.data.param.alpha_angle_bias;
  const float theta_bias = packet.data.param.theta_angle_bias;
  const float packet_range_min = std::max(packet.data.range_min, min_range_m);
  const float packet_range_max = std::min(packet.data.range_max, max_range_m);
  const Eigen::Vector3f color = ColorForRing(ring_index, std::max(1, max_rings));

  float alpha_cur = packet.data.angle_min + angle_bias;
  float theta_cur = packet.data.com_horizontal_angle_start + theta_bias;
  for (int i = 0; i < num_of_points; ++i, alpha_cur += alpha_step, theta_cur += theta_step) {
    PointSample sample;
    sample.ring_index = ring_index;
    sample.sample_index = i;
    sample.color = color;
    sample.raw_range = packet.data.ranges[i];
    sample.alpha_base = alpha_cur;
    sample.theta = theta_cur;
    sample.range_scale = packet.data.param.range_scale;
    sample.range_bias = packet.data.param.range_bias;
    sample.packet_range_min = packet_range_min;
    sample.packet_range_max = packet_range_max;
    sample.a_axis_dist = packet.data.param.a_axis_dist;
    sample.b_axis_dist = packet.data.param.b_axis_dist;
    sample.cos_beta_sin_xi = cos_beta * sin_xi;
    sample.sin_beta_cos_xi = sin_beta * cos_xi;
    sample.sin_beta_sin_xi = sin_beta * sin_xi;
    sample.cos_beta_cos_xi = cos_beta * cos_xi;
    sample.cos_xi = cos_xi;
    samples->push_back(sample);
  }
}

void RebuildFramePoints(const UniLidarCalibration& calibration, ReplayFrame* frame) {
  CHECK(frame != nullptr);
  frame->points.clear();
  frame->points.reserve(frame->samples.size());
  for (const PointSample& sample : frame->samples) {
    CloudPoint point;
    if (ComputePointPosition(sample, calibration, &point)) {
      frame->points.push_back(std::move(point));
    }
  }
}

std::vector<RecordedPacket> LoadPackets(const std::string& path) {
  std::ifstream input(path, std::ios::binary);
  CHECK(input.is_open()) << "Failed to open input file: " << path;

  third_party::RawPacketFileHeader file_header{};
  CHECK(ReadStruct(&input, &file_header)) << "Failed to read file header.";
  CHECK(std::memcmp(file_header.magic, third_party::kRawPacketFileMagic,
                    sizeof(file_header.magic)) == 0)
      << "Unexpected file magic in " << path;
  CHECK_EQ(file_header.version, third_party::kRawPacketFileVersion)
      << "Unsupported raw packet file version.";
  CHECK_EQ(file_header.packet_size_bytes, sizeof(unilidar_sdk2::LidarPointDataPacket))
      << "Recorded packet size does not match current SDK packet struct.";

  std::vector<RecordedPacket> packets;
  while (true) {
    RecordedPacket packet;
    if (!ReadStruct(&input, &packet.record_header)) {
      break;
    }
    CHECK_EQ(packet.record_header.packet_size_bytes, sizeof(packet.packet))
        << "Encountered record with unexpected packet size.";
    input.read(reinterpret_cast<char*>(&packet.packet), sizeof(packet.packet));
    CHECK(input.good()) << "Truncated packet payload in " << path;
    packets.push_back(std::move(packet));
  }
  return packets;
}

std::vector<ReplayFrame> BuildReplayFrames(const std::vector<RecordedPacket>& packets,
                                           int accumulate_rings, float min_range_m,
                                           float max_range_m) {
  const int rings_per_frame = std::max(1, accumulate_rings);
  std::vector<ReplayFrame> frames;
  ReplayFrame current_frame;
  current_frame.samples.reserve(static_cast<size_t>(rings_per_frame) * 300);

  for (size_t i = 0; i < packets.size(); ++i) {
    if (current_frame.packet_count == 0) {
      current_frame.first_sequence = packets[i].record_header.sequence;
      current_frame.first_host_timestamp_ns = packets[i].record_header.host_timestamp_ns;
    }
    current_frame.last_sequence = packets[i].record_header.sequence;
    current_frame.last_host_timestamp_ns = packets[i].record_header.host_timestamp_ns;
    AppendPacketSamples(packets[i].packet, current_frame.packet_count, rings_per_frame, min_range_m,
                        max_range_m, &current_frame.samples);
    ++current_frame.packet_count;

    if (current_frame.packet_count >= rings_per_frame) {
      frames.push_back(std::move(current_frame));
      current_frame = ReplayFrame{};
      current_frame.samples.reserve(static_cast<size_t>(rings_per_frame) * 300);
    }
  }
  if (current_frame.packet_count > 0) {
    frames.push_back(std::move(current_frame));
  }
  return frames;
}

ReplayFrame BuildMergedBeginningFrame(const std::vector<ReplayFrame>& frames, int merge_count) {
  ReplayFrame merged;
  if (frames.empty() || merge_count <= 0) {
    return merged;
  }

  const size_t clamped_count =
      std::min(frames.size(), static_cast<size_t>(std::max(0, merge_count)));
  size_t total_samples = 0;
  for (size_t i = 0; i < clamped_count; ++i) {
    total_samples += frames[i].samples.size();
  }
  merged.samples.reserve(total_samples);

  merged.first_sequence = frames.front().first_sequence;
  merged.first_host_timestamp_ns = frames.front().first_host_timestamp_ns;
  merged.last_sequence = frames[clamped_count - 1].last_sequence;
  merged.last_host_timestamp_ns = frames[clamped_count - 1].last_host_timestamp_ns;

  for (size_t i = 0; i < clamped_count; ++i) {
    merged.packet_count += frames[i].packet_count;
    merged.samples.insert(merged.samples.end(), frames[i].samples.begin(), frames[i].samples.end());
  }
  for (PointSample& sample : merged.samples) {
    sample.color = ColorForTheta(sample.theta);
  }
  return merged;
}

}  // namespace calibration
