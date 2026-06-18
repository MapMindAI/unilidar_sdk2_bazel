// Copyright 2026 MapMindAI Inc. All rights reserved.

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <glog/logging.h>
#include <pangolin/pangolin.h>

#include "gflags/gflags.h"
#include "unitree_lidar_sdk/calibration/replayer_common.h"
#include "unitree_lidar_sdk/calibration/replayer_viewer.h"
#include "unitree_lidar_sdk/include/unitree_lidar_protocol.h"
#include "unitree_lidar_sdk/raw_packet_file.h"

DEFINE_string(input_path, "/tmp/unitree_lidar_packets.bin", "Input raw packet recording file.");
DEFINE_int32(accumulate_rings, 50, "Number of packets to accumulate into one displayed cloud.");
DEFINE_int32(window_width, 1600, "Viewer width.");
DEFINE_int32(window_height, 900, "Viewer height.");
DEFINE_double(play_hz, 5.0, "Autoplay rate in clouds per second.");
DEFINE_double(point_size, 2.0, "Point size in the viewer.");
DEFINE_double(merged_point_size, 1.0, "Point size for the merged initial-frame cloud.");
DEFINE_double(min_range_m, 0.0, "Additional minimum range filter in meters.");
DEFINE_double(max_range_m, 100.0, "Additional maximum range filter in meters.");
DEFINE_int32(merge_beginning_frames, 1000,
             "Merge the first N replay frames into one static point cloud and "
             "overlay it.");
DEFINE_bool(orthographic_camera, true, "Use an orthographic camera in the Pangolin viewer.");
DEFINE_double(orthographic_extent, 10.0,
              "Half extent of the orthographic camera frustum in viewer units.");

namespace calibration {
namespace internal {

std::vector<RecordedPacket> packets;
std::vector<ReplayFrame> frames;
std::unique_ptr<ReplayFrame> merged_beginning_frame;

double factor_a0 = -0.014;
double factor_a1 = -0.0095;
double factor_a2 = -0.007;

void AppendPacketPoints(const unilidar_sdk2::LidarPointDataPacket& packet, int ring_index,
                        int max_rings, std::vector<CloudPoint>* points) {
  CHECK(points != nullptr);
  const int num_of_points = static_cast<int>(packet.data.point_num);
  const float sin_beta = std::sin(packet.data.param.beta_angle);
  const float cos_beta = std::cos(packet.data.param.beta_angle);
  const float sin_xi = std::sin(packet.data.param.xi_angle);
  const float cos_xi = std::cos(packet.data.param.xi_angle);
  const float cos_beta_sin_xi = cos_beta * sin_xi;
  const float sin_beta_cos_xi = sin_beta * cos_xi;
  const float sin_beta_sin_xi = sin_beta * sin_xi;
  const float cos_beta_cos_xi = cos_beta * cos_xi;
  const float alpha_step = packet.data.angle_increment;
  const float theta_step = packet.data.com_horizontal_angle_step;
  const float angle_bias = packet.data.param.alpha_angle_bias;
  const float theta_bias = packet.data.param.theta_angle_bias;
  const float a_axis_dist = packet.data.param.a_axis_dist;
  const float b_axis_dist = packet.data.param.b_axis_dist;
  const float range_scale = packet.data.param.range_scale;
  const float range_bias = packet.data.param.range_bias;
  const float packet_range_min =
      std::max(packet.data.range_min, static_cast<float>(FLAGS_min_range_m));
  const float packet_range_max =
      std::min(packet.data.range_max, static_cast<float>(FLAGS_max_range_m));

  float alpha_cur = packet.data.angle_min + angle_bias + factor_a0;
  float theta_cur = packet.data.com_horizontal_angle_start + theta_bias;
  const Eigen::Vector3f color = ColorForRing(ring_index, std::max(1, max_rings));

  for (int i = 0; i < num_of_points; ++i, alpha_cur += alpha_step, theta_cur += theta_step) {
    if (packet.data.ranges[i] < 1) {
      continue;
    }
    float range_float =
        range_scale * (static_cast<float>(packet.data.ranges[i]) + range_bias);
    range_float += factor_a1 * 50.0 * range_float * exp(-(1.0 + 20.0 * factor_a2) * range_float);
    if (range_float < packet_range_min || range_float > packet_range_max) {
      continue;
    }

    const float sin_alpha = std::sin(alpha_cur);
    const float cos_alpha = std::cos(alpha_cur);
    const float sin_theta = std::sin(theta_cur);
    const float cos_theta = std::cos(theta_cur);

    const float a = (-cos_beta_sin_xi + sin_beta_cos_xi * sin_alpha) * range_float + b_axis_dist;
    const float b = cos_alpha * cos_xi * range_float;
    const float c = (sin_beta_sin_xi + cos_beta_cos_xi * sin_alpha) * range_float;

    CloudPoint point;
    point.xyz = Eigen::Vector3f(cos_theta * a - sin_theta * b, sin_theta * a + cos_theta * b,
                                c + a_axis_dist);
    point.color = color;
    point.theta = theta_cur;
    points->push_back(point);
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
                                           int accumulate_rings) {
  const int rings_per_frame = std::max(1, accumulate_rings);
  std::vector<ReplayFrame> frames;
  ReplayFrame current_frame;
  current_frame.points.reserve(static_cast<size_t>(rings_per_frame) * 300);

  for (size_t i = 0; i < packets.size(); ++i) {
    if (current_frame.packet_count == 0) {
      current_frame.first_sequence = packets[i].record_header.sequence;
      current_frame.first_host_timestamp_ns = packets[i].record_header.host_timestamp_ns;
    }
    current_frame.last_sequence = packets[i].record_header.sequence;
    current_frame.last_host_timestamp_ns = packets[i].record_header.host_timestamp_ns;
    AppendPacketPoints(packets[i].packet, current_frame.packet_count, rings_per_frame,
                       &current_frame.points);
    ++current_frame.packet_count;

    if (current_frame.packet_count >= rings_per_frame) {
      frames.push_back(std::move(current_frame));
      current_frame = ReplayFrame{};
      current_frame.points.reserve(static_cast<size_t>(rings_per_frame) * 300);
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
  size_t total_points = 0;
  for (size_t i = 0; i < clamped_count; ++i) {
    total_points += frames[i].points.size();
  }
  merged.points.reserve(total_points);

  merged.first_sequence = frames.front().first_sequence;
  merged.first_host_timestamp_ns = frames.front().first_host_timestamp_ns;
  merged.last_sequence = frames[clamped_count - 1].last_sequence;
  merged.last_host_timestamp_ns = frames[clamped_count - 1].last_host_timestamp_ns;

  for (size_t i = 0; i < clamped_count; ++i) {
    merged.packet_count += frames[i].packet_count;
    for (const auto& point : frames[i].points) {
      CloudPoint merged_point = point;
      merged_point.color = ColorForTheta(point.theta);
      merged.points.push_back(std::move(merged_point));
    }
  }
  return merged;
}

void RebuildMap() {
  frames = internal::BuildReplayFrames(packets, FLAGS_accumulate_rings);
  LOG(INFO) << "Prepared " << frames.size()
            << " replay frames using accumulate_rings=" << std::max(1, FLAGS_accumulate_rings);
  if (FLAGS_merge_beginning_frames > 0) {
    merged_beginning_frame = std::make_unique<ReplayFrame>(
        internal::BuildMergedBeginningFrame(frames, FLAGS_merge_beginning_frames));
    LOG(INFO) << "Merged first "
              << std::min(static_cast<int>(frames.size()),
                          std::max(0, FLAGS_merge_beginning_frames))
              << " frames into " << merged_beginning_frame->points.size() << " background points.";
  }
}

void RunViewer() {
  CHECK(!frames.empty()) << "No replay frames loaded.";
  using Clock = std::chrono::steady_clock;

  pangolin::CreateWindowAndBind("Unitree Lidar Packet Replayer", FLAGS_window_width,
                                FLAGS_window_height);
  glEnable(GL_DEPTH_TEST);

  constexpr int kMenuWidth = 280;
  pangolin::CreatePanel("menu").SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(kMenuWidth));
  pangolin::Var<bool> ui_play("menu.Play", false, true);
  pangolin::Var<bool> ui_loop("menu.Loop", true, true);
  pangolin::Var<bool> ui_prev("menu.Prev", false, false);
  pangolin::Var<bool> ui_next("menu.Next", false, false);
  pangolin::Var<bool> ui_reset("menu.Reset", false, false);
  pangolin::Var<bool> ui_show_axis("menu.Show Axis", true, true);
  pangolin::Var<bool> ui_show_merged("menu.Show Merged", true, true);
  pangolin::Var<bool> ui_show_points("menu.Show Points", true, true);
  pangolin::Var<double> ui_point_size("menu.Point Size", FLAGS_point_size, 1.0, 8.0, true);
  pangolin::Var<double> ui_merged_point_size("menu.Merged Pt Size", FLAGS_merged_point_size, 1.0,
                                             8.0, true);
  pangolin::Var<double> ui_play_hz("menu.Play Hz", FLAGS_play_hz, 0.5, 30.0, true);
  pangolin::Var<double> ui_factor_a0("menu.Factor A0", factor_a0, -0.02, 0.02, false);
  pangolin::Var<double> ui_factor_a1("menu.Factor A1", factor_a1, -0.02, 0.02, false);
  pangolin::Var<double> ui_factor_a2("menu.Factor A2", factor_a2, -0.02, 0.02, false);
  pangolin::Var<int> ui_frame_idx("menu.Frame", 0, 0,
                                  std::max(0, static_cast<int>(frames.size()) - 1), false);
  pangolin::Var<int> ui_packet_count("menu.Packets/Frame", std::max(1, FLAGS_accumulate_rings), 0,
                                     0, false);
  pangolin::Var<int> ui_points("menu.Points", 0, 0, 0, false);
  pangolin::Var<int> ui_merged_points("menu.Merged Points",
                                      merged_beginning_frame == nullptr
                                          ? 0
                                          : static_cast<int>(merged_beginning_frame->points.size()),
                                      0, 0, false);
  pangolin::Var<int> ui_seq_first("menu.Seq First", 0, 0, 0, false);
  pangolin::Var<int> ui_seq_last("menu.Seq Last", 0, 0, 0, false);

  std::shared_ptr<pangolin::OpenGlRenderState> render_state;
  pangolin::View* view_3d = nullptr;
  if (FLAGS_orthographic_camera) {
    const double extent = std::max(1e-3, FLAGS_orthographic_extent);
    render_state = std::make_shared<pangolin::OpenGlRenderState>(
        pangolin::ProjectionMatrixOrthographic(-extent, extent, -extent, extent, -5000, 5000),
        pangolin::ModelViewLookAt(0, 0, 20, 0, 0, 0, pangolin::AxisY));
    view_3d =
        &(pangolin::CreateDisplay()
              .SetBounds(0.0, 1.0, pangolin::Attach::Pix(kMenuWidth), 1.0, -1.0f)
              .SetHandler(new pangolin::OrthographicHandler3D(
                  render_state.get(), pangolin::AxisNone, 0.01f, PANGO_DFLT_HANDLER3D_ZF, extent)));
  } else {
    render_state = std::make_shared<pangolin::OpenGlRenderState>(
        pangolin::ProjectionMatrix(1280, 720, 700, 700, 640, 360, 0.1, 5000),
        pangolin::ModelViewLookAt(0, -6, -2, 0, 0, 0, pangolin::AxisY));
    view_3d = &(pangolin::CreateDisplay()
                    .SetBounds(0.0, 1.0, pangolin::Attach::Pix(kMenuWidth), 1.0, -1280.0f / 720.0f)
                    .SetHandler(new pangolin::Handler3D(*render_state)));
  }

  auto check_factor_changed = [&]() {
    bool changed = false;
    if (ui_factor_a0 != factor_a0) {
      changed = true;
      factor_a0 = ui_factor_a0;
    }
    if (ui_factor_a1 != factor_a1) {
      changed = true;
      factor_a1 = ui_factor_a1;
    }
    if (ui_factor_a2 != factor_a2) {
      changed = true;
      factor_a2 = ui_factor_a2;
    }
    if (changed) {
      RebuildMap();
    }
    return changed;
  };

  size_t frame_index = 0;
  auto last_advance_time = Clock::now();
  while (!pangolin::ShouldQuit()) {
    check_factor_changed();
    const auto now = Clock::now();
    const double elapsed_sec =
        std::chrono::duration<double>(now - last_advance_time).count();
    if (ui_play && elapsed_sec >= 1.0 / std::max(0.1, static_cast<double>(ui_play_hz))) {
      last_advance_time = now;
      if (frame_index + 1 < frames.size()) {
        ++frame_index;
      } else if (ui_loop) {
        frame_index = 0;
      } else {
        ui_play = false;
      }
    }

    if (pangolin::Pushed(ui_prev)) {
      frame_index = frame_index == 0 ? 0 : frame_index - 1;
    }
    if (pangolin::Pushed(ui_next)) {
      frame_index = std::min(frame_index + 1, frames.size() - 1);
    }
    if (pangolin::Pushed(ui_reset)) {
      frame_index = 0;
      last_advance_time = now;
    }

    const ReplayFrame& frame = frames[frame_index];
    ui_frame_idx = static_cast<int>(frame_index);
    ui_packet_count = frame.packet_count;
    ui_points = static_cast<int>(frame.points.size());
    ui_seq_first = static_cast<int>(frame.first_sequence);
    ui_seq_last = static_cast<int>(frame.last_sequence);

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    view_3d->Activate(*render_state);
    if (ui_show_axis) {
      pangolin::glDrawAxis(1.0);
    }

    if (ui_show_merged && merged_beginning_frame != nullptr) {
      glPointSize(static_cast<float>(ui_merged_point_size));
      glBegin(GL_POINTS);
      for (const auto& point : merged_beginning_frame->points) {
        glColor3f(point.color.x(), point.color.y(), point.color.z());
        glVertex3f(point.xyz.x(), point.xyz.y(), point.xyz.z());
      }
      glEnd();
    }

    if (ui_show_points) {
      glPointSize(static_cast<float>(ui_point_size));
      glBegin(GL_POINTS);
      for (const auto& point : frame.points) {
        glColor3f(point.color.x(), point.color.y(), point.color.z());
        glVertex3f(point.xyz.x(), point.xyz.y(), point.xyz.z());
      }
      glEnd();
    }

    pangolin::FinishFrame();
  }
}

}  // namespace internal

int Run(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  CHECK(!FLAGS_input_path.empty()) << "input_path is required.";

  internal::packets = internal::LoadPackets(FLAGS_input_path);
  CHECK(!internal::packets.empty()) << "No packets found in " << FLAGS_input_path;
  LOG(INFO) << "Loaded " << internal::packets.size() << " raw lidar packets from "
            << FLAGS_input_path;

  internal::RebuildMap();
  internal::RunViewer();
  return 0;
}

}  // namespace calibration

int main(int argc, char** argv) { return calibration::Run(argc, argv); }
