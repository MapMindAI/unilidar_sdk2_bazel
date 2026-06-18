// Copyright 2026 MapMindAI Inc. All rights reserved.

#include "unitree_lidar_sdk/calibration/replayer_viewer.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>

#include <pangolin/pangolin.h>

namespace pangolin {

struct OrthographicHandler3D : Handler3D {
  OrthographicHandler3D(OpenGlRenderState* cam_state, AxisDirection enforce_up = AxisNone,
                        float trans_scale = 0.01f,
                        float zoom_fraction = PANGO_DFLT_HANDLER3D_ZF,
                        GLprecision initial_extent = 50)
      : Handler3D((*cam_state), enforce_up, trans_scale, zoom_fraction),
        current(initial_extent) {}

  GLprecision current = 50;

  void Mouse(View& display, MouseButton button, int x, int y, bool pressed,  // NOLINT
             int button_state) override {                                     // NOLINT
    last_pos[0] = static_cast<float>(x);
    last_pos[1] = static_cast<float>(y);
    funcKeyState = 0;
    if (pressed) {
      GetPosNormal(display, x, y, p, Pw, Pc, n, last_z);
      if (ValidWinDepth(p[2])) {
        last_z = p[2];
        std::copy(Pc, Pc + 3, rot_center);
      }
      if (button == MouseWheelUp || button == MouseWheelDown) {
        const GLprecision change = (button == MouseWheelUp ? 1 : -1) * 50 * tf;
        current -= change * std::pow(std::log(std::abs(current) + 1), 2);
        current = std::max<GLprecision>(1e-3, current);
        cam_state->SetProjectionMatrix(pangolin::ProjectionMatrixOrthographic(
            -current, current, -current, current, -5000, 5000));
        return;
      }
      funcKeyState = button_state;
    }

    Handler3D::Mouse(display, button, x, y, pressed, button_state);
  }
};

}  // namespace pangolin

namespace calibration {
namespace {

void RefreshMergedFrame(const CalibrationParameters& parameters, ReplayFrame* merged_frame) {
  if (merged_frame == nullptr) {
    return;
  }
  const UniLidarCalibration calibration = BuildCalibrationFromParameters(parameters);
  RebuildFramePoints(calibration, merged_frame);
}

void DrawPlaneRectangle(const PlaneModel& plane) {
  const Eigen::Vector3f p00 =
      plane.centroid + plane.axis_u * plane.uv_min.x() + plane.axis_v * plane.uv_min.y();
  const Eigen::Vector3f p01 =
      plane.centroid + plane.axis_u * plane.uv_min.x() + plane.axis_v * plane.uv_max.y();
  const Eigen::Vector3f p10 =
      plane.centroid + plane.axis_u * plane.uv_max.x() + plane.axis_v * plane.uv_min.y();
  const Eigen::Vector3f p11 =
      plane.centroid + plane.axis_u * plane.uv_max.x() + plane.axis_v * plane.uv_max.y();

  glColor3f(plane.color.x(), plane.color.y(), plane.color.z());
  glBegin(GL_LINE_LOOP);
  glVertex3f(p00.x(), p00.y(), p00.z());
  glVertex3f(p01.x(), p01.y(), p01.z());
  glVertex3f(p11.x(), p11.y(), p11.z());
  glVertex3f(p10.x(), p10.y(), p10.z());
  glEnd();

  glBegin(GL_LINES);
  glVertex3f(plane.centroid.x(), plane.centroid.y(), plane.centroid.z());
  const Eigen::Vector3f tip = plane.centroid + 0.75f * plane.normal;
  glVertex3f(tip.x(), tip.y(), tip.z());
  glEnd();
}

void DrawPlaneInliers(const ReplayFrame& merged_frame, const std::vector<PlaneModel>& planes,
                      float point_size) {
  glPointSize(point_size);
  glBegin(GL_POINTS);
  for (const PlaneModel& plane : planes) {
    glColor3f(plane.color.x(), plane.color.y(), plane.color.z());
    for (int point_index : plane.point_indices) {
      const Eigen::Vector3f xyz = merged_frame.points[point_index].xyz;
      glVertex3f(xyz.x(), xyz.y(), xyz.z());
    }
  }
  glEnd();
}

}  // namespace

void RunViewer(const std::vector<ReplayFrame>& frames, const ReplayFrame* merged_beginning_frame,
               int merged_frame_count, const std::vector<PlaneModel>& planes,
               const ViewerConfig& config) {
  using Clock = std::chrono::steady_clock;

  std::unique_ptr<ReplayFrame> editable_merged_frame;
  if (merged_beginning_frame != nullptr) {
    editable_merged_frame = std::make_unique<ReplayFrame>(*merged_beginning_frame);
  }
  CalibrationParameters editable_parameters = config.initial_calibration_parameters;
  if (editable_parameters.range_coefficients_m.empty()) {
    editable_parameters.range_coefficients_m = {0.0f, 0.0f, 0.0f};
  }
  if (editable_parameters.alpha_theta_coefficients_rad.empty()) {
    editable_parameters.alpha_theta_coefficients_rad = {0.0f, 0.0f};
  }

  pangolin::CreateWindowAndBind("Unitree Lidar Packet Replayer", config.window_width,
                                config.window_height);
  glEnable(GL_DEPTH_TEST);

  constexpr int kMenuWidth = 340;
  pangolin::CreatePanel("menu").SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(kMenuWidth));
  pangolin::Var<bool> ui_play("menu.Play", false, true);
  pangolin::Var<bool> ui_loop("menu.Loop", true, true);
  pangolin::Var<bool> ui_prev("menu.Prev", false, false);
  pangolin::Var<bool> ui_next("menu.Next", false, false);
  pangolin::Var<bool> ui_reset("menu.Reset", false, false);
  pangolin::Var<bool> ui_refresh_merged("menu.Refresh Merged", false, false);
  pangolin::Var<bool> ui_show_axis("menu.Show Axis", true, true);
  pangolin::Var<bool> ui_show_merged("menu.Show Merged", editable_merged_frame != nullptr, true);
  pangolin::Var<bool> ui_show_points("menu.Show Points", true, true);
  pangolin::Var<bool> ui_show_planes_var("menu.Show Planes", config.show_planes, true);
  pangolin::Var<bool> ui_show_plane_inliers_var("menu.Show Plane Pts", config.show_plane_inliers,
                                                true);
  pangolin::Var<double> ui_range_c0("menu.Range c0", editable_parameters.range_coefficients_m[0],
                                    -0.5, 0.5, false);
  pangolin::Var<double> ui_range_c1("menu.Range c1", editable_parameters.range_coefficients_m.size() > 1
                                                         ? editable_parameters.range_coefficients_m[1]
                                                         : 0.0,
                                    -0.5, 0.5, false);
  pangolin::Var<double> ui_range_c2("menu.Range c2", editable_parameters.range_coefficients_m.size() > 2
                                                         ? editable_parameters.range_coefficients_m[2]
                                                         : 0.0,
                                    -0.5, 0.5, false);
  pangolin::Var<double> ui_alpha_t0("menu.Alpha t0",
                                    editable_parameters.alpha_theta_coefficients_rad[0], -0.02, 0.02,
                                    false);
  pangolin::Var<double> ui_alpha_t1("menu.Alpha t1",
                                    editable_parameters.alpha_theta_coefficients_rad.size() > 1
                                        ? editable_parameters.alpha_theta_coefficients_rad[1]
                                        : 0.0,
                                    -0.02, 0.02, false);
  pangolin::Var<double> ui_alpha_t2("menu.Alpha t2",
                                    editable_parameters.alpha_theta_coefficients_rad.size() > 2
                                        ? editable_parameters.alpha_theta_coefficients_rad[2]
                                        : 0.0,
                                    -0.002, 0.002, false);
  pangolin::Var<double> ui_point_size("menu.Point Size", config.point_size, 1.0, 8.0, true);
  pangolin::Var<double> ui_merged_point_size("menu.Merged Pt Size", config.merged_point_size, 1.0,
                                             8.0, true);
  pangolin::Var<double> ui_play_hz("menu.Play Hz", config.play_hz, 0.5, 30.0, true);
  pangolin::Var<int> ui_frame_idx("menu.Frame", 0, 0,
                                  std::max(0, static_cast<int>(frames.size()) - 1), false);
  pangolin::Var<int> ui_packet_count("menu.Packets/Frame", 0, 0, 0, false);
  pangolin::Var<int> ui_points("menu.Points", 0, 0, 0, false);
  pangolin::Var<int> ui_merged_frames("menu.Merged Frames", merged_frame_count, 0, 0, false);
  pangolin::Var<int> ui_merged_points("menu.Merged Points", editable_merged_frame == nullptr
                                                                ? 0
                                                                : static_cast<int>(
                                                                      editable_merged_frame->points
                                                                          .size()),
                                      0, 0, false);
  pangolin::Var<int> ui_plane_count("menu.Planes", static_cast<int>(planes.size()), 0, 0, false);
  pangolin::Var<int> ui_seq_first("menu.Seq First", 0, 0, 0, false);
  pangolin::Var<int> ui_seq_last("menu.Seq Last", 0, 0, 0, false);

  std::shared_ptr<pangolin::OpenGlRenderState> render_state;
  pangolin::View* view_3d = nullptr;
  if (config.orthographic_camera) {
    const double extent = std::max(1e-3, config.orthographic_extent);
    render_state = std::make_shared<pangolin::OpenGlRenderState>(
        pangolin::ProjectionMatrixOrthographic(-extent, extent, -extent, extent, -5000, 5000),
        pangolin::ModelViewLookAt(0, 0, 20, 0, 0, 0, pangolin::AxisY));
    view_3d =
        &(pangolin::CreateDisplay()
              .SetBounds(0.0, 1.0, pangolin::Attach::Pix(kMenuWidth), 1.0, -1.0f)
              .SetHandler(new pangolin::OrthographicHandler3D(
                  render_state.get(), pangolin::AxisNone, 0.01f,
                  PANGO_DFLT_HANDLER3D_ZF, extent)));
  } else {
    render_state = std::make_shared<pangolin::OpenGlRenderState>(
        pangolin::ProjectionMatrix(1280, 720, 700, 700, 640, 360, 0.1, 5000),
        pangolin::ModelViewLookAt(0, -6, -2, 0, 0, 0, pangolin::AxisY));
    view_3d =
        &(pangolin::CreateDisplay()
              .SetBounds(0.0, 1.0, pangolin::Attach::Pix(kMenuWidth), 1.0, -1280.0f / 720.0f)
              .SetHandler(new pangolin::Handler3D(*render_state)));
  }

  size_t frame_index = 0;
  auto last_advance_time = Clock::now();

  double range_c0 = ui_range_c0;
  double range_c1 = ui_range_c1;
  double range_c2 = ui_range_c2;
  double alpha_t0 = ui_alpha_t0;
  double alpha_t1 = ui_alpha_t1;
  double alpha_t2 = ui_alpha_t2;

  while (!pangolin::ShouldQuit()) {
    const auto now = Clock::now();
    const double elapsed_sec = std::chrono::duration<double>(now - last_advance_time).count();
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
    bool parameter_notchange = (range_c0 == ui_range_c0 && range_c1 == ui_range_c1 && range_c2 == ui_range_c2 &&
        alpha_t0 == ui_alpha_t0 && alpha_t1 == ui_alpha_t1 && alpha_t2 == ui_alpha_t2);
    if (!parameter_notchange) {
      range_c0 = ui_range_c0;
      range_c1 = ui_range_c1;
      range_c2 = ui_range_c2;
      alpha_t0 = ui_alpha_t0;
      alpha_t1 = ui_alpha_t1;
      alpha_t2 = ui_alpha_t2;
    }

    if ((!parameter_notchange || pangolin::Pushed(ui_refresh_merged)) && editable_merged_frame != nullptr) {
      editable_parameters.range_coefficients_m = {static_cast<float>(ui_range_c0),
                                                  static_cast<float>(ui_range_c1),
                                                  static_cast<float>(ui_range_c2)};
      editable_parameters.alpha_theta_coefficients_rad = {
          static_cast<float>(ui_alpha_t0), static_cast<float>(ui_alpha_t1), static_cast<float>(ui_alpha_t2)};
      RefreshMergedFrame(editable_parameters, editable_merged_frame.get());
      ui_merged_points = static_cast<int>(editable_merged_frame->points.size());
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

    if (ui_show_merged && editable_merged_frame != nullptr) {
      glPointSize(static_cast<float>(ui_merged_point_size));
      glBegin(GL_POINTS);
      for (const auto& point : editable_merged_frame->points) {
        glColor3f(point.color.x(), point.color.y(), point.color.z());
        glVertex3f(point.xyz.x(), point.xyz.y(), point.xyz.z());
      }
      glEnd();
    }

    if (ui_show_plane_inliers_var && editable_merged_frame != nullptr) {
      DrawPlaneInliers(*editable_merged_frame, planes,
                       static_cast<float>(ui_merged_point_size) + 1.0f);
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

    if (ui_show_planes_var) {
      glLineWidth(2.0f);
      for (const PlaneModel& plane : planes) {
        DrawPlaneRectangle(plane);
      }
    }

    pangolin::FinishFrame();
  }
}

}  // namespace calibration
