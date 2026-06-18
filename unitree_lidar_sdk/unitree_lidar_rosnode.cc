// Copyright 2026 MapMindAI Inc. All rights reserved.

#include <unistd.h>
#include <algorithm>
#include <atomic>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <glog/logging.h>
#include "gflags/gflags.h"
#include "builtin_interfaces/msg/time.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "unitree_lidar_sdk/include/unitree_lidar_sdk.h"

DEFINE_string(serial_port, "/dev/ttyACM0", "Serial port for Unitree lidar.");
DEFINE_int32(baudrate, 4000000, "Serial baudrate for Unitree lidar.");
DEFINE_string(topic_imu, "/unilidar/imu", "ROS topic for sensor_msgs/Imu.");
DEFINE_string(topic_cloud, "/unilidar/cloud", "ROS topic for sensor_msgs/PointCloud2.");
DEFINE_string(frame_id_imu, "unilidar_imu", "Frame id for IMU messages.");
DEFINE_string(frame_id_cloud, "unilidar", "Frame id for point cloud messages.");
DEFINE_int32(qos_depth, 20, "Publisher queue depth.");
DEFINE_int32(work_mode, 8, "Lidar work mode sent after startup.");
DEFINE_bool(use_system_timestamp, true, "Use system timestamp instead of lidar hardware timestamp.");
DEFINE_int32(cloud_queue_size, 10, "Maximum queued point clouds for async ROS publishing.");
DEFINE_int32(cloud_accumulate_rings, 25,
             "Number of single-ring packets to accumulate before publishing.");
// custom packet decode has higher efficient
DEFINE_bool(use_sdk_pointcloud, false,
            "Use Unitree SDK getPointCloud() output instead of custom packet-to-PointCloud2 conversion.");
DEFINE_bool(fix_interring_ts, false, "Fix the interval between rings, when use_sdk_pointcloud == false");
DEFINE_bool(threading, true, "Process point cloud publishing on a separate thread, so we won't lost imu data");

DEFINE_bool(reset_lidar_mode, false, "Reset Lidar mode to serial");
DEFINE_bool(stop_rotate_after_quit, false, "stop rotate after quit");


DEFINE_double(alpha_bais_bias, 0.0, "bias for alpha bias");
DEFINE_double(range_fix_a0, 0.0, "range fix a0");
DEFINE_double(range_fix_a1, 0.0, "range fix a1");


namespace third_party {
namespace {

constexpr size_t kPointStep = 32;
constexpr size_t kXOffset = 0;
constexpr size_t kYOffset = 4;
constexpr size_t kZOffset = 8;
constexpr size_t kIntensityOffset = 16;
constexpr size_t kRingOffset = 20;
constexpr size_t kTimeOffset = 24;

builtin_interfaces::msg::Time ToRosTime(double stamp_s) {
  builtin_interfaces::msg::Time stamp;
  const double floor_sec = std::floor(stamp_s);
  stamp.sec = static_cast<int32_t>(floor_sec);
  stamp.nanosec = static_cast<uint32_t>(
      std::llround((stamp_s - floor_sec) * 1000000000.0));
  if (stamp.nanosec >= 1000000000u) {
    stamp.sec += 1;
    stamp.nanosec -= 1000000000u;
  }
  return stamp;
}

builtin_interfaces::msg::Time ToRosTime(const unilidar_sdk2::TimeStamp& stamp_in) {
  builtin_interfaces::msg::Time stamp;
  stamp.sec = static_cast<int32_t>(stamp_in.sec);
  stamp.nanosec = stamp_in.nsec;
  return stamp;
}

static int64_t ToNs(const builtin_interfaces::msg::Time& stamp) {
  return static_cast<int64_t>(stamp.sec) * 1000000000LL + static_cast<int64_t>(stamp.nanosec);
}

static double ToS(const builtin_interfaces::msg::Time& stamp) {
  return static_cast<double>(ToNs(stamp)) * 1e-9;
}

sensor_msgs::msg::Imu BuildImuMessage(const unilidar_sdk2::LidarImuData& imu) {
  sensor_msgs::msg::Imu msg;
  msg.header.frame_id = FLAGS_frame_id_imu;
  // msg.header.stamp = FLAGS_use_system_timestamp ? ToRosTime(unilidar_sdk2::getSystemTimeStamp())
  //                                               : ToRosTime(imu.info.stamp);
  msg.header.stamp = ToRosTime(imu.info.stamp);
  msg.orientation.x = imu.quaternion[0];
  msg.orientation.y = imu.quaternion[1];
  msg.orientation.z = imu.quaternion[2];
  msg.orientation.w = imu.quaternion[3];
  msg.angular_velocity.x = imu.angular_velocity[0];
  msg.angular_velocity.y = imu.angular_velocity[1];
  msg.angular_velocity.z = imu.angular_velocity[2];
  msg.linear_acceleration.x = imu.linear_acceleration[0];
  msg.linear_acceleration.y = imu.linear_acceleration[1];
  msg.linear_acceleration.z = imu.linear_acceleration[2];
  return msg;
}

void InitializeMessage(sensor_msgs::msg::PointCloud2& msg) {
  msg.header.frame_id = FLAGS_frame_id_cloud;
  msg.height = 1;
  msg.is_bigendian = false;
  msg.is_dense = false;
  msg.point_step = kPointStep;

  msg.fields.resize(7);
  msg.fields[0].name = "x";
  msg.fields[0].offset = kXOffset;
  msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
  msg.fields[0].count = 1;
  msg.fields[1].name = "y";
  msg.fields[1].offset = kYOffset;
  msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
  msg.fields[1].count = 1;
  msg.fields[2].name = "z";
  msg.fields[2].offset = kZOffset;
  msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
  msg.fields[2].count = 1;
  msg.fields[3].name = "padding";
  msg.fields[3].offset = 12;
  msg.fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
  msg.fields[3].count = 1;
  msg.fields[4].name = "intensity";
  msg.fields[4].offset = kIntensityOffset;
  msg.fields[4].datatype = sensor_msgs::msg::PointField::FLOAT32;
  msg.fields[4].count = 1;
  msg.fields[5].name = "ring";
  msg.fields[5].offset = kRingOffset;
  msg.fields[5].datatype = sensor_msgs::msg::PointField::UINT16;
  msg.fields[5].count = 1;
  msg.fields[6].name = "time";
  msg.fields[6].offset = kTimeOffset;
  msg.fields[6].datatype = sensor_msgs::msg::PointField::FLOAT32;
  msg.fields[6].count = 1;

  size_t num_of_points = 300;
  msg.data.assign(static_cast<size_t>(num_of_points) * kPointStep * FLAGS_cloud_accumulate_rings,
                  0);
}

bool BuildCloudMessage(const unilidar_sdk2::LidarPointDataPacket& packet, bool use_system_timestamp,
                       sensor_msgs::msg::PointCloud2& msg) {
  static bool pcl_init = false;
  if (!pcl_init) {
    InitializeMessage(msg);
    pcl_init = true;
  }
  builtin_interfaces::msg::Time current_stamp;
  if (use_system_timestamp) {
    current_stamp = ToRosTime(unilidar_sdk2::getSystemTimeStamp() - packet.data.scan_period);
  } else {
    current_stamp = ToRosTime(packet.data.info.stamp);
  }

  const int num_of_points = packet.data.point_num;
  const float scan_period = packet.data.scan_period;
  // time_increment*300=0.0023115, scan_period=0.004623
  const float time_step = packet.data.time_increment;
  const float sin_beta = std::sin(packet.data.param.beta_angle);   // beta_angle: 0.015708
  const float cos_beta = std::cos(packet.data.param.beta_angle);
  const float sin_xi = std::sin(packet.data.param.xi_angle);  // xi_angle: 0.00545415
  const float cos_xi = std::cos(packet.data.param.xi_angle);
  const float cos_beta_sin_xi = cos_beta * sin_xi;
  const float sin_beta_cos_xi = sin_beta * cos_xi;
  const float sin_beta_sin_xi = sin_beta * sin_xi;
  const float cos_beta_cos_xi = cos_beta * cos_xi;
  const float alpha_step = packet.data.angle_increment;            // 0.010472 == M_PI / 300
  const float theta_step = packet.data.com_horizontal_angle_step;  // approx M_PI / 15000
  const float angle_bias = packet.data.param.alpha_angle_bias;     // 0.0395972
  const float theta_bias = packet.data.param.theta_angle_bias;     // 2.0769
  const float a_axis_dist = packet.data.param.a_axis_dist;
  const float b_axis_dist = packet.data.param.b_axis_dist;
  const float range_scale = packet.data.param.range_scale;
  const float range_bias = packet.data.param.range_bias;
  const float packet_range_min = packet.data.range_min;
  const float packet_range_max = packet.data.range_max;
  const auto& ranges = packet.data.ranges;
  const auto& intensities = packet.data.intensities;

  // Packet-to-cloud model:
  //
  //   one packet = one scanned ring / sweep strip
  //              = N samples with shared calibration + per-sample range/intensity
  //
  //   packet.data
  //     |- param.{beta, xi, alpha_bias, theta_bias, a_axis_dist, b_axis_dist}
  //     |- angle_min + i * angle_increment                  -> alpha_i
  //     |- com_horizontal_angle_start + i * theta_step     -> theta_i
  //     |- ranges[i], intensities[i]
  //     `- scan_period / point_num                         -> dt between samples
  //
  //   For each sample i:
  //     raw range/count -> metric range -> calibrated local beam coordinates (a, b, c)
  //                      -> rotate by horizontal angle theta_i
  //                      -> final point (x, y, z)
  //                      -> write {x, y, z, intensity, ring, relative_time} into PointCloud2
  //
  //   After FLAGS_cloud_accumulate_rings packets:
  //     ring 0 + ring 1 + ... + ring K-1 -> one published PointCloud2 message
  //
  // Geometry intuition for the variables below:
  //   1. alpha_i is the per-point angle within the current ring.
  //   2. theta_i is the horizontal rotation of the whole ring sample.
  //   3. beta/xi plus a_axis_dist/b_axis_dist are factory calibration terms from the SDK.
  //   4. (a, b, c) is the calibrated point before the final horizontal rotation.
  //   5. (x, y, z) is the ROS-frame point written into the outgoing cloud.

  static size_t number_of_pts = 0;
  static uint16_t accumulated_rings = 0;
  static float time_relative = 0.0f;
  if (accumulated_rings == 0) {
    // first ring, assign timestamp
    msg.header.stamp = current_stamp;
  } else if (FLAGS_fix_interring_ts) {
    // fix the delta
    float delta = ToS(current_stamp) - time_relative - ToS(msg.header.stamp);
    time_relative += delta;
  }

  float alpha_cur = packet.data.angle_min + angle_bias + FLAGS_alpha_bais_bias;
  float theta_cur = packet.data.com_horizontal_angle_start + theta_bias;

  // static float last_com_horizontal_angle_end = 0.0;
  // // packet.data.com_horizontal_angle_start - last_com_horizontal_angle_start ~ 0.062 = M_PI / 15000 * 300
  // LOG(INFO) << accumulated_rings << " " << theta_bias << " " << packet.data.com_horizontal_angle_start << " "
  //           << packet.data.com_horizontal_angle_start - last_com_horizontal_angle_end;
  // last_com_horizontal_angle_end = packet.data.com_horizontal_angle_start  + theta_step * 300;

  for (int i = 0; i < num_of_points;
       ++i, alpha_cur += alpha_step, theta_cur += theta_step, time_relative += time_step) {
    // ranges[i] is the raw return in sensor units. First reject obviously invalid returns,
    // then convert into metric distance with SDK-provided scale/bias and clamp against the
    // packet's valid range interval.
    float range_i = ranges[i];
    if (ranges[i] < 1) {
      range_i = 0.0;
    }
    float range_float = range_scale * (static_cast<float>(ranges[i]) + range_bias);
    range_float += FLAGS_range_fix_a0 * 50.0 * range_float * exp(-(1.0 + 20.0 * FLAGS_range_fix_a1) * range_float);

    if (range_float < packet_range_min || range_float > packet_range_max) {
      range_float = 0.0;
    }

    const float sin_alpha = std::sin(alpha_cur);
    const float cos_alpha = std::cos(alpha_cur);
    const float sin_theta = std::sin(theta_cur);
    const float cos_theta = std::cos(theta_cur);

    // Calibrated beam model from Unitree's packet definition:
    //
    //   range_float + {alpha, beta, xi} + {a_axis_dist, b_axis_dist}
    //        -> intermediate coordinates (a, b, c)
    //        -> rotate in the horizontal plane by theta
    //        -> Cartesian point:
    //
    //             x =  cos(theta) * a - sin(theta) * b
    //             y =  sin(theta) * a + cos(theta) * b
    //             z =  c + a_axis_dist
    //
    // Think of (a, b, c) as the sensor-calibrated point before the final azimuth rotation.
    const float a = (-cos_beta_sin_xi + sin_beta_cos_xi * sin_alpha) * range_float + b_axis_dist;
    const float b = cos_alpha * cos_xi * range_float;
    const float c = (sin_beta_sin_xi + cos_beta_cos_xi * sin_alpha) * range_float;
    const float x = cos_theta * a - sin_theta * b;
    const float y = sin_theta * a + cos_theta * b;
    const float z = c + a_axis_dist;
    const float intensity = intensities[i];

    // Each PointCloud2 point stores:
    //   [x, y, z, padding, intensity, ring_id, relative_time]
    // where relative_time is the sample time offset from msg.header.stamp.
    const size_t base = number_of_pts * kPointStep;
    std::memcpy(msg.data.data() + base + kXOffset, &x, sizeof(float));
    std::memcpy(msg.data.data() + base + kYOffset, &y, sizeof(float));
    std::memcpy(msg.data.data() + base + kZOffset, &z, sizeof(float));
    std::memcpy(msg.data.data() + base + kIntensityOffset, &intensity, sizeof(float));
    std::memcpy(msg.data.data() + base + kRingOffset, &accumulated_rings, sizeof(uint16_t));
    std::memcpy(msg.data.data() + base + kTimeOffset, &time_relative, sizeof(float));
    number_of_pts++;
  }

  accumulated_rings = accumulated_rings + 1;
  time_relative += (scan_period - num_of_points * time_step);

  bool publish = false;
  if (accumulated_rings >= FLAGS_cloud_accumulate_rings) {
    msg.width = number_of_pts;
    msg.row_step = msg.point_step * msg.width;

    accumulated_rings = 0;
    number_of_pts = 0;
    time_relative = 0.0f;
    publish = true;
    LOG_EVERY_N(INFO, 10) << "[LIDAR] publish (from LidarPointDataPacket) " << msg.width << " points"
                          <<  time_step * 300 << "-" << scan_period;
  }
  return publish;
}

sensor_msgs::msg::PointCloud2 BuildCloudMessage(const unilidar_sdk2::PointCloudUnitree& cloud,
                                                bool /*use_system_timestamp*/) {
  sensor_msgs::msg::PointCloud2 msg;
  InitializeMessage(msg);
  // msg.header.stamp = use_system_timestamp ? ToRosTime(unilidar_sdk2::getSystemTimeStamp())
  //                                         : ToRosTime(cloud.stamp);
  msg.header.stamp = ToRosTime(cloud.stamp);
  msg.width = static_cast<uint32_t>(cloud.points.size());
  msg.row_step = msg.point_step * msg.width;
  msg.data.resize(static_cast<size_t>(msg.row_step), 0);
  for (size_t i = 0; i < cloud.points.size(); ++i) {
    const auto& pt = cloud.points[i];
    const size_t base = i * kPointStep;
    const uint16_t ring = static_cast<uint16_t>(pt.ring);
    std::memcpy(msg.data.data() + base + kXOffset, &pt.x, sizeof(float));
    std::memcpy(msg.data.data() + base + kYOffset, &pt.y, sizeof(float));
    std::memcpy(msg.data.data() + base + kZOffset, &pt.z, sizeof(float));
    std::memcpy(msg.data.data() + base + kIntensityOffset, &pt.intensity, sizeof(float));
    std::memcpy(msg.data.data() + base + kRingOffset, &ring, sizeof(uint16_t));
    std::memcpy(msg.data.data() + base + kTimeOffset, &pt.time, sizeof(float));
  }
  LOG_EVERY_N(INFO, 10) << "[LIDAR] publish (from PointCloudUnitree) " << msg.width << " points";
  return msg;
}

struct QueuedCloud {
  bool is_sdk_cloud = false;
  unilidar_sdk2::LidarPointDataPacket packet;
  unilidar_sdk2::PointCloudUnitree sdk_cloud;
};

class UnitreeLidarRosNode final : public rclcpp::Node {
 public:
  UnitreeLidarRosNode()
      : rclcpp::Node("unitree_lidar_rosnode"),
        lidar_reader_(unilidar_sdk2::createUnitreeLidarReader()) {
    CHECK(lidar_reader_ != nullptr);
    const auto qos = rclcpp::QoS(rclcpp::KeepLast(std::max(1, FLAGS_qos_depth)));
    pub_imu_ = create_publisher<sensor_msgs::msg::Imu>(FLAGS_topic_imu, qos);
    pub_cloud_ = create_publisher<sensor_msgs::msg::PointCloud2>(FLAGS_topic_cloud, qos);

    const int init_status =
        lidar_reader_->initializeSerial(FLAGS_serial_port, static_cast<uint32_t>(FLAGS_baudrate),
                                        FLAGS_cloud_accumulate_rings, FLAGS_use_system_timestamp, 0, 100.0);
    CHECK_EQ(init_status, 0) << "Failed to initialize Unitree lidar serial on "
                             << FLAGS_serial_port;
    LOG(INFO) << "Initialized Unitree lidar serial on " << FLAGS_serial_port
              << " baudrate=" << FLAGS_baudrate;
    LOG(INFO) << "Unitree lidar gflags:"
              << " serial_port=" << FLAGS_serial_port
              << " baudrate=" << FLAGS_baudrate
              << " topic_imu=" << FLAGS_topic_imu
              << " topic_cloud=" << FLAGS_topic_cloud
              << " frame_id_imu=" << FLAGS_frame_id_imu
              << " frame_id_cloud=" << FLAGS_frame_id_cloud
              << " qos_depth=" << FLAGS_qos_depth
              << " work_mode=" << FLAGS_work_mode
              << " use_system_timestamp=" << FLAGS_use_system_timestamp
              << " cloud_queue_size=" << FLAGS_cloud_queue_size
              << " cloud_accumulate_rings=" << FLAGS_cloud_accumulate_rings
              << " use_sdk_pointcloud=" << FLAGS_use_sdk_pointcloud
              << " threading=" << FLAGS_threading
              << " reset_lidar_mode=" << FLAGS_reset_lidar_mode;

    lidar_reader_->startLidarRotation();
    lidar_reader_->setLidarWorkMode(static_cast<uint32_t>(FLAGS_work_mode));
    // lidar_reader_->syncLidarTimeStamp();
    ::sleep(1);
    if (FLAGS_reset_lidar_mode) {
      lidar_reader_->resetLidar();
      ::sleep(1);
    }

    worker_thread_ = std::thread([this]() { ReadLoop(); });
    if (FLAGS_threading) {
      cloud_thread_ = std::thread([this]() { CloudPublishLoop(); });
    }
  }

  ~UnitreeLidarRosNode() override {
    stop_requested_.store(true);
    cloud_cv_.notify_all();
    if (worker_thread_.joinable()) {
      worker_thread_.join();
    }
    if (FLAGS_threading && cloud_thread_.joinable()) {
      cloud_thread_.join();
    }
    if (lidar_reader_ != nullptr) {
      if (FLAGS_stop_rotate_after_quit) lidar_reader_->stopLidarRotation();
      lidar_reader_->closeSerial();
    }
  }

 private:
  void EnqueueCloud(QueuedCloud cloud) {
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    const size_t max_queue_size = static_cast<size_t>(std::max(1, FLAGS_cloud_queue_size));
    if (cloud_queue_.size() >= max_queue_size) {
      cloud_queue_.pop_front();
      LOG(WARNING) << "Dropping stale Unitree cloud due to publisher backlog.";
    }
    cloud_queue_.emplace_back(std::move(cloud));
    cloud_cv_.notify_one();
  }

  void CloudPublishLoop() {
    sensor_msgs::msg::PointCloud2 msg;
    while (true) {
      QueuedCloud cloud;
      {
        std::unique_lock<std::mutex> lock(cloud_mutex_);
        cloud_cv_.wait(lock, [this]() { return stop_requested_.load() || !cloud_queue_.empty(); });
        if (stop_requested_.load() && cloud_queue_.empty()) return;
        cloud = std::move(cloud_queue_.front());
        cloud_queue_.pop_front();
      }
      if (cloud.is_sdk_cloud) {
        pub_cloud_->publish(BuildCloudMessage(cloud.sdk_cloud, FLAGS_use_system_timestamp));
      } else if (BuildCloudMessage(cloud.packet, FLAGS_use_system_timestamp, msg)) {
        pub_cloud_->publish(msg);
      }
    }
  }

  void ReadLoop() {
    unilidar_sdk2::LidarImuData imu;
    unilidar_sdk2::PointCloudUnitree cloud;
    sensor_msgs::msg::PointCloud2 msg;
    bool had_lidar = false;
    while (rclcpp::ok() && !stop_requested_.load()) {
      const int result = lidar_reader_->runParse();
      switch (result) {
        case LIDAR_IMU_DATA_PACKET_TYPE:
          // only publish imu after receiving lidar
          if (lidar_reader_->getImuData(imu) && had_lidar) {
            pub_imu_->publish(BuildImuMessage(imu));
          }
          break;
        case LIDAR_POINT_DATA_PACKET_TYPE:
        had_lidar = true;
          if (FLAGS_threading) {
            if (FLAGS_use_sdk_pointcloud) {
              if (lidar_reader_->getPointCloud(cloud)) {
                EnqueueCloud({true, {}, std::move(cloud)});
              }
            } else {
              EnqueueCloud({false, lidar_reader_->getLidarPointDataPacket(), {}});
            }
          } else {
            if (FLAGS_use_sdk_pointcloud) {
              if (lidar_reader_->getPointCloud(cloud)) {
                pub_cloud_->publish(BuildCloudMessage(cloud, FLAGS_use_system_timestamp));
              }
            } else {
              if (BuildCloudMessage(lidar_reader_->getLidarPointDataPacket(),
                                    FLAGS_use_system_timestamp, msg)) {
                pub_cloud_->publish(msg);
              }
            }
          }
          break;
        default:
          break;
      }
    }
  }

  unilidar_sdk2::UnitreeLidarReader* lidar_reader_ = nullptr;
  std::atomic<bool> stop_requested_ = false;
  std::thread worker_thread_;
  std::thread cloud_thread_;
  std::mutex cloud_mutex_;
  std::condition_variable cloud_cv_;
  std::deque<QueuedCloud> cloud_queue_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_;
};

}  // namespace
}  // namespace third_party

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  rclcpp::init(argc, argv);
  auto node = std::make_shared<third_party::UnitreeLidarRosNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
