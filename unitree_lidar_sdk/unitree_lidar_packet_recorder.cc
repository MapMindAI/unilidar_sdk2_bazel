// Copyright 2026 MapMindAI Inc. All rights reserved.

#include <unistd.h>

#include <atomic>
#include <cstdint>
#include <csignal>
#include <cstring>
#include <memory>
#include <string>

#include "common/base/glog.h"
#include "common/file/file.h"
#include "gflags/gflags.h"
#include "unitree_lidar_sdk/raw_packet_file.h"
#include "unitree_lidar_sdk/include/unitree_lidar_protocol.h"
#include "unitree_lidar_sdk/include/unitree_lidar_sdk.h"

DEFINE_string(serial_port, "/dev/ttyACM0", "Serial port for Unitree lidar.");
DEFINE_int32(baudrate, 4000000, "Serial baudrate for Unitree lidar.");
DEFINE_int32(work_mode, 8, "Lidar work mode sent after startup.");
DEFINE_bool(use_system_timestamp, true, "Use system timestamp inside the SDK.");
DEFINE_int32(cloud_accumulate_rings, 18, "SDK scan accumulation setting for serial init.");
DEFINE_bool(reset_lidar_mode, false, "Reset lidar mode after startup.");
DEFINE_string(output_path, "/tmp/unitree_lidar_packets.bin", "Output binary file path.");
DEFINE_int32(max_packets, -1, "Maximum number of point packets to record. Negative means no limit.");

namespace dm::third_party {
namespace {

std::atomic<bool> g_stop_requested{false};

void HandleSignal(int /*signum*/) { g_stop_requested.store(true); }

int64_t GetSystemTimestampNs() { return static_cast<int64_t>(unilidar_sdk2::getSystemTimeStamp() * 1e9); }

void WriteAll(dm::file::File* file, const void* data, size_t size) {
  CHECK(file != nullptr);
  CHECK(data != nullptr);
  const auto* bytes = reinterpret_cast<const char*>(data);
  CHECK_EQ(file->Write(bytes, size), size);
}

}  // namespace

int Run(int argc, char** argv) {
  DM_InitGoogleLogging(argc, argv);
  std::signal(SIGINT, HandleSignal);
  std::signal(SIGTERM, HandleSignal);

  CHECK(!FLAGS_output_path.empty()) << "output_path is required.";
  CHECK_OK(dm::file::CheckAndCreateParent(FLAGS_output_path));

  std::unique_ptr<unilidar_sdk2::UnitreeLidarReader> lidar_reader(
      unilidar_sdk2::createUnitreeLidarReader());
  CHECK(lidar_reader != nullptr);

  const int init_status = lidar_reader->initializeSerial(
      FLAGS_serial_port, static_cast<uint32_t>(FLAGS_baudrate),
      static_cast<uint16_t>(FLAGS_cloud_accumulate_rings), FLAGS_use_system_timestamp, 0.0f,
      100.0f);
  CHECK_EQ(init_status, 0) << "Failed to initialize Unitree lidar serial on "
                           << FLAGS_serial_port;

  auto file = dm::file::File::Create(FLAGS_output_path, "wb");
  CHECK_OK(file.status());
  auto out = std::move(file.value());

  RawPacketFileHeader file_header{};
  std::memcpy(file_header.magic, kRawPacketFileMagic, sizeof(file_header.magic));
  file_header.version = kRawPacketFileVersion;
  file_header.packet_size_bytes = sizeof(unilidar_sdk2::LidarPointDataPacket);
  WriteAll(out.get(), &file_header, sizeof(file_header));

  lidar_reader->startLidarRotation();
  ::sleep(1);
  lidar_reader->setLidarWorkMode(static_cast<uint32_t>(FLAGS_work_mode));
  ::sleep(1);
  if (FLAGS_reset_lidar_mode) {
    lidar_reader->resetLidar();
    ::sleep(1);
  }

  int64_t recorded_packets = 0;
  while (!g_stop_requested.load()) {
    const int result = lidar_reader->runParse();
    if (result != LIDAR_POINT_DATA_PACKET_TYPE) {
      continue;
    }

    const unilidar_sdk2::LidarPointDataPacket packet = lidar_reader->getLidarPointDataPacket();
    RawPacketRecordHeader record_header{};
    record_header.host_timestamp_ns = static_cast<uint64_t>(GetSystemTimestampNs());
    record_header.sequence = packet.data.info.seq;
    record_header.packet_size_bytes = sizeof(packet);

    WriteAll(out.get(), &record_header, sizeof(record_header));
    WriteAll(out.get(), &packet, sizeof(packet));
    ++recorded_packets;

    LOG_EVERY_N(INFO, 50) << "Recorded " << recorded_packets
                          << " raw point packets to " << FLAGS_output_path
                          << ", last seq=" << packet.data.info.seq;

    if (FLAGS_max_packets >= 0 && recorded_packets >= FLAGS_max_packets) {
      break;
    }
  }

  lidar_reader->stopLidarRotation();
  lidar_reader->closeSerial();
  LOG(INFO) << "Saved " << recorded_packets << " raw point packets to " << FLAGS_output_path;
  return 0;
}

}  // namespace dm::third_party

int main(int argc, char** argv) { return dm::third_party::Run(argc, argv); }
