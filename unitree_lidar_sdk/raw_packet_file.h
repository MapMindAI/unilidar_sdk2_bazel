// Copyright 2026 MapMindAI Inc. All rights reserved.

#ifndef THIRD_PARTY_UNITREE_LIDAR_SDK_RAW_PACKET_FILE_H_
#define THIRD_PARTY_UNITREE_LIDAR_SDK_RAW_PACKET_FILE_H_

#include <cstdint>

namespace dm::third_party {

struct RawPacketFileHeader {
  char magic[8];
  uint32_t version;
  uint32_t packet_size_bytes;
};

struct RawPacketRecordHeader {
  uint64_t host_timestamp_ns;
  uint32_t sequence;
  uint32_t packet_size_bytes;
};

constexpr char kRawPacketFileMagic[8] = {'U', 'L', 'P', 'K', 'T', '0', '1', '\0'};
constexpr uint32_t kRawPacketFileVersion = 1;

}  // namespace dm::third_party

#endif  // THIRD_PARTY_UNITREE_LIDAR_SDK_RAW_PACKET_FILE_H_
