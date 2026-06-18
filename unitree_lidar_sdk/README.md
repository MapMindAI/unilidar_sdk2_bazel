# Unitree LiDAR SDK

This directory wraps the vendor `unilidar_sdk2` library and adds a few repo-specific tools:

- a ROS2 publisher node for IMU + point cloud
- a raw packet recorder for `LidarPointDataPacket`
- an offline replayer with Pangolin visualization

## Files

- [unitree_lidar_rosnode.cc](./unitree_lidar_rosnode.cc): live ROS2 node
- [unitree_lidar_packet_recorder.cc](./unitree_lidar_packet_recorder.cc): saves raw `LidarPointDataPacket` records
- [unitree_lidar_packet_replayer.cc](./unitree_lidar_packet_replayer.cc): loads saved packets, decodes them, and opens a Pangolin viewer
- [raw_packet_file.h](./raw_packet_file.h): shared on-disk packet file format
- `include/`: vendor SDK headers
- `lib/`: vendor static libraries

## Live ROS Node

The ROS node reads the lidar over serial and publishes:

- IMU: `sensor_msgs::msg::Imu`
- cloud: `sensor_msgs::msg::PointCloud2`

Example:

```bash
bazel-bin/third_party/unitree_lidar_sdk/unitree_lidar_rosnode \
  --serial_port=/dev/ttyACM0 \
  --baudrate=4000000 \
  --topic_imu=/unilidar/imu \
  --topic_cloud=/unilidar/cloud \
  --cloud_accumulate_rings=18
```

Notable flags:

- `--use_sdk_pointcloud=true|false`
- `--fix_interring_ts=true|false`
- `--cloud_accumulate_rings=<N>`
- `--reset_lidar_mode=true|false`

`--use_sdk_pointcloud=false` uses the custom packet-to-XYZ conversion in this repo.

## Raw Packet Recorder

The recorder stores the raw vendor packet bytes for each `LidarPointDataPacket`, plus a small host-side record header. This is useful when you want to:

- replay the exact same packet stream offline
- compare different decode formulas
- debug calibration / angle interpretation without a live sensor

Example:

```bash
bazel-bin/unitree_lidar_sdk/unitree_lidar_packet_recorder \
  --serial_port=/dev/ttyACM0 \
  --output_path=data/unitree_lidar_packets.bin \
  --max_packets=1000
```

Useful flags:

- `--output_path=<file>`
- `--max_packets=<N>`; negative means no limit
- `--work_mode=<mode>`
- `--reset_lidar_mode=true|false`

Stop with `Ctrl+C`.

## Offline Replayer

The replayer loads the recorded packet file, decodes each packet with the same geometry model used by the live node, groups packets into clouds, and visualizes them in Pangolin.

Example:

```bash
bazel-bin/unitree_lidar_sdk/unitree_lidar_packet_replayer \
  --input_path=data/unitree_lidar_packets.bin \
  --accumulate_rings=50 \
  --merge_beginning_frames=10
```

Useful flags:

- `--input_path=<file>`
- `--accumulate_rings=<N>`
- `--merge_beginning_frames=<N>`
- `--play_hz=<rate>`
- `--point_size=<size>`
- `--merged_point_size=<size>`
- `--min_range_m=<min>`
- `--max_range_m=<max>`

Viewer controls:

- `Play`: autoplay frames
- `Prev` / `Next`: step one frame
- `Reset`: go back to frame 0
- `Loop`: restart when reaching the end
- `Show Merged`: overlay the merged first `N` frames as a static background cloud
- `Point Size`: point size in Pangolin
- `Merged Pt Size`: point size for the merged background cloud

Each replay frame stores:

- first and last packet sequence id
- first and last host receive timestamp
- decoded point list

## Raw Packet File Format

The capture format is defined in [raw_packet_file.h](./raw_packet_file.h).

File layout:

1. `RawPacketFileHeader`
2. repeated records:
   - `RawPacketRecordHeader`
   - raw `unilidar_sdk2::LidarPointDataPacket` bytes

Current format constants:

- magic: `ULPKT01`
- version: `1`

`RawPacketRecordHeader` stores:

- `host_timestamp_ns`: host receive time when the packet was written
- `sequence`: lidar packet sequence from `packet.data.info.seq`
- `packet_size_bytes`: byte size of the following raw packet payload

## Decode Model

Both the live node custom path and the offline replayer use the same basic decode path:

```text
raw ranges + calibration params
  -> alpha/theta stepping
  -> local calibrated coordinates (a, b, c)
  -> horizontal rotation by theta
  -> final Cartesian point (x, y, z)
```

The relevant vendor fields come from `unilidar_sdk2::LidarPointDataPacket`:

- `com_horizontal_angle_start`
- `com_horizontal_angle_step`
- `angle_min`
- `angle_increment`
- `beta_angle`
- `xi_angle`
- `theta_angle_bias`
- `alpha_angle_bias`
- `a_axis_dist`
- `b_axis_dist`
- `range_bias`
- `range_scale`

If you are debugging plane striping or line-to-line offsets, recording packets and replaying them is the easiest way to test alternate hypotheses without needing the live device.

## Notes

- The recorder/replayer currently focus on `LIDAR_POINT_DATA_PACKET_TYPE` only.
- The replayer opens a GUI window, so it needs a desktop session.
- The packet file is tied to the current vendor struct layout. If the vendor SDK changes packet structure, update `raw_packet_file.h` compatibility checks and rebuild.
