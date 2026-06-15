# Unitree Lidar Collector

## How to use

1. prepare the environment
```
mkdir -p ~/work
cd ~/work
git clone https://github.com/MapMindAI/unilidar_sdk2_bazel.git
bash unilidar_sdk2_bazel/docker_compose/boot_app/enable_unilidar_web_boot.sh
```

2. then got to web `http://<device-ip>:8080/`


# Unitree Lidar SDK

This directory contains:

- the vendor SDK headers and prebuilt libraries under `include/` and `lib/`
- example programs under `examples/`
- a ROS 2 bridge node in [unitree_lidar_rosnode.cc](/unitree_lidar_rosnode.cc)
- a lightweight remote control webserver in `docker_compose/unilidar_mapping/webserver.py`

## `unitree_lidar_rosnode`

`unitree_lidar_rosnode` reads Unitree lidar data from the SDK and publishes:

- `/unilidar/imu` as `sensor_msgs::msg::Imu`
- `/unilidar/cloud` as `sensor_msgs::msg::PointCloud2`

The node supports two cloud-generation paths:

1. `--use_sdk_pointcloud=true`
   Uses `UnitreeLidarReader::getPointCloud(PointCloudUnitree&)` and converts the SDK cloud directly into ROS `PointCloud2`.

2. `--use_sdk_pointcloud=false`
   Uses raw `LidarPointDataPacket` packets and builds `PointCloud2` manually inside `BuildCloudMessage(...)`.

## Highlight: Custom Packet-to-`PointCloud2` Conversion

The custom path is implemented in `BuildCloudMessage(const LidarPointDataPacket&, ...)`.

What it does:

- allocates a fixed `PointCloud2` layout with:
  - `x` at offset `0`
  - `y` at offset `4`
  - `z` at offset `8`
  - `intensity` at offset `16`
  - `ring` at offset `20`
  - `time` at offset `24`
- reads raw ranges and intensities from each Unitree packet
- applies the Unitree calibration parameters
- converts each sample to 3D XYZ
- accumulates multiple single-ring packets into one ROS cloud when `--cloud_accumulate_rings > 1`

This path exists so the project can control:

- exact field layout expected by downstream code
- ring accumulation behavior
- per-point relative timing
- timestamp policy when `--use_system_timestamp` is enabled

## Highlight: Threading

The node has two processing modes:

1. `--threading=true`
   - `ReadLoop()` stays focused on `runParse()`
   - point cloud work is queued and published from `CloudPublishLoop()`
   - this is intended to reduce IMU starvation caused by expensive cloud conversion or publishing

2. `--threading=false`
   - cloud conversion and publish happen inline in `ReadLoop()`
   - this is simpler, but can block IMU handling when cloud processing is slow

When investigating degraded performance, check the startup flag dump first and confirm which mode is actually running.

## Highlight: The Logic Around Line 162

In the custom packet path, the code around [unitree_lidar_rosnode.cc:162](/unitree_lidar_rosnode.cc:162) intentionally does **not** use `packet.data.time_increment` directly:

```cpp
// const float time_step = packet.data.time_increment;
const float time_step = scan_period / num_of_points;
```

Why this exists:

- the packet metadata can report a `time_increment` that does not sum to the packet `scan_period`
- when accumulated across many points/rings, that mismatch can distort per-point relative timing
- downstream LIO code is sensitive to that timing consistency

Current behavior:

- the node replaces `time_increment` with a uniform `scan_period / num_of_points`
- `time_relative` is then advanced by that derived step for every point

This is a deliberate timing normalization step. It may help if the SDK packet timing is inconsistent, but it is also a high-impact change and should be treated as one of the first places to inspect when mapping quality changes.

## Main Runtime Flags

Important flags for this node:

- `--serial_port`
- `--baudrate`
- `--topic_imu`
- `--topic_cloud`
- `--frame_id_imu`
- `--frame_id_cloud`
- `--use_system_timestamp`
- `--cloud_accumulate_rings`
- `--use_sdk_pointcloud`
- `--threading`
- `--reset_lidar_mode`

The node logs the effective values of these flags at startup.

## Remote Web Control

This repo now includes a small Python webserver for remote control of the UniLidar Docker stack.

Files:

- `docker_compose/unilidar_mapping/webserver.py`
- `docker_compose/unilidar_mapping/start_webserver.sh`
- `docker_compose/unilidar_mapping/unilidar-web.service`
- `docker_compose/boot_app/enable_unilidar_web_boot.sh`

What it does:

- runs `docker_compose/unilidar_mapping/arm64_start_unilidar.sh`
- runs `docker_compose/unilidar_mapping/arm64_stop_unilidar.sh`
- shows the latest Docker logs from container `UniLidarSdk`

Run it on the remote device:

```bash
cd /home/cat/work/unilidar_sdk2_bazel
python3 docker_compose/unilidar_mapping/webserver.py
```

or:

```bash
cd /home/cat/work/unilidar_sdk2_bazel
bash docker_compose/unilidar_mapping/start_webserver.sh
```

Then open:

```text
http://<device-ip>:8080
```

Optional environment variables:

- `UNILIDAR_WEB_HOST` default `0.0.0.0`
- `UNILIDAR_WEB_PORT` default `8080`
- `UNILIDAR_COMPOSE_NAME` default `unilidar_collection`
- `UNILIDAR_CONTAINER_NAME` default `UniLidarSdk`

Notes:

- the webserver uses only Python standard library modules
- it expects `docker` and `docker compose` to be installed on the target device
- it currently has no authentication, so expose it only on a trusted network or behind a reverse proxy/VPN

### Enable At Boot

This repo includes a `systemd` service file and an installer script for the target device.

Install and enable the webserver on boot:

```bash
cd /home/cat/work/unilidar_sdk2_bazel
sudo bash docker_compose/boot_app/enable_unilidar_web_boot.sh
```

This installs:

- `docker_compose/unilidar_mapping/unilidar-web.service` to `/etc/systemd/system/unilidar-web.service`

Then it runs:

- `systemctl daemon-reload`
- `systemctl enable unilidar-web.service`
- `systemctl restart unilidar-web.service`
