# Unitree Lidar Collector

![raw top](assets/mapping.png)

## How to use

1. prepare the environment
```
mkdir -p ~/work
cd ~/work
git clone https://github.com/MapMindAI/unilidar_sdk2_bazel.git
bash unilidar_sdk2_bazel/docker_compose/boot_app/enable_unilidar_web_boot.sh
```

2. then got to web `http://<device-ip>:8080/`

## Unitree Lidar SDK

This repo provides:

- the vendor SDK headers and prebuilt libraries under `include/` and `lib/`
- example programs under `examples/`
- a ROS 2 bridge node in [unitree_lidar_rosnode.cc](/unitree_lidar_rosnode.cc)
- a lightweight remote control webserver in `docker_compose/unilidar_mapping/webserver.py`
- offline calibration tools under `unitree_lidar_sdk/calibration/`

## Highlight: Custom Packet-to-`PointCloud2` Conversion

`unitree_lidar_rosnode` reads Unitree lidar data from the SDK and publishes:

- `/unilidar/imu` as `sensor_msgs::msg::Imu`
- `/unilidar/cloud` as `sensor_msgs::msg::PointCloud2`

The node supports two cloud-generation paths:

1. `--use_sdk_pointcloud=true`
   Uses `UnitreeLidarReader::getPointCloud(PointCloudUnitree&)` and converts the SDK cloud directly into ROS `PointCloud2`.

2. `--use_sdk_pointcloud=false`
   Uses raw `LidarPointDataPacket` packets and builds `PointCloud2` manually inside `BuildCloudMessage(...)`.

<details>
<summary>Custom Packet-to-<code>PointCloud2</code> Conversion</summary>

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
</details>

## Highlight: Calibration

Offline calibration tools live under [`unitree_lidar_sdk/calibration/`](/unitree_lidar_sdk/calibration), and the full workflow is documented in [`unitree_lidar_sdk/README_calibrate.md`](/unitree_lidar_sdk/README_calibrate.md).

- `unitree_lidar_packet_auto_calibrator`: extracts planes from merged point clouds, evaluates point-to-plane residuals, and searches calibration parameters automatically
- `unitree_lidar_packet_manual_calibrator`: opens the same replay viewer with manual calibration parameters for visual inspection and tuning

**Current Issue**: the automatic calibration still does not give an ideal result.
I currently use manual adjustment in `unitree_lidar_packet_replayer`: load recorded packets,
apply hand-tuned correction factors during decode, rebuild the merged cloud, and inspect the
result in Pangolin until the geometry looks consistent.

| view | before | after |
|---|---|---|
| top view | ![raw top](assets/raw_top.png) | ![calib top](assets/calib_top.png) |
| side view | ![raw side](assets/raw_side.png) | ![calib side](assets/calib_side.png) |


## Highlight: Remote Web Control

This repo includes a small Python webserver for remote control of the UniLidar Docker stack.
It can start and stop the compose stack, copy recorded data to drive, and show the latest Docker logs from container `UniLidarSdk`.

Run it on the remote device:

```bash
cd /home/cat/work/unilidar_sdk2_bazel
python3 docker_compose/unilidar_mapping/webserver.py
```

Or:

```bash
cd /home/cat/work/unilidar_sdk2_bazel
bash docker_compose/unilidar_mapping/start_webserver.sh
```

Then open `http://<device-ip>:8080`.

Optional environment variables:

- `UNILIDAR_WEB_HOST` default `0.0.0.0`
- `UNILIDAR_WEB_PORT` default `8080`
- `UNILIDAR_COMPOSE_NAME` default `unilidar_collection`
- `UNILIDAR_CONTAINER_NAME` default `UniLidarSdk`

<details>
<summary>Enable At Boot</summary>


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

See logs with `sudo journalctl -u unilidar-web.service -b`.
</details>
