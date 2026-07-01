# WTRTK-960H — RTK GNSS Module

## What is RTK?

**Real-Time Kinematic (RTK)** is a differential GNSS technique that achieves centimeter-level positioning accuracy in real time. A base station at a known location transmits correction data (RTCM) to a rover receiver; the rover resolves carrier-phase ambiguities against those corrections and converges to a **fixed solution** (typically ±1–2 cm) within seconds.

Three solution states appear in the output:

| Status | Typical accuracy | Meaning |
|--------|-----------------|---------|
| **Fixed RTK** (固定解) | ~1 cm horizontal | Carrier-phase ambiguities fully resolved |
| **Float RTK** (浮动解) | ~0.1–0.5 m | Ambiguities estimated but not fixed |
| **Single** (单点) | ~1.5 m | No differential corrections applied |

---

## Product — WTRTK-960H

**Manufacturer:** 深圳维特智能科技有限公司 (WitMotion ShenZhen Co., Ltd.)  
**Model:** [WTRTK-960H产品资料](https://wit-motion.yuque.com/wumwnr/docs/isgpx42utz06df8t)
**Purchase:** [Tmall product page](https://detail.tmall.com/item.htm?from=detail&id=935624628635&mi_id=00008K2JT8Mq5BgEpJ4TTt9aTxbyj5R2_PaubFyVEZBoSlw&spm=tbpc.orderdetail.suborder_itemtitle.1.48866aa60VQGuR)  
**Official Docs:** [WTRTK-960H 连接手机APP — 语雀](https://wit-motion.yuque.com/wumwnr/docs/ghvamu57709huc22?singleDoc#)

The WTRTK-960H is a full-constellation, all-frequency RTK GNSS module built on Unicore's **NebulasIV (UM960)** SoC. The **H** variant adds a USB / Bluetooth interface for direct smartphone connectivity, making it suitable for field surveying and mobile GIS workflows.

---

## Specifications

### GNSS Performance

| Parameter | Value |
|-----------|-------|
| Chip | Unicore NebulasIV (UM960) |
| Channels | 1408 |
| Constellations | BDS · GPS · GLONASS · Galileo · QZSS · SBAS |
| Frequencies (BDS) | B1I / B2I / B3I / B1C / B2a |
| Frequencies (GPS) | L1 C/A · L2P · L5 |
| Frequencies (GLONASS) | L1 · L2 |
| Frequencies (Galileo) | E1 · E5b · E5a |
| Frequencies (QZSS) | L1 · L2 · L5 |
| Update rate | 20 Hz |
| Cold-start TTFF | ≤ 30 s |
| RTK initialization | < 5 s (typical) |

### Positioning Accuracy (CEP)

| Mode | Horizontal | Vertical |
|------|-----------|----------|
| Single-point | 1.5 m | 2.5 m |
| DGPS | 0.4 m | 0.8 m |
| **RTK Fixed** | **0.8 cm + 1 ppm** | **1.5 cm + 1 ppm** |

### Other Performance

| Parameter | Value |
|-----------|-------|
| Speed accuracy | 0.03 m/s |
| Time accuracy | 20 ns |
| Anti-jamming | 60 dB narrowband suppression |

### Electrical & Mechanical

| Parameter | Value |
|-----------|-------|
| Dimensions | 26 × 38 × 7.6 mm |
| Power supply | 5 V |
| Typical current | 158 mA |
| Operating temperature | −40 °C to +85 °C |
| Storage temperature | −45 °C to +125 °C |

### Interfaces

| Interface | Details |
|-----------|---------|
| USB | Type-C (plug-and-play) |
| Embedded | XH2.54 × 6-Pin UART |
| Serial baud | 4800–921600 bps (default 115200) |
| Protocol | NMEA 0183 |
| Differential input | RTCM 2.3 · RTCM 3.x · CMR |

---

## Connecting to the Smartphone App

The WTRTK-960H connects to Android/iOS via **USB OTG** or **Bluetooth**, enabling the WitMotion mobile app (or any GNSS app that accepts an external NMEA source) to receive RTK-corrected positions in the field.

Full step-by-step connection guide:  
→ [WTRTK-960H 连接手机APP](https://wit-motion.yuque.com/wumwnr/docs/ghvamu57709huc22?singleDoc#)

General workflow:

1. Plug the WTRTK-960H into the phone via USB-C OTG cable (or pair over Bluetooth).
2. Open the WitMotion app and select the device from the port list.
3. Configure the NTRIP caster (mountpoint, credentials) to receive corrections from a network RTK service or local base station.
4. Once the status shows **固定解 (Fixed)**, positions are centimeter-accurate.
5. Log the trajectory as a tab-separated `.txt` file for post-processing.

---

## Data Format

The device logs one record per second as a tab-separated text file with a Chinese header row. Each record spans **3 lines** due to embedded newlines in the system-time and GPS-time fields.

**Header columns:**

```
时间 | 设备名称 | 系统时间 | GPS时间 | 卫星数量 | 定位状态 | 定向状态 | 解状态 |
经度(°) | 纬度(°) | 位置精度 | GPS高度(m) | 地速(km) | GPS航向(°) |
俯仰角(°) | 基线长度(m) | 差分龄期 | 距离基站(m)
```

**Parsing notes:**
- A new record starts on any line matching `YYYY-M-D HH:MM:SS.mmm`.
- The longitude, latitude, altitude, and fix-status fields are on the **3rd line** of each record (index 2), tab-separated starting at field index 5.

---

## Web Trajectory Visualizer

An interactive map viewer is included at [`web/rtk_viewer.html`](web/rtk_viewer.html).

**Features:**
- Parses the raw `.txt` log format directly in the browser
- OpenStreetMap and Esri Satellite tile layers (toggle top-right)
- Trajectory color-coded by fix type: green = Fixed, orange = Float, red = Single
- Click/hover any point for timestamp, coordinates, altitude, accuracy, satellite count
- Stats bar: total points, path distance, duration, fixed-RTK percentage

**Run with a local HTTP server** (required for auto-loading the data file):

```bash
# from the repo root
python3 -m http.server 8765
# then open:
# http://localhost:8765/web/rtk_viewer.html
```

Alternatively, open `rtk_viewer.html` directly and use the **"Load RTK .txt"** button to pick the file manually.

---

## Live ROS 2 Publisher

The USB-C interface exposes live **NMEA 0183** serial data. This repo includes
[`tools/rtk_ros_publisher.py`](tools/rtk_ros_publisher.py), a ROS 2 Humble
publisher for the WTRTK-960H.

Install runtime dependencies:

```bash
sudo apt install python3-serial
source /opt/ros/humble/setup.bash
```

Run:

```bash
/usr/bin/python3 tools/rtk_ros_publisher.py \
  --port=/dev/ttyACM0 \
  --baudrate=115200 \
  --frame-id=rtk
```

Run with an NTRIP/RTK correction server:

```bash
export RTK_NTRIP_PASSWORD='your-password'

/usr/bin/python3 tools/rtk_ros_publisher.py \
  --port=/dev/ttyACM0 \
  --baudrate=115200 \
  --frame-id=rtk \
  --ntrip-host=your.caster.example.com \
  --ntrip-port=2101 \
  --ntrip-mountpoint=YOUR_MOUNTPOINT \
  --ntrip-user=your-user \
  --ntrip-password-env=RTK_NTRIP_PASSWORD
```

The script logs into the NTRIP caster, sends live GGA rover-position updates
to the caster, receives RTCM correction bytes, and writes those bytes back to
the WTRTK-960H over the same USB serial link. Use `--ntrip-tls` if your caster
requires TLS.

Published topic:

| Topic | Type | Source |
|-------|------|--------|
| `/rtk/fix` | `sensor_msgs/msg/NavSatFix` | NMEA GGA |

Each published fix is also logged with fix status, latitude, longitude,
satellite count, and altitude.

Useful check:

```bash
ros2 topic echo /rtk/fix
```

Docker compose service:

```bash
sudo nano /etc/unilidar/rtk.env
sudo systemctl restart unilidar-web.service
```

Example `/etc/unilidar/rtk.env`:

```bash
RTK_SERIAL_PORT=/dev/ttyUSB0
RTK_BAUDRATE=115200
RTK_FRAME_ID=rtk
RTK_FIX_TOPIC=/rtk/fix

NTRIP_HOST=your.caster.example.com
NTRIP_PORT=2101
NTRIP_MOUNTPOINT=YOUR_MOUNTPOINT
NTRIP_USER=your-user
NTRIP_PASSWORD=your-password
NTRIP_TLS=false
```

The web app is launched by `systemd`, so it does not read `~/.bashrc`.
`docker_compose/boot_app/enable_unilidar_web_boot.sh` creates
`/etc/unilidar/rtk.env` and configures `unilidar-web.service` to load it.
The compose file starts `RtkPublisher` and records `/rtk/fix` in the rosbag
alongside the LiDAR topics. Leave `NTRIP_HOST` empty to publish uncorrected
GNSS fixes without NTRIP.

If the USB device appears as another port, list candidates with:

```bash
ls /dev/ttyACM* /dev/ttyUSB*
```

---


## RTK Trajectory Viewer

An interactive map viewer is included at [`web/rtk_viewer.html`](web/rtk_viewer.html).

<img width="1117" height="514" alt="screenshot-20260701-154516" src="https://github.com/user-attachments/assets/7ccb1a38-094d-4215-8de0-b86182417938" />


**Features:**
- Parses the raw `.txt` log format directly in the browser (no server required for manual load)
- OpenStreetMap and Esri Satellite tile layers; upscales tiles beyond native zoom instead of showing "data not available"
- Trajectory color-coded by fix type: **green** = Fixed · **orange** = Float · **red** = Single
- Click / hover any point for timestamp, coordinates, altitude, accuracy, satellite count
- Stats bar: total points, path distance, duration, fixed-RTK percentage

**Export a ROS 2 bag to viewer format:**

```bash
source /opt/ros/humble/setup.bash
python3 tools/bag_to_rtk_txt.py /path/to/bag/        # writes <bag>_rtk.txt
python3 tools/bag_to_rtk_txt.py bag.db3 -o out.txt   # explicit output
```

**Run with a local HTTP server** (required for auto-loading the data file):

```bash
# from the repo root
python3 -m http.server 8765
# then open:
# http://localhost:8765/web/rtk_viewer.html
```

Alternatively, open `rtk_viewer.html` directly and use the **"Load RTK .txt"** button to pick the file manually.


## Applications

- Ground-truth trajectory for LiDAR-inertial odometry evaluation
- Georeferencing point clouds from UniLiDAR sessions
- High-precision GIS data collection
- Mobile mapping and surveying
