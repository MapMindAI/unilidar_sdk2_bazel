#!/usr/bin/python3
# coding=utf-8
"""RTK GPS publisher — minimal ROS 2 port of gps_ntrip_node.py."""

import base64
import math
import os
import socket
import time

try:
    import serial
except ImportError as exc:
    raise SystemExit(
        "Missing dependency: pyserial. Install with: sudo apt install python3-serial"
    ) from exc

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus

# ── Config from environment (replaces hardcoded values in reference) ──────────
PORT             = os.environ.get("RTK_SERIAL_PORT",   "/dev/ttyUSB0")
BAUDRATE         = int(os.environ.get("RTK_BAUDRATE",  "115200"))
FRAME_ID         = os.environ.get("RTK_FRAME_ID",      "rtk")
FIX_TOPIC        = os.environ.get("RTK_FIX_TOPIC",     "/rtk/fix")
NTRIP_SERVER     = os.environ.get("NTRIP_HOST",        "")
NTRIP_PORT       = int(os.environ.get("NTRIP_PORT",    "2101"))
NTRIP_USERNAME   = os.environ.get("NTRIP_USER",        "")
NTRIP_PASSWORD   = os.environ.get("NTRIP_PASSWORD",    "")
NTRIP_MOUNTPOINT = os.environ.get("NTRIP_MOUNTPOINT",  "")

def log(msg):
    # print(msg, flush=True)
    return

AskGGA = "GPGGA 0.1\r\n"

# ── Globals (same as reference) ───────────────────────────────────────────────
sp             = None
ntrip_socket   = None
ntrip_is_connect = False
gga_rx_flag    = False
gga_rx_data    = ""
node           = None
fix_pub        = None


def send_gpgga_to_gps():
    global sp, gga_rx_flag
    if gga_rx_flag is True:
        gga_rx_flag = False
        return
    if sp is not None and sp.is_open:
        try:
            sp.write(AskGGA.encode('utf-8'))
            log("[gps] sent GPGGA request")
        except Exception as e:
            log(f"[gps] ERROR: failed to send GPGGA: {e}")
            node.get_logger().error(f"Failed to send GPGGA to GPS: {e}")


def send_gpgga_to_ntrip():
    global ntrip_socket
    if ntrip_socket is None:
        return
    try:
        ntrip_socket.send(gga_rx_data.encode('utf-8') + b"\r\n")
        log(f"[ntrip] sent GGA: {gga_rx_data.strip()}")
    except Exception as e:
        log(f"[ntrip] ERROR: failed to send GGA: {e}")
        return


def parse_gps(data):
    global gga_rx_flag, gga_rx_data
    for line in data.splitlines():
        line = line.strip()
        if not line:
            continue
        if line.startswith('$'):
            log(f"[serial] {line}")
    if "$GPGGA" in data or "$GNGGA" in data:
        fields = data.split(',')
        if len(fields) >= 14:
            try:
                fix_quality = int(fields[6])
                if fix_quality is None:
                    return
                lat_raw   = float(fields[2])
                lon_raw   = float(fields[4])
                altitude  = float(fields[9])

                lat_deg = int(lat_raw // 100)
                lat = lat_deg + (lat_raw - lat_deg * 100) / 60.0
                if fields[3] == 'S':
                    lat = -lat

                lon_deg = int(lon_raw // 100)
                lon = lon_deg + (lon_raw - lon_deg * 100) / 60.0
                if fields[5] == 'W':
                    lon = -lon

                gga_rx_flag = True
                gga_rx_data = data.strip()
                log(f"[gps] fix={fix_quality} lat={lat:.7f} lon={lon:.7f} alt={altitude:.2f}m")
                node.get_logger().info(gga_rx_data)

                # Publish NavSatFix
                msg = NavSatFix()
                msg.header.stamp = node.get_clock().now().to_msg()
                msg.header.frame_id = FRAME_ID
                if fix_quality <= 0:
                    msg.status.status = NavSatStatus.STATUS_NO_FIX
                elif fix_quality in (2, 4, 5):
                    msg.status.status = NavSatStatus.STATUS_GBAS_FIX
                else:
                    msg.status.status = NavSatStatus.STATUS_FIX
                msg.status.service = (
                    NavSatStatus.SERVICE_GPS
                    | NavSatStatus.SERVICE_GLONASS
                    | NavSatStatus.SERVICE_COMPASS
                    | NavSatStatus.SERVICE_GALILEO
                )
                msg.latitude  = lat
                msg.longitude = lon
                msg.altitude  = altitude
                msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
                fix_pub.publish(msg)
                log(f"[ros] published NavSatFix lat={lat:.7f} lon={lon:.7f}")

            except (ValueError, IndexError) as e:
                log(f"[gps] WARNING: failed to parse GGA fields: {e} | data={data.strip()!r}")
                return


def gps_ntrip_node():
    global sp, ntrip_socket, ntrip_is_connect, gga_rx_flag, gga_rx_data, node, fix_pub

    log("[init] starting rtk_ros_publisher")
    log(f"[config] PORT={PORT}  BAUDRATE={BAUDRATE}  FRAME_ID={FRAME_ID}  TOPIC={FIX_TOPIC}")
    log(f"[config] NTRIP_SERVER={NTRIP_SERVER!r}  NTRIP_PORT={NTRIP_PORT}  MOUNTPOINT={NTRIP_MOUNTPOINT!r}")

    log("[init] calling rclpy.init()")
    rclpy.init()
    node    = Node('rtk_ros_publisher')
    fix_pub = node.create_publisher(NavSatFix, FIX_TOPIC, 20)
    log(f"[init] ROS2 node ready, publishing on {FIX_TOPIC}")
    node.get_logger().info(f"Opening {PORT} at {BAUDRATE} baud")
    node.get_logger().info(f"NTRIP_SERVER:NTRIP_PORT : {NTRIP_SERVER}:{NTRIP_PORT}")
    node.get_logger().info(f"NTRIP_MOUNTPOINT : {NTRIP_MOUNTPOINT}")
    node.get_logger().info(f"NTRIP_USERNAME : {NTRIP_USERNAME}")
    node.get_logger().info(f"NTRIP_PASSWORD : {NTRIP_PASSWORD}")

    # ── Phase 1: open serial and wait for first GGA ───────────────────────────
    log(f"[phase1] opening serial port {PORT} @ {BAUDRATE}")
    try:
        sp = serial.Serial(PORT, BAUDRATE, timeout=1)
    except Exception as e:
        log(f"[phase1] ERROR: cannot open {PORT}: {e}")
        raise
    log(f"[phase1] serial port open, sending GPGGA request")
    time.sleep(1)
    sp.write(AskGGA.encode('utf-8'))

    rate_sec = 1.0 / 100.0
    buffer = bytearray()
    wait_log_t = time.monotonic()

    node.get_logger().info("[phase1] waiting for first GGA sentence...")
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.0)
        if sp.in_waiting > 0:
            data = sp.read(sp.in_waiting)
            if isinstance(data, str):
                data = data.encode('utf-8')
            buffer.extend(data)
        else:
            if len(buffer) > 0:
                try:
                    data_str = buffer.decode('utf-8')
                    parse_gps(data_str)
                    buffer = bytearray()
                except UnicodeDecodeError:
                    log(f"[phase1] WARNING: UnicodeDecodeError, raw bytes={bytes(buffer)!r}")
                    node.get_logger().warning("Failed to decode data, check data format")
                    buffer = bytearray()
        if gga_rx_flag is True:
            log("[phase1] got first GGA — proceeding")
            break
        now = time.monotonic()
        if now - wait_log_t >= 5.0:
            log(f"[phase1] still waiting for GGA... (buffered {len(buffer)} bytes)")
            wait_log_t = now
        time.sleep(rate_sec)

    # ── Phase 2: connect NTRIP (if configured) ────────────────────────────────
    if NTRIP_SERVER:
        node.get_logger().info(f"[phase2] connecting to NTRIP {NTRIP_SERVER}:{NTRIP_PORT} mount={NTRIP_MOUNTPOINT!r}")
        try:
            ntrip_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            ntrip_socket.connect((NTRIP_SERVER, NTRIP_PORT))
            node.get_logger().info("[phase2] TCP connected to NTRIP server, sending HTTP request")

            auth_str    = NTRIP_USERNAME + ":" + NTRIP_PASSWORD
            auth_base64 = base64.b64encode(auth_str.encode('utf-8')).decode('utf-8')
            request = (
                "GET /%s HTTP/1.0\r\n"
                "User-Agent: NTRIP ntrip_client\r\n"
                "Accept:*/*\r\n"
                "Connection:close\r\n"
                "Authorization: Basic %s\r\n"
                "\r\n"
            ) % (NTRIP_MOUNTPOINT, auth_base64)
            ntrip_socket.send(request.encode('utf-8'))
            node.get_logger().info("[phase2] NTRIP HTTP request sent, waiting for ICY 200 OK")
        except Exception as e:
            node.get_logger().error(f"Failed to connect to NTRIP server: {e}")
            ntrip_socket = None
    else:
        node.get_logger().info("NTRIP_HOST not set — running without NTRIP corrections")

    # ── Phase 3: main loop ────────────────────────────────────────────────────
    next_ask_gps   = time.monotonic() + 3.0
    next_send_ntrip = time.monotonic() + 1.0
    buffer = bytearray()
    heartbeat_t = time.monotonic()
    node.get_logger().info("[phase3] entering main loop")

    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.0)
        now = time.monotonic()

        if now >= next_ask_gps:
            send_gpgga_to_gps()
            next_ask_gps = now + 3.0

        if now >= next_send_ntrip:
            send_gpgga_to_ntrip()
            next_send_ntrip = now + 1.0

        # Read serial
        if sp.in_waiting > 0:
            data = sp.read(sp.in_waiting)
            if isinstance(data, str):
                data = data.encode('utf-8')
            buffer.extend(data)

            start_index = buffer.find(b'$')
            end_index   = buffer.find(b'\r\n')
            if start_index != -1 and end_index != -1 and start_index < end_index:
                gps_data     = buffer[start_index:end_index + 2]
                decoded_data = gps_data.decode('utf-8', errors='ignore')
                parse_gps(decoded_data)
                buffer = bytearray()

        # Read RTCM from NTRIP and forward to serial
        if ntrip_socket is not None:
            try:
                ntrip_socket.settimeout(0.01)
                rtcm_data = ntrip_socket.recv(4096)
                if rtcm_data:
                    if b"ICY 200 OK" in rtcm_data:
                        ntrip_is_connect = True
                        log("[ntrip] received ICY 200 OK — NTRIP stream active")
                        send_gpgga_to_ntrip()
                        node.get_logger().info("Connected to NTRIP server.")
                    else:
                        log(f"[ntrip] received {len(rtcm_data)} bytes RTCM → serial")
                    sp.write(rtcm_data)
            except socket.timeout:
                pass
            except Exception as e:
                node.get_logger().error(f"Error receiving RTCM data: {e}")

        if now - heartbeat_t >= 30.0:
            log(f"[heartbeat] running, gga_rx_flag={gga_rx_flag}, ntrip_connected={ntrip_is_connect}")
            heartbeat_t = now

        time.sleep(rate_sec)

    log("[shutdown] exiting main loop")
    if sp is not None and sp.is_open:
        sp.close()
        log("[shutdown] serial port closed")
    if ntrip_socket is not None:
        ntrip_socket.close()
        log("[shutdown] NTRIP socket closed")
    node.destroy_node()
    rclpy.shutdown()
    log("[shutdown] done")


if __name__ == '__main__':
    try:
        gps_ntrip_node()
    except KeyboardInterrupt:
        log("[shutdown] KeyboardInterrupt")
    except Exception as e:
        log(f"[FATAL] unhandled exception: {e}")
        raise
