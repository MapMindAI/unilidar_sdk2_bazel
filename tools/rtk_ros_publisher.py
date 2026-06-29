#!/usr/bin/python3
"""Publish WTRTK-960H USB NMEA data as ROS 2 topics."""

import argparse
import base64
import math
import os
import socket
import ssl
import sys
import threading
import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

try:
    import serial
except ImportError as exc:
    raise SystemExit(
        "Missing dependency: pyserial. Install with: sudo apt install python3-serial"
    ) from exc

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus


class NtripClient:
    def __init__(
        self,
        node: Node,
        args: argparse.Namespace,
        serial_port: serial.Serial,
        serial_write_lock: threading.Lock,
    ):
        self.node = node
        self.args = args
        self.serial_port = serial_port
        self.serial_write_lock = serial_write_lock
        self.stop_event = threading.Event()
        self.thread: Optional[threading.Thread] = None
        self.latest_gga: Optional[str] = None
        self.latest_gga_lock = threading.Lock()

    def start(self) -> None:
        self.thread = threading.Thread(target=self.run, name="ntrip_client", daemon=True)
        self.thread.start()

    def stop(self) -> None:
        self.stop_event.set()
        if self.thread is not None:
            self.thread.join(timeout=3.0)

    def update_gga(self, raw_line: str) -> None:
        with self.latest_gga_lock:
            self.latest_gga = raw_line

    def run(self) -> None:
        while not self.stop_event.is_set() and rclpy.ok():
            try:
                self.connect_and_forward()
            except Exception as exc:
                self.node.get_logger().warning(f"NTRIP disconnected: {exc}")
            self.stop_event.wait(self.args.ntrip_reconnect_sec)

    def connect_socket(self) -> socket.socket:
        sock = socket.create_connection(
            (self.args.ntrip_host, self.args.ntrip_port),
            timeout=self.args.ntrip_connect_timeout,
        )
        if self.args.ntrip_tls:
            context = ssl.create_default_context()
            sock = context.wrap_socket(sock, server_hostname=self.args.ntrip_host)
        sock.settimeout(self.args.ntrip_response_timeout)
        return sock

    def request_headers(self) -> bytes:
        mountpoint = self.args.ntrip_mountpoint.lstrip("/")
        request = [
            f"GET /{mountpoint} HTTP/1.0",
            f"Host: {self.args.ntrip_host}:{self.args.ntrip_port}",
            "User-Agent: NTRIP rtk_ros_publisher/1.0",
            "Ntrip-Version: Ntrip/2.0",
            "Accept: */*",
            "Connection: close",
        ]
        password = self.args.ntrip_password
        if self.args.ntrip_password_env:
            password = os.environ.get(self.args.ntrip_password_env, password)
        if self.args.ntrip_user or password:
            token = base64.b64encode(f"{self.args.ntrip_user}:{password}".encode("utf-8"))
            request.append(f"Authorization: Basic {token.decode('ascii')}")
        request.extend(["", ""])
        return "\r\n".join(request).encode("ascii")

    def read_response_header(self, sock: socket.socket) -> Tuple[str, bytes]:
        data = b""
        while b"\r\n\r\n" not in data:
            try:
                chunk = sock.recv(4096)
            except socket.timeout as exc:
                raise RuntimeError(
                    "timed out waiting for NTRIP response header; check caster host/port, "
                    "TLS setting, mountpoint, username/password, and network reachability"
                ) from exc
            if not chunk:
                raise RuntimeError(
                    "caster closed before response header; check host/port, TLS setting, "
                    "mountpoint, and NTRIP username/password"
                )
            data += chunk
            if len(data) > 16384:
                raise RuntimeError("caster response header too large")
        header, rest = data.split(b"\r\n\r\n", 1)
        return header.decode("iso-8859-1", errors="replace"), rest

    def send_latest_gga(self, sock: socket.socket, force: bool = False) -> bool:
        with self.latest_gga_lock:
            gga = self.latest_gga
        if gga is None and self.args.ntrip_gga:
            gga = self.args.ntrip_gga
        if gga is None:
            if force:
                self.node.get_logger().warning(
                    "NTRIP is connected, but no GGA is available yet for caster position updates"
                )
            return False
        sock.sendall((gga.strip() + "\r\n").encode("ascii", errors="ignore"))
        return True

    def connect_and_forward(self) -> None:
        with self.connect_socket() as sock:
            sock.sendall(self.request_headers())
            header, initial_rtcm = self.read_response_header(sock)
            first_line = header.splitlines()[0] if header else ""
            if not (first_line.startswith("ICY 200") or " 200 " in first_line):
                raise RuntimeError(first_line or "unexpected empty caster response")

            self.node.get_logger().info(
                f"NTRIP connected to {self.args.ntrip_host}:{self.args.ntrip_port}/"
                f"{self.args.ntrip_mountpoint.lstrip('/')}"
            )
            sent_gga = self.send_latest_gga(sock, force=True)
            next_gga_time = time.monotonic() + (
                self.args.ntrip_gga_interval if sent_gga else 1.0
            )

            if initial_rtcm:
                self.write_rtcm(initial_rtcm)

            sock.settimeout(self.args.ntrip_read_timeout)
            while not self.stop_event.is_set() and rclpy.ok():
                now = time.monotonic()
                if now >= next_gga_time:
                    sent_gga = self.send_latest_gga(sock)
                    next_gga_time = now + (self.args.ntrip_gga_interval if sent_gga else 1.0)
                try:
                    data = sock.recv(4096)
                except socket.timeout:
                    continue
                if not data:
                    raise RuntimeError("caster closed connection")
                self.write_rtcm(data)

    def write_rtcm(self, data: bytes) -> None:
        with self.serial_write_lock:
            self.serial_port.write(data)


@dataclass
class GgaFix:
    stamp_text: str
    latitude_deg: float
    longitude_deg: float
    fix_quality: int
    satellites: int
    hdop: Optional[float]
    altitude_m: Optional[float]
    geoid_separation_m: Optional[float]
    differential_age_s: Optional[float]


@dataclass
class GstStdDev:
    latitude_m: float
    longitude_m: float
    altitude_m: float


def split_nmea(line: str) -> Optional[Tuple[str, List[str]]]:
    line = line.strip()
    if not line.startswith("$"):
        return None
    body = line[1:]
    checksum_text = None
    if "*" in body:
        body, checksum_text = body.split("*", 1)
        checksum = 0
        for char in body:
            checksum ^= ord(char)
        try:
            expected = int(checksum_text[:2], 16)
        except ValueError:
            return None
        if checksum != expected:
            return None

    fields = body.split(",")
    if not fields or len(fields[0]) < 3:
        return None
    return fields[0][-3:], fields


def parse_float(value: str) -> Optional[float]:
    if value == "":
        return None
    try:
        return float(value)
    except ValueError:
        return None


def parse_int(value: str) -> int:
    try:
        return int(value)
    except ValueError:
        return 0


def parse_lat_lon(value: str, hemisphere: str) -> Optional[float]:
    raw = parse_float(value)
    if raw is None:
        return None
    degrees = int(raw // 100)
    minutes = raw - degrees * 100
    decimal = degrees + minutes / 60.0
    if hemisphere in ("S", "W"):
        decimal = -decimal
    return decimal


def parse_gga(fields: List[str]) -> Optional[GgaFix]:
    if len(fields) < 10:
        return None
    latitude = parse_lat_lon(fields[2], fields[3])
    longitude = parse_lat_lon(fields[4], fields[5])
    if latitude is None or longitude is None:
        return None
    return GgaFix(
        stamp_text=fields[1],
        latitude_deg=latitude,
        longitude_deg=longitude,
        fix_quality=parse_int(fields[6]),
        satellites=parse_int(fields[7]),
        hdop=parse_float(fields[8]),
        altitude_m=parse_float(fields[9]),
        geoid_separation_m=parse_float(fields[11]) if len(fields) > 11 else None,
        differential_age_s=parse_float(fields[13]) if len(fields) > 13 else None,
    )


def parse_gst(fields: List[str]) -> Optional[GstStdDev]:
    if len(fields) < 9:
        return None
    latitude_std = parse_float(fields[6])
    longitude_std = parse_float(fields[7])
    altitude_std = parse_float(fields[8])
    if latitude_std is None or longitude_std is None or altitude_std is None:
        return None
    return GstStdDev(latitude_m=latitude_std, longitude_m=longitude_std, altitude_m=altitude_std)


def fix_quality_to_status(fix_quality: int) -> int:
    if fix_quality <= 0:
        return NavSatStatus.STATUS_NO_FIX
    if fix_quality in (2, 4, 5):
        return NavSatStatus.STATUS_GBAS_FIX
    return NavSatStatus.STATUS_FIX


def fix_quality_label(fix_quality: int) -> str:
    labels: Dict[int, str] = {0: "invalid", 1: "single", 2: "dgps", 4: "rtk_fixed", 5: "rtk_float"}
    return labels.get(fix_quality, f"quality_{fix_quality}")


def covariance_from_quality(fix_quality: int, hdop: Optional[float], gst: Optional[GstStdDev]) -> Tuple[List[float], int]:
    if gst is not None:
        return (
            [
                gst.latitude_m**2,
                0.0,
                0.0,
                0.0,
                gst.longitude_m**2,
                0.0,
                0.0,
                0.0,
                gst.altitude_m**2,
            ],
            NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN,
        )

    if fix_quality <= 0:
        return ([0.0] * 9, NavSatFix.COVARIANCE_TYPE_UNKNOWN)

    if fix_quality == 4:
        horizontal_std_m, vertical_std_m = 0.02, 0.04
    elif fix_quality == 5:
        horizontal_std_m, vertical_std_m = 0.25, 0.50
    elif fix_quality == 2:
        horizontal_std_m, vertical_std_m = 0.40, 0.80
    else:
        horizontal_std_m, vertical_std_m = 1.50, 2.50

    if hdop is not None and fix_quality not in (4, 5):
        horizontal_std_m *= max(hdop, 1.0)

    return (
        [
            horizontal_std_m**2,
            0.0,
            0.0,
            0.0,
            horizontal_std_m**2,
            0.0,
            0.0,
            0.0,
            vertical_std_m**2,
        ],
        NavSatFix.COVARIANCE_TYPE_APPROXIMATED,
    )


class RtkRosPublisher(Node):
    def __init__(self, args: argparse.Namespace):
        super().__init__("rtk_ros_publisher")
        self.args = args
        self.fix_pub = self.create_publisher(NavSatFix, args.fix_topic, args.qos_depth)
        self.latest_gst: Optional[GstStdDev] = None
        self.ntrip_client: Optional[NtripClient] = None

    def set_ntrip_client(self, ntrip_client: Optional[NtripClient]) -> None:
        self.ntrip_client = ntrip_client

    def handle_sentence(self, sentence_type: str, fields: List[str], raw_line: str) -> None:
        if sentence_type == "GGA":
            if self.ntrip_client is not None:
                self.ntrip_client.update_gga(raw_line)
            fix = parse_gga(fields)
            if fix is not None:
                self.publish_fix(fix)
        elif sentence_type == "GST":
            gst = parse_gst(fields)
            if gst is not None:
                self.latest_gst = gst

    def publish_fix(self, fix: GgaFix) -> None:
        stamp = self.get_clock().now().to_msg()

        msg = NavSatFix()
        msg.header.stamp = stamp
        msg.header.frame_id = self.args.frame_id
        msg.status.status = fix_quality_to_status(fix.fix_quality)
        msg.status.service = (
            NavSatStatus.SERVICE_GPS
            | NavSatStatus.SERVICE_GLONASS
            | NavSatStatus.SERVICE_COMPASS
            | NavSatStatus.SERVICE_GALILEO
        )
        msg.latitude = fix.latitude_deg
        msg.longitude = fix.longitude_deg
        msg.altitude = fix.altitude_m if fix.altitude_m is not None else math.nan
        covariance, covariance_type = covariance_from_quality(
            fix.fix_quality, fix.hdop, self.latest_gst
        )
        msg.position_covariance = covariance
        msg.position_covariance_type = covariance_type
        self.fix_pub.publish(msg)

        self.get_logger().info(
            f"NavSatFix status={fix_quality_label(fix.fix_quality)} "
            f"lat={fix.latitude_deg:.8f} lon={fix.longitude_deg:.8f} "
            f"satellites={fix.satellites} alt_m={msg.altitude:.3f}"
        )


def parse_args(argv: List[str]) -> Tuple[argparse.Namespace, List[str]]:
    parser = argparse.ArgumentParser(
        description="Read WTRTK-960H NMEA over USB serial and publish ROS 2 topics."
    )
    parser.add_argument("--port", default="/dev/ttyACM0", help="USB serial device.")
    parser.add_argument("--baudrate", type=int, default=115200, help="Serial baudrate.")
    parser.add_argument("--timeout", type=float, default=0.1, help="Serial read timeout in seconds.")
    parser.add_argument("--frame-id", default="rtk", help="ROS frame_id for GNSS messages.")
    parser.add_argument("--fix-topic", default="/rtk/fix", help="sensor_msgs/NavSatFix topic.")
    parser.add_argument("--qos-depth", type=int, default=20, help="Publisher queue depth.")
    parser.add_argument("--ntrip-host", default="", help="NTRIP caster host. Empty disables NTRIP.")
    parser.add_argument("--ntrip-port", type=int, default=2101, help="NTRIP caster port.")
    parser.add_argument("--ntrip-mountpoint", default="", help="NTRIP mountpoint.")
    parser.add_argument("--ntrip-user", default="", help="NTRIP username.")
    parser.add_argument("--ntrip-password", default="", help="NTRIP password.")
    parser.add_argument(
        "--ntrip-password-env",
        default="",
        help="Read NTRIP password from this environment variable instead of the command line.",
    )
    parser.add_argument("--ntrip-tls", action="store_true", help="Use TLS for the NTRIP connection.")
    parser.add_argument(
        "--ntrip-gga",
        default="",
        help="Optional fixed GGA sentence to send to the caster until live GGA is available.",
    )
    parser.add_argument(
        "--ntrip-gga-interval",
        type=float,
        default=10.0,
        help="Seconds between GGA position updates sent to the caster.",
    )
    parser.add_argument(
        "--ntrip-reconnect-sec",
        type=float,
        default=5.0,
        help="Seconds to wait before reconnecting to the NTRIP caster.",
    )
    parser.add_argument(
        "--ntrip-connect-timeout",
        type=float,
        default=10.0,
        help="NTRIP TCP connect timeout in seconds.",
    )
    parser.add_argument(
        "--ntrip-response-timeout",
        type=float,
        default=10.0,
        help="Seconds to wait for the NTRIP caster response header after connecting.",
    )
    parser.add_argument(
        "--ntrip-read-timeout",
        type=float,
        default=1.0,
        help="RTCM socket read timeout after the NTRIP stream is connected.",
    )
    return parser.parse_known_args(argv)


def main(argv: Optional[List[str]] = None) -> int:
    args, ros_args = parse_args(sys.argv[1:] if argv is None else argv)
    rclpy.init(args=[sys.argv[0]] + ros_args)
    node = RtkRosPublisher(args)
    ntrip_client = None

    try:
        with serial.Serial(args.port, args.baudrate, timeout=args.timeout) as ser:
            node.get_logger().info(
                f"Reading WTRTK-960H NMEA from {args.port} at {args.baudrate} baud"
            )
            if args.ntrip_host:
                if not args.ntrip_mountpoint:
                    node.get_logger().error("--ntrip-mountpoint is required when --ntrip-host is set")
                    return 2
                if args.ntrip_password_env and args.ntrip_password_env not in os.environ:
                    node.get_logger().error(
                        f"--ntrip-password-env is set to '{args.ntrip_password_env}', "
                        "but that environment variable does not exist"
                    )
                    return 2
                serial_write_lock = threading.Lock()
                ntrip_client = NtripClient(node, args, ser, serial_write_lock)
                node.set_ntrip_client(ntrip_client)
                ntrip_client.start()
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.0)
                raw = ser.readline()
                if not raw:
                    continue
                line = raw.decode("ascii", errors="replace").strip()
                node.get_logger().info(f"received: {line}")
                parsed = split_nmea(line)
                if parsed is None:
                    node.get_logger().debug(f"Ignoring invalid NMEA: {line}")
                    continue
                sentence_type, fields = parsed
                node.handle_sentence(sentence_type, fields, line)
    except serial.SerialException as exc:
        node.get_logger().error(f"Serial error on {args.port}: {exc}")
        return 1
    except KeyboardInterrupt:
        pass
    finally:
        if ntrip_client is not None:
            ntrip_client.stop()
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
