#!/usr/bin/python3
"""Publish RTK GPS NMEA data as ROS 2 NavSatFix with optional NTRIP corrections."""

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


# ── NMEA data types ───────────────────────────────────────────────────────────

@dataclass
class GgaFix:
    latitude_deg: float
    longitude_deg: float
    fix_quality: int
    satellites: int
    hdop: Optional[float]
    altitude_m: Optional[float]
    geoid_sep_m: Optional[float]


@dataclass
class GstStdDev:
    latitude_m: float
    longitude_m: float
    altitude_m: float


# ── NMEA parsing ──────────────────────────────────────────────────────────────

def _parse_float(v: str) -> Optional[float]:
    try:
        return float(v) if v else None
    except ValueError:
        return None


def _parse_int(v: str) -> int:
    try:
        return int(v)
    except ValueError:
        return 0


def _parse_lat_lon(value: str, hemi: str) -> Optional[float]:
    raw = _parse_float(value)
    if raw is None:
        return None
    deg = int(raw // 100)
    dec = deg + (raw - deg * 100) / 60.0
    return -dec if hemi in ("S", "W") else dec


def _checksum_ok(line: str) -> bool:
    if "*" not in line:
        return True
    body, cs = line[1:].split("*", 1)
    calc = 0
    for ch in body:
        calc ^= ord(ch)
    try:
        return calc == int(cs[:2], 16)
    except ValueError:
        return False


def parse_sentence(line: str) -> Optional[Tuple[str, List[str]]]:
    """Return (sentence_type, fields) for a valid NMEA line, or None."""
    line = line.strip()
    if not line.startswith("$") or not _checksum_ok(line):
        return None
    fields = line[1:].split("*")[0].split(",")
    if not fields or len(fields[0]) < 3:
        return None
    return fields[0][-3:], fields  # last 3 chars: "GGA", "GST", etc.


def parse_gga(fields: List[str]) -> Optional[GgaFix]:
    if len(fields) < 10:
        return None
    lat = _parse_lat_lon(fields[2], fields[3])
    lon = _parse_lat_lon(fields[4], fields[5])
    if lat is None or lon is None:
        return None
    return GgaFix(
        latitude_deg=lat,
        longitude_deg=lon,
        fix_quality=_parse_int(fields[6]),
        satellites=_parse_int(fields[7]),
        hdop=_parse_float(fields[8]),
        altitude_m=_parse_float(fields[9]),
        geoid_sep_m=_parse_float(fields[11]) if len(fields) > 11 else None,
    )


def parse_gst(fields: List[str]) -> Optional[GstStdDev]:
    if len(fields) < 9:
        return None
    lat_s = _parse_float(fields[6])
    lon_s = _parse_float(fields[7])
    alt_s = _parse_float(fields[8])
    if lat_s is None or lon_s is None or alt_s is None:
        return None
    return GstStdDev(latitude_m=lat_s, longitude_m=lon_s, altitude_m=alt_s)


# ── Fix quality helpers ───────────────────────────────────────────────────────

_FIX_LABELS: Dict[int, str] = {
    0: "invalid", 1: "single", 2: "dgps", 4: "rtk_fixed", 5: "rtk_float"
}


def fix_label(q: int) -> str:
    return _FIX_LABELS.get(q, f"quality_{q}")


def fix_status(q: int) -> int:
    if q <= 0:
        return NavSatStatus.STATUS_NO_FIX
    if q in (2, 4, 5):
        return NavSatStatus.STATUS_GBAS_FIX
    return NavSatStatus.STATUS_FIX


def position_covariance(
    q: int, hdop: Optional[float], gst: Optional[GstStdDev]
) -> Tuple[List[float], int]:
    if gst is not None:
        cov = [gst.latitude_m**2, 0, 0, 0, gst.longitude_m**2, 0, 0, 0, gst.altitude_m**2]
        return cov, NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
    if q <= 0:
        return [0.0] * 9, NavSatFix.COVARIANCE_TYPE_UNKNOWN
    if q == 4:
        h, v = 0.02, 0.04
    elif q == 5:
        h, v = 0.25, 0.50
    elif q == 2:
        h, v = 0.40, 0.80
    else:
        h, v = 1.50, 2.50
    if hdop is not None and q not in (4, 5):
        h *= max(hdop, 1.0)
    return [h**2, 0, 0, 0, h**2, 0, 0, 0, v**2], NavSatFix.COVARIANCE_TYPE_APPROXIMATED


# ── NTRIP client (background thread) ─────────────────────────────────────────

class NtripClient:
    def __init__(
        self,
        node: Node,
        args: argparse.Namespace,
        ser: "serial.Serial",
        write_lock: threading.Lock,
    ) -> None:
        self.node = node
        self.args = args
        self.ser = ser
        self.write_lock = write_lock
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._gga: Optional[str] = None
        self._gga_lock = threading.Lock()

    def update_gga(self, raw_line: str) -> None:
        with self._gga_lock:
            self._gga = raw_line

    def start(self) -> None:
        self._thread = threading.Thread(target=self._run, name="ntrip", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=3.0)

    def _run(self) -> None:
        while not self._stop.is_set() and rclpy.ok():
            try:
                self._connect_and_stream()
            except Exception as exc:
                self.node.get_logger().warning(f"NTRIP disconnected: {exc}")
            self._stop.wait(self.args.ntrip_reconnect_sec)

    def _open_socket(self) -> socket.socket:
        sock = socket.create_connection(
            (self.args.ntrip_host, self.args.ntrip_port),
            timeout=self.args.ntrip_connect_timeout,
        )
        if self.args.ntrip_tls:
            ctx = ssl.create_default_context()
            sock = ctx.wrap_socket(sock, server_hostname=self.args.ntrip_host)
        return sock

    def _http_request(self) -> bytes:
        mp = self.args.ntrip_mountpoint.lstrip("/")
        password = self.args.ntrip_password
        if self.args.ntrip_password_env:
            password = os.environ.get(self.args.ntrip_password_env, password)
        token = base64.b64encode(f"{self.args.ntrip_user}:{password}".encode()).decode()
        return (
            f"GET /{mp} HTTP/1.0\r\n"
            f"Host: {self.args.ntrip_host}:{self.args.ntrip_port}\r\n"
            f"User-Agent: NTRIP rtk_ros_publisher/2.0\r\n"
            f"Ntrip-Version: Ntrip/2.0\r\n"
            f"Accept: */*\r\n"
            f"Authorization: Basic {token}\r\n"
            f"Connection: close\r\n\r\n"
        ).encode("ascii")

    def _read_header(self, sock: socket.socket) -> Tuple[str, bytes]:
        data = b""
        sock.settimeout(self.args.ntrip_response_timeout)
        while b"\r\n\r\n" not in data:
            chunk = sock.recv(4096)
            if not chunk:
                raise RuntimeError("caster closed before sending response header")
            data += chunk
            if len(data) > 16384:
                raise RuntimeError("caster response header too large")
        header, rest = data.split(b"\r\n\r\n", 1)
        return header.decode("iso-8859-1", errors="replace"), rest

    def _send_gga(self, sock: socket.socket) -> bool:
        with self._gga_lock:
            gga = self._gga
        if gga is None and self.args.ntrip_gga:
            gga = self.args.ntrip_gga
        if gga is None:
            return False
        sock.sendall((gga.strip() + "\r\n").encode("ascii", errors="ignore"))
        return True

    def _connect_and_stream(self) -> None:
        with self._open_socket() as sock:
            sock.sendall(self._http_request())
            header, initial_rtcm = self._read_header(sock)
            first_line = header.splitlines()[0] if header else ""
            if not (first_line.startswith("ICY 200") or " 200 " in first_line):
                raise RuntimeError(first_line or "unexpected caster response")

            self.node.get_logger().info(
                f"NTRIP connected: {self.args.ntrip_host}:{self.args.ntrip_port}"
                f"/{self.args.ntrip_mountpoint.lstrip('/')}"
            )
            sent = self._send_gga(sock)
            if not sent:
                self.node.get_logger().warning("NTRIP connected — waiting for first GGA to send position")
            next_gga = time.monotonic() + (self.args.ntrip_gga_interval if sent else 1.0)

            if initial_rtcm:
                self._write_rtcm(initial_rtcm)

            sock.settimeout(self.args.ntrip_read_timeout)
            while not self._stop.is_set() and rclpy.ok():
                now = time.monotonic()
                if now >= next_gga:
                    sent = self._send_gga(sock)
                    next_gga = now + (self.args.ntrip_gga_interval if sent else 1.0)
                try:
                    data = sock.recv(4096)
                except socket.timeout:
                    continue
                if not data:
                    raise RuntimeError("caster closed connection")
                self._write_rtcm(data)

    def _write_rtcm(self, data: bytes) -> None:
        with self.write_lock:
            self.ser.write(data)


# ── ROS 2 node ────────────────────────────────────────────────────────────────

class RtkNode(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("rtk_ros_publisher")
        self.args = args
        self.fix_pub = self.create_publisher(NavSatFix, args.fix_topic, args.qos_depth)
        self._latest_gst: Optional[GstStdDev] = None
        self._ntrip: Optional[NtripClient] = None

    def set_ntrip(self, client: Optional[NtripClient]) -> None:
        self._ntrip = client

    def handle_line(self, line: str) -> None:
        # self.get_logger().info(line)
        parsed = parse_sentence(line)
        if parsed is None:
            self.get_logger().debug(f"skipping invalid/unknown: {line}")
            return
        sentence_type, fields = parsed
        if sentence_type == "GGA":
            if self._ntrip is not None:
                self._ntrip.update_gga(line)
            fix = parse_gga(fields)
            if fix is not None:
                self._publish(fix)
        elif sentence_type == "GST":
            gst = parse_gst(fields)
            if gst is not None:
                self._latest_gst = gst

    def _publish(self, fix: GgaFix) -> None:
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.args.frame_id
        msg.status.status = fix_status(fix.fix_quality)
        msg.status.service = (
            NavSatStatus.SERVICE_GPS
            | NavSatStatus.SERVICE_GLONASS
            | NavSatStatus.SERVICE_COMPASS
            | NavSatStatus.SERVICE_GALILEO
        )
        msg.latitude = fix.latitude_deg
        msg.longitude = fix.longitude_deg
        msg.altitude = fix.altitude_m if fix.altitude_m is not None else math.nan
        cov, cov_type = position_covariance(fix.fix_quality, fix.hdop, self._latest_gst)
        msg.position_covariance = cov
        msg.position_covariance_type = cov_type
        self.fix_pub.publish(msg)
        self.get_logger().info(
            f"fix={fix_label(fix.fix_quality)} "
            f"lat={fix.latitude_deg:.8f} lon={fix.longitude_deg:.8f} "
            f"sats={fix.satellites} alt={msg.altitude:.2f}m"
        )


# ── Serial read loop ──────────────────────────────────────────────────────────

def read_loop(node: RtkNode, ser: "serial.Serial") -> None:
    """Buffer bytes from serial and emit complete NMEA sentences to the node."""
    buf = bytearray()
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.0)

        waiting = ser.in_waiting
        if waiting:
            buf.extend(ser.read(waiting))

        # Extract all complete sentences ($....\r\n) from the buffer.
        while True:
            start = buf.find(b"$")
            if start == -1:
                buf.clear()
                break
            end = buf.find(b"\r\n", start)
            if end == -1:
                # Discard any garbage before the $ and wait for more data.
                if start > 0:
                    del buf[:start]
                break
            sentence_bytes = buf[start:end]
            del buf[:end + 2]
            try:
                node.handle_line(sentence_bytes.decode("ascii", errors="replace"))
            except Exception:
                pass

        if not waiting:
            time.sleep(0.005)


# ── Entry point ───────────────────────────────────────────────────────────────

def parse_args(argv: List[str]) -> Tuple[argparse.Namespace, List[str]]:
    p = argparse.ArgumentParser(description="RTK NMEA → ROS 2 NavSatFix publisher")
    p.add_argument("--port",      default=os.environ.get("RTK_SERIAL_PORT", "/dev/ttyUSB0"),
                   help="Serial device (env: RTK_SERIAL_PORT)")
    p.add_argument("--baudrate",  type=int, default=int(os.environ.get("RTK_BAUDRATE", "115200")),
                   help="Baud rate (env: RTK_BAUDRATE)")
    p.add_argument("--frame-id",  default=os.environ.get("RTK_FRAME_ID", "rtk"),
                   help="ROS frame_id (env: RTK_FRAME_ID)")
    p.add_argument("--fix-topic", default=os.environ.get("RTK_FIX_TOPIC", "/rtk/fix"),
                   help="NavSatFix topic (env: RTK_FIX_TOPIC)")
    p.add_argument("--qos-depth", type=int, default=20)
    p.add_argument("--init-cmd",  default="GPGGA 1\r\n",
                   help="Command sent to GPS on open to enable NMEA output.")
    # NTRIP
    p.add_argument("--ntrip-host",       default=os.environ.get("NTRIP_HOST", ""),
                   help="NTRIP caster host (env: NTRIP_HOST). Empty disables NTRIP.")
    p.add_argument("--ntrip-port",       type=int, default=int(os.environ.get("NTRIP_PORT", "2101")),
                   help="NTRIP caster port (env: NTRIP_PORT)")
    p.add_argument("--ntrip-mountpoint", default=os.environ.get("NTRIP_MOUNTPOINT", ""),
                   help="NTRIP mountpoint (env: NTRIP_MOUNTPOINT)")
    p.add_argument("--ntrip-user",       default=os.environ.get("NTRIP_USER", ""),
                   help="NTRIP username (env: NTRIP_USER)")
    p.add_argument("--ntrip-password",   default=os.environ.get("NTRIP_PASSWORD", ""),
                   help="NTRIP password (env: NTRIP_PASSWORD)")
    p.add_argument("--ntrip-password-env", default="",
                   help="Read NTRIP password from this env var instead of --ntrip-password.")
    p.add_argument("--ntrip-tls",        action="store_true",
                   default=os.environ.get("NTRIP_TLS", "").lower() == "true",
                   help="Use TLS for NTRIP (env: NTRIP_TLS=true)")
    p.add_argument("--ntrip-gga",        default="",
                   help="Fixed GGA sentence to send until live GGA is available.")
    p.add_argument("--ntrip-gga-interval",    type=float, default=10.0)
    p.add_argument("--ntrip-reconnect-sec",   type=float, default=5.0)
    p.add_argument("--ntrip-connect-timeout", type=float, default=10.0)
    p.add_argument("--ntrip-response-timeout",type=float, default=10.0)
    p.add_argument("--ntrip-read-timeout",    type=float, default=1.0)
    return p.parse_known_args(argv)


def main(argv: Optional[List[str]] = None) -> int:
    args, ros_args = parse_args(sys.argv[1:] if argv is None else argv)
    rclpy.init(args=[sys.argv[0]] + ros_args)
    node = RtkNode(args)
    ntrip = None
    try:
        with serial.Serial(args.port, args.baudrate, timeout=1) as ser:
            node.get_logger().info(f"Opened {args.port} at {args.baudrate} baud")

            if args.init_cmd:
                ser.write(args.init_cmd.encode("ascii"))
                node.get_logger().info(f"Sent init command: {args.init_cmd.strip()!r}")
                time.sleep(1.0)

            if args.ntrip_host:
                if not args.ntrip_mountpoint:
                    node.get_logger().error("--ntrip-mountpoint is required when --ntrip-host is set")
                    return 2
                write_lock = threading.Lock()
                ntrip = NtripClient(node, args, ser, write_lock)
                node.set_ntrip(ntrip)
                ntrip.start()

            read_loop(node, ser)

    except serial.SerialException as exc:
        node.get_logger().error(f"Serial error on {args.port}: {exc}")
        return 1
    except KeyboardInterrupt:
        pass
    finally:
        if ntrip is not None:
            ntrip.stop()
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
