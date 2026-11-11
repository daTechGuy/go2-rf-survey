#!/usr/bin/env python3
"""Receive the Mid-360's point stream directly and publish /livox/lidar.

    (started by: bash tools/go2loop.sh mid360-up | mid360-calibrate)

On this Go2 (2026-11-12) the Mid-360 streams points to the Jetson on its
own -- 192.168.123.20:56300 -> 192.168.123.18:56301, ~2,100 packets/s of 96
points -- but livox_ros_driver2 (1.2.6, SDK2) never completes its handshake
with Unitree's firmware: it opens only the discovery port, never 56301, so
the points arrive with nobody listening. This listens on 56301 itself.

Packet (Livox Ethernet protocol, as decoded on the robot): 36-byte header --
version u8, length u16, time_interval u16, dot_num u16, udp_cnt u16,
frame_cnt u8, data_type u8, time_type u8, 12 reserved, crc32 u32,
timestamp u64 -- then dot_num points. data_type 1 = int32 x, y, z in mm,
u8 reflectivity, u8 tag (14 bytes). All-zero points are no-returns.

Publishes PointCloud2 (x, y, z float32, metres, frame livox_frame) every
0.1 s, every ``keep``-th point. The IMU stream isn't sent unasked, and
nothing here needs it.
"""

from __future__ import annotations

import socket
import struct
import sys
import time

HEADER = 36
POINT = 14
HEADER_FMT = "<BHHHHBBB"


def parse_header(d: bytes):
    """(dot_num, data_type) or None for something that isn't a point packet."""
    if len(d) < HEADER:
        return None
    _ver, length, _ti, dot, _cnt, _frame, dtype, _ttype = struct.unpack_from(HEADER_FMT, d, 0)
    if length != len(d):
        return None
    return dot, dtype


def decode(d: bytes, keep: int = 1):
    """Points (x, y, z) in metres from one packet, pure Python (tests, and a
    fallback without numpy). data_type 1 only; no-returns dropped."""
    h = parse_header(d)
    if h is None or h[1] != 1 or len(d) < HEADER + POINT * h[0]:
        return []
    out = []
    for k in range(0, h[0], keep):
        x, y, z = struct.unpack_from("<iii", d, HEADER + POINT * k)
        if x or y or z:
            out.append((x / 1000.0, y / 1000.0, z / 1000.0))
    return out


def main() -> int:
    import numpy as np
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import PointCloud2, PointField
    from std_msgs.msg import Header

    keep = int(sys.argv[1]) if len(sys.argv) > 1 else 4
    port = int(sys.argv[2]) if len(sys.argv) > 2 else 56301
    dt = np.dtype([("x", "<i4"), ("y", "<i4"), ("z", "<i4"), ("r", "u1"), ("t", "u1")])

    rclpy.init()
    node = Node("mid360_udp")
    pub = node.create_publisher(PointCloud2, "/livox/lidar", qos_profile_sensor_data)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 8 << 20)   # capped by net.core.rmem_max
    sock.bind(("0.0.0.0", port))
    sock.settimeout(0.05)
    print(f"mid360_udp: listening on :{port} (receive buffer "
          f"{sock.getsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF)} bytes), "
          f"every {keep}th point -> /livox/lidar", flush=True)

    chunks, packets, frames, other, t_pub, t_rep = [], 0, 0, 0, time.monotonic(), time.monotonic()
    try:
        while rclpy.ok():
            try:
                d = sock.recv(2048)
                h = parse_header(d)
                if h is not None and h[1] == 1:
                    a = np.frombuffer(d, dtype=dt, count=h[0], offset=HEADER)[::keep]
                    a = a[(a["x"] != 0) | (a["y"] != 0) | (a["z"] != 0)]
                    chunks.append(np.stack([a["x"], a["y"], a["z"]], axis=1))
                    packets += 1
                else:
                    other += 1
            except socket.timeout:
                pass
            now = time.monotonic()
            if now - t_pub >= 0.1 and chunks:
                xyz = (np.concatenate(chunks) / 1000.0).astype(np.float32)
                chunks = []
                msg = PointCloud2(header=Header(stamp=node.get_clock().now().to_msg(),
                                                frame_id="livox_frame"))
                msg.height, msg.width = 1, len(xyz)
                msg.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                              for i, n in enumerate("xyz")]
                msg.is_bigendian, msg.point_step, msg.is_dense = False, 12, True
                msg.row_step = 12 * len(xyz)
                msg.data = xyz.tobytes()
                pub.publish(msg)
                frames += 1
                t_pub = now
            if now - t_rep >= 10.0:
                print(f"   {packets / (now - t_rep):.0f} packets/s, {frames / (now - t_rep):.1f} frames/s"
                      + (f", {other} other packets" if other else ""), flush=True)
                packets = frames = other = 0
                t_rep = now
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
