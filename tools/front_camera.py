#!/usr/bin/env python3
"""Go2 front camera -> /go2/camera/compressed (sensor_msgs/CompressedImage, JPEG).

Runs in the container next to foxglove_bridge (started by viz_entry.sh), so
Foxglove shows what the robot sees beside the live map. Read-only towards
the robot: it only asks the camera service for pictures.

    python3 tools/front_camera.py                 # stream, 5 fps
    python3 tools/front_camera.py --fps 2         # lighter (e.g. over WiFi later)
    python3 tools/front_camera.py --once /surveys/camera_test.jpg
                                                  # one frame to a file, then exit

How it talks to the camera, and why not like go2-nav2-wifi: camera_frames.py.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage

sys.path.insert(0, str(Path(__file__).resolve().parent))
from camera_frames import GET_IMAGE_SAMPLE, VIDEO_SERVICE, FramePacer, jpeg_bytes  # noqa: E402

try:
    from unitree_api.msg import Request, Response
except ImportError as exc:  # pragma: no cover - environment
    raise SystemExit(f"unitree_api not found ({exc}) -- run inside go2-survey:humble") from exc

TOPIC = "/go2/camera/compressed"
FRAME = "front_camera"


class FrontCamera(Node):
    def __init__(self, fps: float, timeout_s: float, once: Path | None) -> None:
        super().__init__("go2_front_camera")
        self.pacer = FramePacer(fps=fps, timeout_s=timeout_s)
        self.once = once
        self.done = False
        self._rid = time.time_ns() % (1 << 40)
        self.req_pub = self.create_publisher(Request, f"/api/{VIDEO_SERVICE}/request", 10)
        self.create_subscription(Response, f"/api/{VIDEO_SERVICE}/response", self._on_response, 10)
        self.img_pub = self.create_publisher(CompressedImage, TOPIC, qos_profile_sensor_data)
        self.create_timer(0.02, self._tick)
        self.create_timer(10.0, self._report)
        self._last = (0, 0, 0)
        print(f"front camera: /api/{VIDEO_SERVICE} api {GET_IMAGE_SAMPLE} -> {TOPIC} "
              f"at <= {fps:g} fps", flush=True)

    def _now(self) -> float:
        return time.monotonic()

    def _tick(self) -> None:
        if self.done or not self.pacer.due(self._now()):
            return
        if self.req_pub.get_subscription_count() == 0:
            return                                   # camera service not discovered yet
        self._rid += 1
        req = Request()
        req.header.identity.id = self._rid
        req.header.identity.api_id = GET_IMAGE_SAMPLE
        self.req_pub.publish(req)
        self.pacer.sent(self._now(), self._rid)

    def _on_response(self, msg: Response) -> None:
        rid = int(msg.header.identity.id)
        code = int(msg.header.status.code)
        jpeg = jpeg_bytes(msg.binary) if code == 0 else None
        if not self.pacer.got(rid, ok=jpeg is not None):
            return
        if jpeg is None:
            return
        if self.once is not None:
            self.once.parent.mkdir(parents=True, exist_ok=True)
            self.once.write_bytes(jpeg)
            print(f"saved {len(jpeg)} bytes -> {self.once}", flush=True)
            self.done = True
            return
        out = CompressedImage()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = FRAME
        out.format = "jpeg"
        out.data = jpeg
        self.img_pub.publish(out)
        if self.pacer.frames == 1:
            print(f"first frame published ({len(jpeg)} bytes)", flush=True)

    def _report(self) -> None:
        p = self.pacer
        f, t, b = p.frames - self._last[0], p.timeouts - self._last[1], p.bad - self._last[2]
        self._last = (p.frames, p.timeouts, p.bad)
        if self.req_pub.get_subscription_count() == 0:
            print("WARNING: no videohub service seen on /api/videohub/request", flush=True)
        elif f == 0:
            print(f"WARNING: no frames in 10 s (timeouts {t}, bad replies {b})", flush=True)
        else:
            print(f"   {f / 10.0:.1f} fps (timeouts {t}, bad {b})", flush=True)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--fps", type=float, default=5.0)
    ap.add_argument("--timeout", type=float, default=1.0, help="s to wait for one reply")
    ap.add_argument("--once", type=Path, default=None, help="save one JPEG here and exit")
    a = ap.parse_args()
    rclpy.init()
    node = FrontCamera(a.fps, a.timeout, a.once)
    try:
        deadline = time.monotonic() + 15.0
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
            if a.once is not None and time.monotonic() > deadline:
                seen = node.req_pub.get_subscription_count() > 0
                print("no frame in 15 s: " + (
                    f"videohub is there but sent no good frame ({node.pacer.bad} bad "
                    f"replies, {node.pacer.timeouts} timeouts)"
                    if seen else "no videohub service on /api/videohub/request"), flush=True)
                return 1
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
