#!/usr/bin/env python3
"""Front-camera frames from the Go2's ``videohub`` service. Pure logic.

The camera is a Unitree request/response service like sport or
obstacles_avoid: send api 1001 (GetImageSample, no parameter) on
/api/videohub/request, and the reply's ``binary`` field on
/api/videohub/response is one JPEG. IDs from unitree_sdk2
include/unitree/robot/go2/video/video_api.hpp and unitree_sdk2_python's
VideoClient (``_CallBinary(1001, [])`` -> ``response.binary``).

This avoids what dancher00/go2-nav2-wifi needs for the same picture (its
robot_front_camera_bridge.py): unitree_sdk2_python, or a C++ helper built
against unitree_sdk2 on the robot, then OpenCV to decode. Here the JPEG goes
straight into a sensor_msgs/CompressedImage, which Foxglove shows as is.

Two traps handled here, both testable without the robot:
  * ``binary`` is ``int8[]`` in unitree_api/Response (signed), so it arrives
    as array('b') or a list of -128..127 -- not bytes.
  * one request in flight at a time, paced to ``fps``; a reply that never
    comes must not stall the stream (``timeout_s``).
Stdlib only.
"""

from __future__ import annotations

VIDEO_SERVICE = "videohub"
GET_IMAGE_SAMPLE = 1001
JPEG_SOI = b"\xff\xd8"


def jpeg_bytes(binary):
    """The reply's ``binary`` as JPEG bytes, or None if it isn't a JPEG."""
    if binary is None:
        return None
    if isinstance(binary, (bytes, bytearray)):
        b = bytes(binary)
    elif hasattr(binary, "tobytes"):                  # array('b'), numpy
        b = binary.tobytes()
    else:                                             # list of signed ints
        b = bytes(int(v) & 0xFF for v in binary)
    if len(b) < 4 or b[:2] != JPEG_SOI:
        return None
    return b


class FramePacer:
    """When to ask for the next frame. Caller passes time in (seconds)."""

    def __init__(self, fps: float = 5.0, timeout_s: float = 1.0) -> None:
        if fps <= 0:
            raise ValueError("fps must be > 0")
        self.period = 1.0 / fps
        self.timeout_s = timeout_s
        self.in_flight = None          # request id awaiting a reply
        self.sent_at = 0.0
        self.next_at = 0.0
        self.frames = self.timeouts = self.bad = 0

    def due(self, now: float) -> bool:
        if self.in_flight is not None:
            if now - self.sent_at < self.timeout_s:
                return False
            self.in_flight = None      # gave up on it
            self.timeouts += 1
        return now >= self.next_at

    def sent(self, now: float, request_id) -> None:
        self.in_flight, self.sent_at = request_id, now
        self.next_at = now + self.period

    def got(self, request_id, ok: bool = True) -> bool:
        """True if this reply answers the request in flight (else ignore it)."""
        if request_id != self.in_flight:
            return False
        self.in_flight = None
        if ok:
            self.frames += 1
        else:
            self.bad += 1
        return True
