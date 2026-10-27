"""Tests for the front-camera frame logic. Stdlib only.

    python tools/tests/test_camera_frames.py
"""

from __future__ import annotations

import array
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from camera_frames import FramePacer, jpeg_bytes  # noqa: E402

JPEG = b"\xff\xd8\xff\xe0" + bytes(range(256)) + b"\xff\xd9"


def signed(b: bytes):
    return [v - 256 if v > 127 else v for v in b]


class TestJpegBytes(unittest.TestCase):
    def test_bytes_pass_through(self):
        self.assertEqual(jpeg_bytes(JPEG), JPEG)

    def test_signed_int_list(self):
        # int8[]: 0xFF arrives as -1. A naive bytes(list) raises on it.
        self.assertEqual(jpeg_bytes(signed(JPEG)), JPEG)

    def test_signed_array(self):
        self.assertEqual(jpeg_bytes(array.array("b", signed(JPEG))), JPEG)

    def test_not_a_jpeg(self):
        self.assertIsNone(jpeg_bytes(b"\x89PNG\r\n\x1a\n"))
        self.assertIsNone(jpeg_bytes(b""))
        self.assertIsNone(jpeg_bytes([]))
        self.assertIsNone(jpeg_bytes(None))


class TestPacer(unittest.TestCase):
    def test_first_request_due_immediately(self):
        self.assertTrue(FramePacer(fps=5).due(0.0))

    def test_one_in_flight(self):
        p = FramePacer(fps=5, timeout_s=1.0)
        p.sent(0.0, 1)
        self.assertFalse(p.due(0.5))              # still waiting for reply 1

    def test_paced_to_fps(self):
        p = FramePacer(fps=5)
        p.sent(0.0, 1)
        self.assertTrue(p.got(1))
        self.assertFalse(p.due(0.1))
        self.assertTrue(p.due(0.2))

    def test_lost_reply_times_out(self):
        p = FramePacer(fps=5, timeout_s=1.0)
        p.sent(0.0, 1)
        self.assertTrue(p.due(1.1))
        self.assertEqual(p.timeouts, 1)

    def test_stale_reply_ignored(self):
        p = FramePacer(fps=5, timeout_s=1.0)
        p.sent(0.0, 1)
        p.due(1.1)                                # gave up on 1
        p.sent(1.1, 2)
        self.assertFalse(p.got(1))                # late reply to 1
        self.assertEqual(p.in_flight, 2)
        self.assertTrue(p.got(2))
        self.assertEqual(p.frames, 1)

    def test_bad_frame_counted(self):
        p = FramePacer()
        p.sent(0.0, 7)
        p.got(7, ok=False)
        self.assertEqual((p.frames, p.bad), (0, 1))

    def test_fps_must_be_positive(self):
        with self.assertRaises(ValueError):
            FramePacer(fps=0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
