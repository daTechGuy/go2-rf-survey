"""Tests for decoding the Mid-360's point packets. Stdlib only.

The packet layout is the one decoded on the robot (2026-11-12): 1380 bytes,
36-byte header, 96 points of int32 x/y/z mm + reflectivity + tag.

    python tools/tests/test_mid360_udp.py
"""

from __future__ import annotations

import struct
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from mid360_udp import HEADER, POINT, decode, parse_header  # noqa: E402


def packet(points, dtype=1):
    body = b"".join(struct.pack("<iiiBB", x, y, z, 50, 0) for x, y, z in points)
    length = HEADER + len(body)
    head = struct.pack("<BHHHHBBB", 0, length, 0, len(points), 7, 1, dtype, 0)
    head += b"\x00" * (HEADER - len(head) - 8) + struct.pack("<Q", 123456789)
    return head + body


class TestPackets(unittest.TestCase):
    def test_real_size(self):
        p = packet([(1000, 2000, -300)] * 96)
        self.assertEqual(len(p), 1380)                     # as seen on the robot
        self.assertEqual(parse_header(p), (96, 1))

    def test_decode_mm_to_m(self):
        pts = decode(packet([(1500, -2000, 250), (0, 0, 0), (-10, 20, 30)]))
        self.assertEqual(pts, [(1.5, -2.0, 0.25), (-0.01, 0.02, 0.03)])   # no-return dropped

    def test_keep_every_nth(self):
        p = packet([(i + 1, 0, 0) for i in range(96)])
        self.assertEqual(len(decode(p, keep=4)), 24)

    def test_other_packets_ignored(self):
        self.assertEqual(decode(packet([(1, 2, 3)], dtype=2)), [])     # not the 14-byte format
        self.assertIsNone(parse_header(b"\x00" * 10))
        bad = bytearray(packet([(1, 2, 3)]))
        struct.pack_into("<H", bad, 1, 999)                           # length field disagrees
        self.assertIsNone(parse_header(bytes(bad)))


if __name__ == "__main__":
    unittest.main(verbosity=2)
