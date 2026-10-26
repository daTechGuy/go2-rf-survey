"""Point-cloud helpers shared by bag_to_map.py (offline) and live_map.py (on
the robot). numpy only -- no matplotlib, no ROS -- so a live node can import
it without dragging in plotting.

* cloud_to_xyz   PointCloud2 payload -> (N, 3) array, from the field table
                 (not sensor_msgs_py, whose read_points changed between distros)
* xyz_to_payload (N, 3) array -> PointCloud2 fields/point_step/bytes
* voxel_downsample / VoxelAccumulator   bounded-memory map accumulation
"""

from __future__ import annotations

import numpy as np

# sensor_msgs/PointField datatype -> numpy scalar code
_POINTFIELD = {1: "i1", 2: "u1", 3: "i2", 4: "u2", 5: "i4", 6: "u4", 7: "f4", 8: "f8"}
FLOAT32 = 7

# Voxel keys pack three 21-bit signed cell indices into one int64, so
# deduplication is a fast 1-D np.unique. +/-2^20 cells at 5 cm is +/-52 km.
_OFF = 1 << 20
_MASK = (1 << 21) - 1


def cloud_to_xyz(fields, point_step, data, is_bigendian=False,
                 width=None, height=1, row_step=None) -> np.ndarray:
    """PointCloud2 payload -> (N, 3) float64 array of finite x, y, z.

    ``fields`` is a list of (name, offset, datatype). Handles organized clouds
    whose rows are padded (row_step > width * point_step) by unpacking row by
    row; the common unorganized case is a single frombuffer.
    """
    byname = {name: (off, dt) for name, off, dt in fields}
    missing = {"x", "y", "z"} - set(byname)
    if missing:
        raise ValueError(f"cloud has no {sorted(missing)} field(s)")
    order = ">" if is_bigendian else "<"
    dtype = np.dtype({
        "names": ["x", "y", "z"],
        "formats": [order + _POINTFIELD[byname[k][1]] for k in "xyz"],
        "offsets": [byname[k][0] for k in "xyz"],
        "itemsize": point_step,
    })
    buf = memoryview(data).cast("B") if not isinstance(data, (bytes, bytearray)) else data

    if width is not None and row_step is not None and height > 1 \
            and row_step != width * point_step:
        rows = [np.frombuffer(buf, dtype=dtype, count=width, offset=r * row_step)
                for r in range(height)]
        arr = np.concatenate(rows)
    else:
        arr = np.frombuffer(buf, dtype=dtype, count=len(buf) // point_step)

    xyz = np.column_stack([arr["x"], arr["y"], arr["z"]]).astype(np.float64)
    return xyz[np.isfinite(xyz).all(axis=1)]


def xyz_to_payload(pts: np.ndarray):
    """(N, 3) -> (fields, point_step, data) for an unorganized float32 x/y/z
    PointCloud2. ``fields`` is [(name, offset, datatype)]."""
    data = np.ascontiguousarray(pts, dtype="<f4").tobytes()
    return [("x", 0, FLOAT32), ("y", 4, FLOAT32), ("z", 8, FLOAT32)], 12, data


def _voxel_keys(pts: np.ndarray, voxel: float) -> np.ndarray:
    k = np.floor(pts / voxel).astype(np.int64) + _OFF
    k = np.clip(k, 0, _MASK)
    return (k[:, 0] << 42) | (k[:, 1] << 21) | k[:, 2]


def voxel_downsample(pts: np.ndarray, voxel: float) -> np.ndarray:
    """Keep one point per voxel (the first seen). Real coordinates, not centres."""
    if len(pts) == 0:
        return pts
    _, idx = np.unique(_voxel_keys(pts, voxel), return_index=True)
    return pts[idx]


class VoxelAccumulator:
    """Streams scans in and keeps memory bounded: raw points are buffered and
    folded into the deduplicated set whenever the buffer gets large. A few
    minutes at 15 Hz is tens of millions of raw points; the room itself is a
    few hundred thousand voxels."""

    def __init__(self, voxel: float = 0.05, flush_at: int = 2_000_000):
        self.voxel = voxel
        self.flush_at = flush_at
        self._kept = np.empty((0, 3))
        self._pending: list = []
        self._pending_n = 0
        self.raw_points = 0
        self.scans = 0

    def add(self, pts: np.ndarray) -> None:
        self.scans += 1
        self.raw_points += len(pts)
        self._pending.append(pts)
        self._pending_n += len(pts)
        if self._pending_n >= self.flush_at:
            self._flush()

    def _flush(self) -> None:
        if not self._pending:
            return
        merged = np.concatenate([self._kept, *self._pending])
        self._kept = voxel_downsample(merged, self.voxel)
        self._pending, self._pending_n = [], 0

    def result(self) -> np.ndarray:
        self._flush()
        return self._kept

    def reset(self) -> None:
        self.__init__(self.voxel, self.flush_at)
