"""ROS 2 node joining WiFi scans to SLAM poses.

Two modes, both needed at different phases of the project:

* **continuous** -- scan whenever the robot has moved far enough since the last
  sample. This is the phase-5 mode: drive by teleop, get a heatmap, no autonomy
  required.
* **triggered** -- scan only on a ``~/scan_now`` service call. This is the
  phase-8 mode, called by the waypoint follower once the robot has come to rest.

Samples are timestamped at scan *completion*, not request (decision D4). While
stationary the two agree; they diverge the moment anyone scans on the move, and
a silently wrong timestamp is worse than a slow survey.

Rows are flushed to disk as they are produced. A survey run is 60-90 minutes of
battery and cannot be repeated cheaply, so a crash or a flat battery must cost
the last sample, not the whole run.
"""

from __future__ import annotations

import csv
import math
import threading
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

import tf2_ros
from std_msgs.msg import String
from std_srvs.srv import Trigger

from go2_survey_wifi.scanner import ScanError, scan

CSV_FIELDS = [
    "stamp",
    "sample_id",
    "x",
    "y",
    "z",
    "yaw",
    "bssid",
    "ssid",
    "freq",
    "band",
    "rssi",
]


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """Yaw about Z, in radians. Avoids a dependency on tf_transformations,
    which is packaged inconsistently across ROS distros."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class WifiSurveyNode(Node):
    def __init__(self) -> None:
        super().__init__("wifi_survey")

        self.declare_parameter("interface", "wlan1")
        self.declare_parameter("use_sudo", False)
        self.declare_parameter("scan_timeout_s", 15.0)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("output_csv", "survey.csv")
        # "continuous" or "triggered"
        self.declare_parameter("mode", "continuous")
        # continuous mode: how often to consider scanning, and how far the
        # robot must have moved since the last sample for one to be worth it.
        self.declare_parameter("poll_period_s", 2.0)
        self.declare_parameter("min_move_m", 1.0)
        # TF staleness guard. A pose older than this means SLAM has stalled,
        # and a sample pinned to a stale pose is worse than no sample.
        self.declare_parameter("max_tf_age_s", 1.0)

        p = self.get_parameter
        self.interface: str = p("interface").value
        self.use_sudo: bool = p("use_sudo").value
        self.scan_timeout_s: float = p("scan_timeout_s").value
        self.map_frame: str = p("map_frame").value
        self.base_frame: str = p("base_frame").value
        self.mode: str = p("mode").value
        self.min_move_m: float = p("min_move_m").value
        self.max_tf_age_s: float = p("max_tf_age_s").value

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        out = Path(p("output_csv").value).expanduser()
        out.parent.mkdir(parents=True, exist_ok=True)
        new_file = not out.exists() or out.stat().st_size == 0
        # Line-buffered so each sweep reaches the OS immediately.
        self._fh = out.open("a", newline="", buffering=1)
        self._writer = csv.DictWriter(self._fh, fieldnames=CSV_FIELDS)
        if new_file:
            self._writer.writeheader()
        self.output_path = out

        self._sample_id = 0
        self._last_xy: tuple[float, float] | None = None
        self._scan_lock = threading.Lock()   # one sweep at a time, ever
        self._counts = {"samples": 0, "rows": 0, "failures": 0, "skipped_tf": 0}

        self.status_pub = self.create_publisher(String, "~/status", 10)

        cb = MutuallyExclusiveCallbackGroup()
        self.srv = self.create_service(
            Trigger, "~/scan_now", self._on_scan_now, callback_group=cb
        )

        if self.mode == "continuous":
            self.timer = self.create_timer(
                p("poll_period_s").value, self._on_poll, callback_group=cb
            )
            self.get_logger().info(
                f"continuous mode: sampling every {self.min_move_m} m of travel"
            )
        else:
            self.timer = None
            self.get_logger().info("triggered mode: waiting on ~/scan_now")

        self.get_logger().info(
            f"scanning on {self.interface!r}, writing {out}\n"
            f"  pose from {self.map_frame} -> {self.base_frame}"
        )

    # ---------------------------------------------------------------- pose

    def _lookup_pose(self, when) -> tuple[float, float, float, float] | None:
        """Pose at ``when``, or None if TF cannot supply a usable one.

        Falls back to the latest available transform when the exact stamp is
        not yet in the buffer, but refuses anything older than max_tf_age_s --
        a stale pose would silently misplace the sample on the map.
        """
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, when
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.map_frame, self.base_frame, rclpy.time.Time()
                )
            except Exception as exc:
                self.get_logger().warning(
                    f"no {self.map_frame}->{self.base_frame} transform: {exc}"
                )
                return None

        stamp = tf.header.stamp
        age = (self.get_clock().now().nanoseconds * 1e-9) - (
            stamp.sec + stamp.nanosec * 1e-9
        )
        if age > self.max_tf_age_s:
            self.get_logger().warning(
                f"pose is {age:.2f}s stale (limit {self.max_tf_age_s}s) -- "
                "is SLAM still running? discarding sample"
            )
            self._counts["skipped_tf"] += 1
            return None

        t = tf.transform.translation
        r = tf.transform.rotation
        return t.x, t.y, t.z, yaw_from_quaternion(r.x, r.y, r.z, r.w)

    # ---------------------------------------------------------------- scan

    def _do_sample(self) -> tuple[bool, str]:
        """Run one sweep, join it to a pose, write the rows.

        Returns (ok, human-readable message). Never raises: a failed sample
        must not take the node down mid-survey.
        """
        if not self._scan_lock.acquire(blocking=False):
            return False, "a scan is already in flight"
        try:
            try:
                observations = scan(
                    self.interface,
                    timeout_s=self.scan_timeout_s,
                    use_sudo=self.use_sudo,
                )
            except ScanError as exc:
                self._counts["failures"] += 1
                self.get_logger().error(f"scan failed: {exc}")
                return False, str(exc)

            # Completion time, per D4.
            now = self.get_clock().now()
            pose = self._lookup_pose(now)
            if pose is None:
                return False, "no usable pose; sample discarded"

            x, y, z, yaw = pose
            stamp = now.nanoseconds * 1e-9
            sid = self._sample_id
            self._sample_id += 1

            for o in observations:
                self._writer.writerow(
                    {
                        "stamp": f"{stamp:.3f}",
                        "sample_id": sid,
                        "x": f"{x:.4f}",
                        "y": f"{y:.4f}",
                        "z": f"{z:.4f}",
                        "yaw": f"{yaw:.5f}",
                        "bssid": o.bssid,
                        "ssid": o.ssid,
                        "freq": o.freq_mhz,
                        "band": o.band,
                        "rssi": f"{o.rssi_dbm:.1f}",
                    }
                )
            self._fh.flush()

            self._last_xy = (x, y)
            self._counts["samples"] += 1
            self._counts["rows"] += len(observations)

            # An empty sweep is a real measurement -- no coverage here -- and
            # is logged as data, not as a problem.
            msg = (
                f"sample {sid} at ({x:.2f}, {y:.2f}): "
                f"{len(observations)} AP(s)"
                + ("  [no coverage]" if not observations else "")
            )
            self.get_logger().info(msg)
            self._publish_status()
            return True, msg
        finally:
            self._scan_lock.release()

    def _publish_status(self) -> None:
        c = self._counts
        self.status_pub.publish(
            String(
                data=(
                    f"samples={c['samples']} rows={c['rows']} "
                    f"failures={c['failures']} stale_pose={c['skipped_tf']} "
                    f"file={self.output_path.name}"
                )
            )
        )

    # ------------------------------------------------------------ triggers

    def _on_poll(self) -> None:
        """Continuous mode: sample once the robot has travelled far enough."""
        pose = self._lookup_pose(rclpy.time.Time())
        if pose is None:
            return
        x, y = pose[0], pose[1]
        if self._last_xy is not None:
            moved = math.hypot(x - self._last_xy[0], y - self._last_xy[1])
            if moved < self.min_move_m:
                return
        self._do_sample()

    def _on_scan_now(self, request, response):
        """Triggered mode: called by the waypoint follower once at rest."""
        ok, message = self._do_sample()
        response.success = ok
        response.message = message
        return response

    def destroy_node(self) -> bool:
        c = self._counts
        self.get_logger().info(
            f"survey closed: {c['samples']} samples, {c['rows']} rows -> "
            f"{self.output_path}"
        )
        try:
            self._fh.close()
        except Exception:
            pass
        return super().destroy_node()


def main(argv=None) -> None:
    rclpy.init(args=argv)
    node = WifiSurveyNode()
    # Multi-threaded so a 2-5 s blocking sweep does not stall TF intake.
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
