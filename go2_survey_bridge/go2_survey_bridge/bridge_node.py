"""Bridge ROS ``/cmd_vel`` to Go2 sport-mode requests.

Subscribes ``geometry_msgs/Twist`` and publishes ``unitree_api/msg/Request``
on ``/api/sport/request``, which ``unitree_ros2`` carries to the robot.

The robot is commanded at a steady rate rather than only on message arrival.
Sport mode expects a continuous stream; republishing the slew-limited current
velocity at a fixed period gives smooth motion and a single place to enforce
the watchdog.

Safety, in order of precedence:

1. **e-stop latched** -- nothing moves until ``~/clear_estop``
2. **watchdog tripped** -- one StopMove, then silence until commands resume
3. **normal** -- clamped, slew-limited Move at ``publish_rate_hz``

Services: ``~/stand_up``, ``~/balance_stand``, ``~/stop``, ``~/damp``,
``~/estop``, ``~/clear_estop``, ``~/recovery_stand``.

``BalanceStand`` must have been issued before Move is honoured, so the bridge
sends it on startup unless ``auto_balance_stand`` is false.
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_srvs.srv import Trigger

from go2_survey_bridge import sport_api
from go2_survey_bridge.limits import Limits, Velocity, VelocityLimiter, Watchdog

try:
    from unitree_api.msg import Request
except ImportError as exc:  # pragma: no cover - environment, not logic
    raise ImportError(
        "unitree_api messages not found. Inside the go2-survey:humble "
        "container they are baked into the image (/opt/unitree_msgs) -- "
        "rebuild it if this appears there. Elsewhere, source a unitree_ros2 "
        "workspace first.\n"
        "See docs/plan.md D1 for the runtime decision."
    ) from exc


class CmdVelBridge(Node):
    def __init__(self) -> None:
        super().__init__("go2_cmd_bridge")

        self.declare_parameter("max_vx", 0.6)
        self.declare_parameter("max_vy", 0.4)
        self.declare_parameter("max_vyaw", 0.8)
        self.declare_parameter("accel_xy", 0.8)
        self.declare_parameter("accel_yaw", 1.6)
        self.declare_parameter("cmd_timeout_s", 0.3)
        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("auto_balance_stand", True)
        # Suppress Move entirely below this magnitude. Sport mode jitters when
        # handed a stream of near-zero velocities; a deadband keeps the robot
        # genuinely still between waypoints, which matters because that is
        # when the WiFi scan happens.
        self.declare_parameter("deadband_xy", 0.02)
        self.declare_parameter("deadband_yaw", 0.03)

        g = self.get_parameter
        self.limits = Limits(
            max_vx=g("max_vx").value,
            max_vy=g("max_vy").value,
            max_vyaw=g("max_vyaw").value,
            accel_xy=g("accel_xy").value,
            accel_yaw=g("accel_yaw").value,
        )
        self.limiter = VelocityLimiter(self.limits)
        self.watchdog = Watchdog(timeout_s=g("cmd_timeout_s").value)
        self.deadband_xy = g("deadband_xy").value
        self.deadband_yaw = g("deadband_yaw").value

        self._target = Velocity()
        self._estop = False
        self._request_id = 0
        self._last_tick: float | None = None
        self._sent_stop = False

        # Sport requests are commands, not telemetry: reliable, shallow queue.
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.req_pub = self.create_publisher(
            Request, sport_api.SPORT_REQUEST_TOPIC, qos
        )
        self.status_pub = self.create_publisher(String, "~/status", 10)

        self.create_subscription(
            Twist, g("cmd_vel_topic").value, self._on_cmd_vel, 10
        )

        for name, handler in [
            ("stand_up", self._srv_stand_up),
            ("balance_stand", self._srv_balance_stand),
            ("stop", self._srv_stop),
            ("damp", self._srv_damp),
            ("recovery_stand", self._srv_recovery_stand),
            ("estop", self._srv_estop),
            ("clear_estop", self._srv_clear_estop),
        ]:
            self.create_service(Trigger, f"~/{name}", handler)

        rate = max(1.0, g("publish_rate_hz").value)
        self.create_timer(1.0 / rate, self._tick)

        if g("auto_balance_stand").value:
            # One-shot, slightly delayed: the publisher needs a moment to
            # match with the subscriber or the first request is dropped.
            self._startup_timer = self.create_timer(1.0, self._startup)

        self.get_logger().info(
            f"bridge up: {g('cmd_vel_topic').value} -> "
            f"{sport_api.SPORT_REQUEST_TOPIC} at {rate:.0f} Hz\n"
            f"  limits vx={self.limits.max_vx} vy={self.limits.max_vy} "
            f"vyaw={self.limits.max_vyaw}\n"
            f"  watchdog {self.watchdog.timeout_s}s"
        )

    # ------------------------------------------------------------- plumbing

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _send(self, api_id: int, parameter: str = "") -> None:
        msg = Request()
        msg.header.identity.id = self._request_id
        msg.header.identity.api_id = api_id
        msg.parameter = parameter
        self.req_pub.publish(msg)
        self._request_id += 1

    def _startup(self) -> None:
        self._startup_timer.cancel()
        self.get_logger().info("sending BalanceStand (Move is ignored without it)")
        self._send(sport_api.BALANCE_STAND)

    # -------------------------------------------------------------- control

    def _on_cmd_vel(self, msg: Twist) -> None:
        if self._estop:
            return
        self.watchdog.feed(self._now())
        self._target = Velocity(msg.linear.x, msg.linear.y, msg.angular.z)

    def _tick(self) -> None:
        now = self._now()
        dt = 0.0 if self._last_tick is None else now - self._last_tick
        self._last_tick = now

        if self._estop:
            return

        if self.watchdog.check(now):
            age = self.watchdog.age(now)
            self.get_logger().warning(
                f"watchdog: no /cmd_vel for {age:.2f}s "
                f"(limit {self.watchdog.timeout_s}s) -- stopping"
            )
            self.limiter.stop_immediately()
            self._target = Velocity()
            self._send(sport_api.STOP_MOVE)
            self._sent_stop = True
            self._publish_status("watchdog_stop")
            return

        if self.watchdog.tripped:
            return                      # already stopped; stay quiet

        out = self.limiter.step(self._target, dt)

        if abs(out.vx) < self.deadband_xy and abs(out.vy) < self.deadband_xy \
                and abs(out.vyaw) < self.deadband_yaw:
            # Inside the deadband: issue StopMove once, then hold still. This
            # is the state the robot is in while a WiFi scan runs.
            if not self._sent_stop:
                self._send(sport_api.STOP_MOVE)
                self._sent_stop = True
                self.limiter.reset()
                self._publish_status("idle")
            return

        self._sent_stop = False
        self._send(sport_api.MOVE, sport_api.move_parameter(out.vx, out.vy, out.vyaw))

    def _publish_status(self, state: str) -> None:
        v = self.limiter.current
        self.status_pub.publish(
            String(
                data=(
                    f"state={state} vx={v.vx:.3f} vy={v.vy:.3f} "
                    f"vyaw={v.vyaw:.3f} estop={self._estop} "
                    f"watchdog_trips={self.watchdog.trip_count}"
                )
            )
        )

    # -------------------------------------------------------------- services

    def _halt(self) -> None:
        self.limiter.stop_immediately()
        self._target = Velocity()
        self._send(sport_api.STOP_MOVE)
        self._sent_stop = True

    def _simple(self, api_id: int, response, note: str = ""):
        if self._estop and api_id not in (sport_api.DAMP,):
            response.success = False
            response.message = "e-stop latched; call ~/clear_estop first"
            return response
        self._send(api_id)
        response.success = True
        response.message = f"sent {sport_api.api_name(api_id)}{note}"
        self.get_logger().info(response.message)
        return response

    def _srv_stand_up(self, _req, res):
        return self._simple(sport_api.STAND_UP, res)

    def _srv_balance_stand(self, _req, res):
        return self._simple(sport_api.BALANCE_STAND, res)

    def _srv_recovery_stand(self, _req, res):
        return self._simple(sport_api.RECOVERY_STAND, res)

    def _srv_stop(self, _req, res):
        self._halt()
        res.success = True
        res.message = "sent StopMove"
        return res

    def _srv_damp(self, _req, res):
        # Damp is allowed even under e-stop: it is the safest terminal state,
        # and refusing it would mean the latch prevents the one command most
        # likely to be wanted in an emergency.
        self.limiter.stop_immediately()
        self._target = Velocity()
        self._send(sport_api.DAMP)
        res.success = True
        res.message = "sent Damp -- robot is limp"
        self.get_logger().warning(res.message)
        return res

    def _srv_estop(self, _req, res):
        self._estop = True
        self._halt()
        res.success = True
        res.message = "E-STOP latched; call ~/clear_estop to resume"
        self.get_logger().error(res.message)
        self._publish_status("estop")
        return res

    def _srv_clear_estop(self, _req, res):
        self._estop = False
        self.limiter.reset()
        self._target = Velocity()
        # Deliberately does not resume motion: the next /cmd_vel does that,
        # ramping from rest. Clearing an e-stop must never itself move a robot.
        res.success = True
        res.message = "e-stop cleared; robot stays still until /cmd_vel resumes"
        self.get_logger().info(res.message)
        return res

    def destroy_node(self) -> bool:
        try:
            self._send(sport_api.STOP_MOVE)
        except Exception:
            pass
        return super().destroy_node()


def main(argv=None) -> None:
    rclpy.init(args=argv)
    node = CmdVelBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
