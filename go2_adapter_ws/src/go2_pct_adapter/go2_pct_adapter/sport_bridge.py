"""Convert body-frame Twist to Unitree Sport requests in an existing Foxy graph."""

import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, Int64, String
from unitree_api.msg import Request

from .core import MotionGate, NavigationReadiness, finite, normalize_quaternion, stamp_is_fresh, stop_command
from .state_adapter import stamp_seconds, xyz, xyzw


class SportBridge(Node):
    def __init__(self):
        super().__init__("go2_sport_bridge")
        defaults = {
            "cmd_topic": "/pct_go2/cmd_vel",
            "sport_topic": "/api/sport/request",
            "planning_frame": "odom", "base_frame": "base_link",
            "cmd_timeout_sec": 0.25, "odom_timeout_sec": 0.25,
            "cloud_timeout_sec": 0.5, "publish_rate_hz": 20.0,
            "max_vx": 0.2, "max_vy": 0.1, "max_yaw_rate": 0.3,
            "enable_motion": False, "require_lease": False,
            "lease_timeout_sec": 1.0,
            "require_route_valid": False, "require_tracking_valid": False,
            "tracking_timeout_sec": 0.25,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.cfg = {name: self.get_parameter(name).value for name in defaults}
        if not finite((self.cfg["publish_rate_hz"], self.cfg["lease_timeout_sec"])) or min(
                self.cfg["publish_rate_hz"], self.cfg["lease_timeout_sec"]) <= 0:
            raise ValueError("rate and lease timeout must be finite and positive")
        self.gate = MotionGate(
            self.cfg["cmd_timeout_sec"], self.cfg["odom_timeout_sec"],
            self.cfg["cloud_timeout_sec"],
            (self.cfg["max_vx"], self.cfg["max_vy"], self.cfg["max_yaw_rate"]))
        self.gate.enable(self.cfg["enable_motion"], time.monotonic())
        self.readiness = NavigationReadiness(
            self.cfg["require_route_valid"], self.cfg["require_tracking_valid"],
            self.cfg["tracking_timeout_sec"])
        self.lease_id, self.lease_at = 0, -float("inf")
        self.have_published_motion = False
        self.last_status = None
        self.request_pub = self.create_publisher(Request, self.cfg["sport_topic"], 10)
        self.preview_pub = self.create_publisher(Request, "/pct_go2/sport_request_preview", 10)
        self.status_pub = self.create_publisher(String, "/pct_go2/bridge_status", 10)
        self.execution_pub = self.create_publisher(Bool, "/pct_go2/execution_enabled", 1)
        self.subscriptions_kept = [
            self.create_subscription(Twist, self.cfg["cmd_topic"], self.on_cmd, 1),
            self.create_subscription(Bool, "/pct_go2/motion_enable", self.on_enable, 1),
            self.create_subscription(Int64, "/pct_go2/lease_id", self.on_lease, 1),
            self.create_subscription(Odometry, "/pct_go2/body_pose", self.on_odom,
                                     qos_profile_sensor_data),
            self.create_subscription(PointCloud2, "/pct_go2/cloud", self.on_cloud,
                                     qos_profile_sensor_data),
            self.create_subscription(Bool, "/pct_go2/route_valid", self.on_route,
                                     QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)),
            self.create_subscription(Bool, "/pct_go2/tracking_valid", self.on_tracking, 1),
        ]
        self.timer = self.create_timer(1.0 / self.cfg["publish_rate_hz"], self.tick)
        self.get_logger().info("Sport bridge started; enable_motion=" + str(self.gate.enabled))

    def on_cmd(self, msg):
        self.gate.update_command((*xyz(msg.linear), *xyz(msg.angular)), time.monotonic())

    def on_enable(self, msg):
        was_enabled = self.gate.enabled
        self.gate.enable(msg.data, time.monotonic())
        if was_enabled and not self.gate.enabled and self.have_published_motion:
            self.request_pub.publish(self.make_request(stop_command()))
            self.have_published_motion = False

    def on_lease(self, msg):
        # An external SDK lease manager acquires AND renews the lease. This
        # adapter never invents the undocumented sport_lease request protocol.
        self.lease_id = int(msg.data)
        self.lease_at = time.monotonic()

    def on_route(self, msg):
        self.readiness.route(msg.data, time.monotonic())
        if not msg.data and self.cfg["require_route_valid"]:
            self.gate.command = None

    def on_tracking(self, msg):
        self.readiness.tracking(msg.data, time.monotonic())

    def fresh(self, stamp, timeout):
        return stamp_is_fresh(self.get_clock().now().nanoseconds * 1e-9,
                              stamp_seconds(stamp), timeout)

    def on_odom(self, msg):
        valid = (msg.header.frame_id == self.cfg["planning_frame"] and
                 msg.child_frame_id == self.cfg["base_frame"] and
                 finite(xyz(msg.pose.pose.position)) and
                 self.fresh(msg.header.stamp, self.cfg["odom_timeout_sec"]))
        try:
            normalize_quaternion(xyzw(msg.pose.pose.orientation))
        except ValueError:
            valid = False
        if valid:
            self.gate.odom_at = time.monotonic()
        else:
            self.gate.invalidate("odom")

    def on_cloud(self, msg):
        if msg.width * msg.height > 0 and self.fresh(
                msg.header.stamp, self.cfg["cloud_timeout_sec"]):
            self.gate.cloud_at = time.monotonic()
        else:
            self.gate.invalidate("cloud")

    def make_request(self, command):
        msg = Request()
        msg.header.identity.id = time.time_ns()
        msg.header.identity.api_id = command.api_id
        msg.header.lease.id = self.lease_id
        msg.parameter = command.parameter
        return msg

    def tick(self):
        now = time.monotonic()
        command, reason = self.gate.evaluate(now)
        readiness_reason = self.readiness.reason(now)
        if readiness_reason is not None:
            command, reason = stop_command(), readiness_reason
        lease_ok = (not self.cfg["require_lease"] or
                    (self.lease_id > 0 and
                     0 <= now - self.lease_at <= self.cfg["lease_timeout_sec"]))
        if not lease_ok:
            command, reason = stop_command(), "lease_unavailable"
        request = self.make_request(command)
        self.preview_pub.publish(request)
        self.execution_pub.publish(Bool(data=(self.gate.enabled and lease_ok and
                                              reason in ("ready", "zero_command"))))
        if self.gate.enabled:
            self.request_pub.publish(request)
            self.have_published_motion = True
        status = json.dumps({"motion_enabled": self.gate.enabled,
                             "reason": reason, "api_id": command.api_id,
                             "lease_ok": lease_ok})
        self.status_pub.publish(String(data=status))
        if status != self.last_status:
            self.get_logger().info(status)
            self.last_status = status

    def stop_if_active(self):
        if self.have_published_motion:
            self.request_pub.publish(self.make_request(stop_command()))


def main(args=None):
    rclpy.init(args=args)
    node = SportBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Best effort only: the existing controller/SDK must retain its own
        # watchdog for process crashes and loss of the network connection.
        if rclpy.ok():
            node.stop_if_active()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
