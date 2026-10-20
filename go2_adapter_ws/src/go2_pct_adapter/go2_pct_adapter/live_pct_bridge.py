"""Use fresh robot odometry as PCT's start when receiving a navigation goal.

Requires the downloaded pct_scan_bridge / pct_planner packages at runtime.
"""

import copy
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile, DurabilityPolicy
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Bool, Empty
from pct_scan_bridge.bridge_node import PCTScanBridge
from pct_planner.utils import traj2ros

from .core import finite, stamp_is_fresh
from .state_adapter import stamp_seconds, xyz


class LivePCTBridge(PCTScanBridge):
    def __init__(self):
        super().__init__()
        self.declare_parameter("robot_body_height", 0.4)
        self.declare_parameter("robot_odom_timeout_sec", 0.25)
        self.body_height = self.get_parameter("robot_body_height").value
        self.odom_timeout = self.get_parameter("robot_odom_timeout_sec").value
        if not finite((self.body_height, self.odom_timeout)) or self.body_height < 0 or self.odom_timeout <= 0:
            raise ValueError("body height must be nonnegative and odom timeout positive")
        self.latest_odom = None
        self.odom_received = -float("inf")
        self.route_pub = self.create_publisher(
            Bool, "/pct_go2/route_valid",
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.disable_pub = self.create_publisher(Bool, "/pct_go2/motion_enable", 1)
        self.cancel_sub = self.create_subscription(Empty, "/pct_go2/cancel", self.on_cancel, 1)
        self.route_pub.publish(Bool(data=False))
        self.robot_sub = self.create_subscription(
            Odometry, "/pct_go2/body_pose", self.on_robot_odom, qos_profile_sensor_data)

    def on_robot_odom(self, msg):
        if msg.header.frame_id != self.frame_id or not finite(xyz(msg.pose.pose.position)):
            self.latest_odom = None
            return
        self.latest_odom = copy.deepcopy(msg)
        self.odom_received = time.monotonic()

    def start_pose_cb(self, msg):
        self.get_logger().warning("Live PCT start comes from robot odometry; ignoring manual start")

    def invalidate_route(self):
        # Every new goal requires a fresh operator enable after previewing it.
        self.disable_pub.publish(Bool(data=False))
        self.route_pub.publish(Bool(data=False))
        empty = Path()
        empty.header.frame_id = self.frame_id
        empty.header.stamp = self.get_clock().now().to_msg()
        self.path_pub.publish(empty)

    def on_cancel(self, msg):
        self.invalidate_route()
        self.start_pose = self.goal_pose = None
        self.get_logger().info("Route cancelled; motion disabled")

    def plan_current_waypoints(self):
        # The base constructor can call this if plan_on_startup was overridden.
        # Live operation never starts a scene-demo route during construction.
        if not hasattr(self, "route_pub"):
            return
        self.route_pub.publish(Bool(data=False))
        if self.start_pose is None or self.goal_pose is None:
            return
        try:
            trajectory = self.planner.plan(
                self.start_pose, self.goal_pose, start_layer=self.start_layer,
                end_layer=self.goal_layer, optimize=self.optimize_path)
            if (trajectory is None or len(trajectory) < 2 or
                    any(len(row) != 3 or not finite(row) for row in trajectory)):
                raise ValueError("PCT returned no valid route")
            world = self.planner_to_world(trajectory)
            path = traj2ros(world, frame_id=self.frame_id,
                           stamp=self.get_clock().now().to_msg())
            self.path_pub.publish(path)
        except Exception as exc:
            self.get_logger().error("Route planning failed; motion remains disabled: " + str(exc))
            return
        self.route_pub.publish(Bool(data=True))
        self.get_logger().info("New route published; inspect preview before enabling motion")

    def goal_pose_cb(self, msg):
        self.invalidate_route()
        if msg.header.frame_id != self.frame_id or not finite(xyz(msg.pose.position)):
            self.get_logger().error("Goal must have finite coordinates in " + self.frame_id)
            return
        if self.latest_odom is None or not 0 <= time.monotonic() - self.odom_received <= self.odom_timeout:
            self.get_logger().error("Cannot plan without fresh robot odometry")
            return
        if not stamp_is_fresh(self.get_clock().now().nanoseconds * 1e-9,
                              stamp_seconds(self.latest_odom.header.stamp), self.odom_timeout):
            self.get_logger().error("Robot odometry timestamp is stale")
            return
        p = self.latest_odom.pose.pose.position
        # PCT stores ground-surface height; SCAN adds its body-height offset.
        if self.set_start(p.x, p.y, p.z - self.body_height, plan=False):
            super().goal_pose_cb(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LivePCTBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
