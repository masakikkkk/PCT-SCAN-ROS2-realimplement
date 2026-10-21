"""Build PCT snapshots from /uslam/cloud_map and replan an observed goal.

No SLAM, frontier exploration, controller or motion enable is started here.
Map builds run in one daemon worker; completed planners are swapped by the ROS
executor, never mutated concurrently with A*.
"""

import json
import math
import queue
import threading
import time

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from nav_msgs.msg import Path
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener

from pct_planner.utils import traj2ros

from .core import finite, stamp_is_fresh
from .live_pct_bridge import LivePCTBridge
from .online_map import (ONLINE_DEFAULTS, build_tomogram, cloud_xyz,
                         transform_points, validate_settings)
from .state_adapter import stamp_seconds, xyz, xyzw


class OnlinePCTBridge(LivePCTBridge):
    def __init__(self):
        super().__init__()
        if not self.get_parameter("defer_map_loading").value:
            raise ValueError("online_pct_bridge requires defer_map_loading=true")
        for name, value in ONLINE_DEFAULTS.items():
            self.declare_parameter(name, value)
        self.online = validate_settings({name: self.get_parameter(name).value
                                         for name in ONLINE_DEFAULTS})
        self.map_buffer = Buffer()
        self.map_listener = TransformListener(self.map_buffer, self)
        self.latest_map = None
        self.map_sequence = self.built_sequence = self.fault_epoch = 0
        self.map_received = self.active_map_received = -math.inf
        self.active_map_stamp = 0.0
        self.last_build = -math.inf
        self.building = False
        self.build_results = queue.Queue(maxsize=1)
        self.requested_goal = None
        self.needs_plan = self.route_active = False
        self.reason, self.backend, self.last_error = "waiting_for_map", "", ""
        self.status_pub = self.create_publisher(String, "/pct_go2/online_status", 1)
        self.map_sub = self.create_subscription(
            PointCloud2, self.online["map_topic"], self.on_map,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.online_timer = self.create_timer(0.2, self.online_tick)
        self.get_logger().info("Online PCT waiting for accumulated PointCloud2 on " +
                               self.online["map_topic"])

    def map_fresh(self, received, stamp):
        return (0 <= time.monotonic() - received <= self.online["map_timeout_sec"] and
                stamp_is_fresh(self.get_clock().now().nanoseconds * 1e-9, stamp,
                               self.online["map_timeout_sec"]))

    def clear_online_route(self, reason):
        # Internal map refreshes stop tracking but preserve the user's existing
        # enable for this goal. A new goal/cancel still uses Live's disable path.
        self.route_pub.publish(Bool(data=False))
        if self.route_active:
            empty = Path()
            empty.header.frame_id = self.frame_id
            empty.header.stamp = self.get_clock().now().to_msg()
            self.path_pub.publish(empty)
        self.route_active = False
        self.reason = reason

    def on_map(self, msg):
        try:
            stamp = stamp_seconds(msg.header.stamp)
            if not stamp_is_fresh(self.get_clock().now().nanoseconds * 1e-9,
                                  stamp, self.online["map_timeout_sec"]):
                raise ValueError("Map timestamp is stale, zero or in the future")
            if not msg.header.frame_id:
                raise ValueError("Map frame_id is empty")
            count = msg.width * msg.height
            if not self.online["map_min_points"] <= count <= self.online["map_max_points"]:
                raise ValueError("Map point count is outside configured limits")
            translation, rotation = (0, 0, 0), (0, 0, 0, 1)
            if msg.header.frame_id != self.frame_id:
                tf = self.map_buffer.lookup_transform(
                    self.frame_id, msg.header.frame_id, Time.from_msg(msg.header.stamp),
                    timeout=Duration(seconds=0.0)).transform
                translation, rotation = xyz(tf.translation), xyzw(tf.rotation)
                # Validate before accepting the snapshot; never rename frames.
                transform_points(np.zeros((1, 3)), translation, rotation)
            self.map_sequence += 1
            self.map_received = time.monotonic()
            self.latest_map = (msg, translation, rotation, self.map_sequence,
                               self.map_received, stamp, self.fault_epoch)
        except Exception as exc:
            self.latest_map = None
            self.fault_epoch += 1
            self.active_map_received = -math.inf
            self.last_error = str(exc)
            self.clear_online_route("invalid_map")

    def launch_build(self, snapshot):
        self.building = True
        self.last_build = time.monotonic()
        planner_type, cfg, root = type(self.planner), self.cfg, self.rsg_root

        def work():
            try:
                msg, translation, rotation, seq, received, stamp, epoch = snapshot
                points = cloud_xyz(msg, self.online["map_max_points"])
                points = transform_points(points, translation, rotation)
                metadata = build_tomogram(points, self.online)
                candidate = planner_type(cfg, root)
                candidate.loadTomogramData(metadata)
                result = (candidate, metadata, seq, received, stamp, epoch, None)
            except Exception as exc:
                result = (None, None, snapshot[3], snapshot[4], snapshot[5],
                          snapshot[6], str(exc))
            self.build_results.put(result)

        threading.Thread(target=work, name="online-pct-map", daemon=True).start()

    def on_cancel(self, msg):
        super().on_cancel(msg)
        self.requested_goal = None
        self.route_active = self.needs_plan = False
        self.reason = "cancelled"

    def goal_pose_cb(self, msg):
        self.invalidate_route()
        self.route_active = False
        self.requested_goal = None
        self.start_pose = self.goal_pose = None
        if msg.header.frame_id != self.frame_id or not finite(xyz(msg.pose.position)):
            self.reason = "invalid_goal"
            return
        # Retain the ORIGINAL XYZ, even before a map exists. Never retain layer
        # indices or a clipped/snap-to-border substitute across map versions.
        self.requested_goal = np.asarray(xyz(msg.pose.position), dtype=np.float32)
        self.needs_plan = True
        self.reason = "waiting_for_map"

    def robot_ground(self):
        now = time.monotonic()
        if (self.latest_odom is None or not 0 <= now - self.odom_received <= self.odom_timeout or
                not stamp_is_fresh(self.get_clock().now().nanoseconds * 1e-9,
                                   stamp_seconds(self.latest_odom.header.stamp), self.odom_timeout)):
            return None
        position = np.asarray(xyz(self.latest_odom.pose.pose.position), dtype=np.float32)
        position[2] -= self.body_height
        return position

    def snap_online(self, position):
        radius = self.online["goal_snap_radius_m"]
        snapped = self.planner.snap_to_traversable(
            position, radius_cells=int(math.ceil(radius / self.planner.resolution)))
        if snapped is None:
            return None
        pose, idx, _ = snapped
        if (np.linalg.norm(pose[:2] - position[:2]) > radius or
                abs(pose[2] - position[2]) > self.online["goal_height_tolerance_m"]):
            return None
        return pose, int(idx[0])

    def plan_online(self):
        start = self.robot_ground()
        if start is None:
            self.reason = "waiting_for_odometry"
            return
        self.needs_plan = False
        self.clear_online_route("replanning")
        try:
            snapped_start, snapped_goal = self.snap_online(start), self.snap_online(self.requested_goal)
            if snapped_start is None or snapped_goal is None:
                self.reason = ("start_not_observed" if snapped_start is None else
                               "goal_not_observed")
                return
            self.start_pose, self.start_layer = snapped_start
            self.goal_pose, self.goal_layer = snapped_goal
            route = self.planner.plan(self.start_pose, self.goal_pose,
                                      start_layer=self.start_layer, end_layer=self.goal_layer,
                                      optimize=False)
            if route is None or len(route) < 2 or not np.all(np.isfinite(route)):
                self.reason = "no_known_route"
                return
            path = traj2ros(route, frame_id=self.frame_id,
                           stamp=self.get_clock().now().to_msg())
            self.path_pub.publish(path)
            self.route_pub.publish(Bool(data=True))
            self.route_active, self.reason, self.last_error = True, "route_ready", ""
        except Exception as exc:
            self.reason, self.last_error = "planning_failed", str(exc)

    def online_tick(self):
        try:
            candidate, meta, seq, received, stamp, epoch, error = self.build_results.get_nowait()
        except queue.Empty:
            pass
        else:
            self.building = False
            # Remember failed attempts too: retry only with a NEW map message.
            self.built_sequence = seq
            if error or epoch != self.fault_epoch or not self.map_fresh(received, stamp):
                self.active_map_received = -math.inf
                self.last_error = error or "Map build was superseded by an invalid/stale input"
                self.clear_online_route("map_build_failed")
            else:
                self.clear_online_route("map_updated")
                self.planner = candidate
                self.active_map_received, self.active_map_stamp = received, stamp
                self.backend = meta["backend"]
                self.needs_plan = self.requested_goal is not None
                self.last_error = ""
                if self.tomogram_pub is not None:
                    try:
                        self.publish_tomogram()
                    except Exception as exc:
                        self.get_logger().warning("Tomogram visualization failed: " + str(exc))
        if not self.map_fresh(self.active_map_received, self.active_map_stamp):
            self.clear_online_route("waiting_for_map" if self.built_sequence == 0 else "map_timeout")
        elif self.requested_goal is not None:
            start = self.robot_ground()
            if (start is not None and
                    np.linalg.norm(start[:2] - self.requested_goal[:2]) <= self.online["goal_reached_radius_m"] and
                    abs(start[2] - self.requested_goal[2]) <= self.online["goal_height_tolerance_m"]):
                self.on_cancel(None)
                self.reason = "goal_reached"
            elif self.needs_plan:
                self.plan_online()
        if (not self.building and self.latest_map is not None and
                self.latest_map[3] > self.built_sequence and
                self.map_fresh(self.latest_map[4], self.latest_map[5]) and
                time.monotonic() - self.last_build >= self.online["map_update_period_sec"]):
            self.launch_build(self.latest_map)
        self.status_pub.publish(String(data=json.dumps({
            "reason": self.reason, "map_topic": self.online["map_topic"],
            "map_sequence": self.map_sequence, "built_sequence": self.built_sequence,
            "building": self.building, "backend": self.backend,
            "have_goal": self.requested_goal is not None,
            "route_valid": self.route_active, "error": self.last_error})))


def main(args=None):
    rclpy.init(args=args)
    node = OnlinePCTBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.on_cancel(None)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
