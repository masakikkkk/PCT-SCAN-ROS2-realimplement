"""ROS-independent command gating and rigid-body math (Python 3.8 compatible)."""

import json
import math
from dataclasses import dataclass


def finite(values):
    return all(math.isfinite(float(value)) for value in values)


def normalize_quaternion(q):
    if len(q) != 4 or not finite(q):
        raise ValueError("quaternion must contain four finite values")
    scale = max(abs(value) for value in q)
    if scale < 1e-8:
        raise ValueError("zero quaternion")
    scaled = tuple(value / scale for value in q)
    norm = math.sqrt(sum(value * value for value in scaled))
    return tuple(value / norm for value in scaled)


def multiply_quaternion(a, b):
    x, y, z, w = a
    X, Y, Z, W = b
    return normalize_quaternion((
        w*X + x*W + y*Z - z*Y,
        w*Y - x*Z + y*W + z*X,
        w*Z + x*Y - y*X + z*W,
        w*W - x*X - y*Y - z*Z,
    ))


def rotate(q, v):
    x, y, z, w = normalize_quaternion(q)
    u = (x, y, z)
    cross = (y*v[2] - z*v[1], z*v[0] - x*v[2], x*v[1] - y*v[0])
    dot = sum(a*b for a, b in zip(u, v))
    return tuple(2*dot*u[i] + (2*w*w - 1)*v[i] + 2*w*cross[i]
                 for i in range(3))


def sensor_pose(body_position, body_orientation, translation, rotation):
    """Compose T_world_body * T_body_sensor; quaternions use xyzw."""
    if not finite((*body_position, *translation)):
        raise ValueError("pose translation is not finite")
    offset = rotate(body_orientation, translation)
    return (tuple(a+b for a, b in zip(body_position, offset)),
            multiply_quaternion(normalize_quaternion(body_orientation),
                                normalize_quaternion(rotation)))


def stamp_is_fresh(now_sec, stamp_sec, max_age, future_tolerance=0.05):
    return (finite((now_sec, stamp_sec)) and stamp_sec > 0 and
            -future_tolerance <= now_sec - stamp_sec <= max_age)


@dataclass(frozen=True)
class SportCommand:
    api_id: int
    parameter: str


def stop_command():
    return SportCommand(1003, "{}")


def move_command(vx, vy, yaw_rate):
    if not finite((vx, vy, yaw_rate)):
        raise ValueError("non-finite velocity")
    return SportCommand(1008, json.dumps(
        {"x": float(vx), "y": float(vy), "z": float(yaw_rate)}, allow_nan=False))


class MotionGate:
    """Fresh command + odometry + cloud are required, even in preview mode.

    Times passed here must come from time.monotonic(), never ROS time.
    Invalid commands invalidate the previous command instead of keeping it alive.
    """

    def __init__(self, cmd_timeout=0.25, odom_timeout=0.25,
                 cloud_timeout=0.5, limits=(0.2, 0.1, 0.3)):
        if not finite((cmd_timeout, odom_timeout, cloud_timeout, *limits)) or min(
                cmd_timeout, odom_timeout, cloud_timeout, *limits) <= 0:
            raise ValueError("timeouts and speed limits must be finite and positive")
        self.timeouts = (cmd_timeout, odom_timeout, cloud_timeout)
        self.limits = tuple(limits)
        self.enabled = False
        self.enabled_at = math.inf
        self.command = None
        self.cmd_at = self.odom_at = self.cloud_at = -math.inf

    def enable(self, enabled, now):
        if bool(enabled) != self.enabled:
            self.enabled = bool(enabled)
            self.enabled_at = now if enabled else math.inf
            # A new enable requires a command received after that transition.
            self.command = None
            self.cmd_at = -math.inf

    def update_command(self, values, now):
        if len(values) != 6 or not finite(values):
            self.command = None
            return False
        # linear.z / angular.x / angular.y are unsupported by Sport Move.
        if any(abs(values[i]) > 1e-6 for i in (2, 3, 4)):
            self.command = None
            return False
        requested = (values[0], values[1], values[5])
        self.command = tuple(max(-limit, min(limit, value))
                             for value, limit in zip(requested, self.limits))
        self.cmd_at = now
        return True

    def invalidate(self, source):
        if source not in ("odom", "cloud"):
            raise ValueError("unknown sensor source")
        setattr(self, source + "_at", -math.inf)

    def evaluate(self, now):
        if self.command is None:
            return stop_command(), "no_valid_command"
        for source, timestamp, timeout in zip(
                ("cmd", "odom", "cloud"),
                (self.cmd_at, self.odom_at, self.cloud_at), self.timeouts):
            if not 0 <= now - timestamp <= timeout:
                return stop_command(), source + "_timeout"
        if self.enabled and self.cmd_at < self.enabled_at:
            return stop_command(), "waiting_for_new_command"
        if all(abs(value) < 1e-6 for value in self.command):
            return stop_command(), "zero_command"
        return move_command(*self.command), "ready" if self.enabled else "preview"


class NavigationReadiness:
    """Optional route approval plus a live, bounded-validity tracker heartbeat."""

    def __init__(self, require_route=False, require_tracking=False, timeout=0.25):
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError("tracking timeout must be finite and positive")
        self.require_route = require_route
        self.require_tracking = require_tracking
        self.timeout = timeout
        self.route_valid = False
        self.route_at = -math.inf
        self.tracking_valid = False
        self.tracking_at = -math.inf

    def route(self, valid, now):
        self.route_valid = bool(valid)
        self.route_at = now
        self.tracking_valid = False
        self.tracking_at = -math.inf

    def tracking(self, valid, now):
        self.tracking_valid = bool(valid)
        self.tracking_at = now

    def reason(self, now):
        if self.require_route and not self.route_valid:
            return "route_unavailable"
        if self.require_tracking:
            if not self.tracking_valid:
                return "tracking_unavailable"
            if not 0 <= now - self.tracking_at <= self.timeout:
                return "tracking_timeout"
            if self.require_route and self.tracking_at < self.route_at:
                return "waiting_for_tracking"
        return None
