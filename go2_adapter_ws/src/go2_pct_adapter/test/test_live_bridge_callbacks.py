"""Callback regressions with fake transport; these are NOT live ROS tests."""

import importlib.util
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace as NS

import pytest


class Recorder:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


class FakePath:
    def __init__(self):
        self.header = NS(frame_id="", stamp=None)
        self.poses = []


@pytest.fixture
def node(monkeypatch):
    def module(name, **symbols):
        result = ModuleType(name)
        result.__dict__.update(symbols)
        monkeypatch.setitem(sys.modules, name, result)

    module('rclpy')
    module('rclpy.qos', qos_profile_sensor_data=object(), QoSProfile=object,
           DurabilityPolicy=NS(TRANSIENT_LOCAL=1))
    module('nav_msgs.msg', Odometry=object, Path=FakePath)
    module('std_msgs.msg', Bool=lambda **kw: NS(**kw), Empty=object)
    module('pct_scan_bridge.bridge_node', PCTScanBridge=object)
    module('pct_planner.utils', traj2ros=lambda route, **kw: NS(poses=list(route)))
    module('go2_pct_adapter.state_adapter',
           stamp_seconds=lambda stamp: stamp.sec,
           xyz=lambda p: (p.x, p.y, p.z))
    source = Path(__file__).parents[1] / 'go2_pct_adapter/live_pct_bridge.py'
    spec = importlib.util.spec_from_file_location('go2_pct_adapter._live_regression', source)
    live = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(live)
    result = live.LivePCTBridge.__new__(live.LivePCTBridge)
    result.route_pub, result.disable_pub, result.path_pub = Recorder(), Recorder(), Recorder()
    result.frame_id = 'odom'
    result.start_pose, result.goal_pose = (0, 0, 0), (1, 0, 0)
    result.start_layer = result.goal_layer = 0
    result.optimize_path = False
    result.planner_to_world = lambda data: data
    result.get_clock = lambda: NS(now=lambda: NS(to_msg=lambda: NS(sec=10)))
    result.get_logger = lambda: NS(info=lambda msg: None, error=lambda msg: None)
    result.planner = NS(plan=lambda *a, **kw: [[0, 0, 0], [1, 0, 0]])
    return result


def test_bad_new_goal_disables_old_route_before_validation(node):
    node.goal_pose_cb(NS(header=NS(frame_id='wrong'), pose=NS(position=NS(x=1, y=0, z=0))))
    assert node.disable_pub.messages[-1].data is False
    assert node.route_pub.messages[-1].data is False
    assert node.path_pub.messages[-1].poses == []


def test_failed_new_plan_never_approves_old_route(node):
    node.planner.plan = lambda *a, **kw: None
    node.plan_current_waypoints()
    assert [msg.data for msg in node.route_pub.messages] == [False]
    assert node.path_pub.messages == []


def test_planner_exception_keeps_route_invalid(node):
    def fail(*a, **kw):
        raise RuntimeError('native planner failure')
    node.planner.plan = fail
    node.plan_current_waypoints()
    assert [msg.data for msg in node.route_pub.messages] == [False]


def test_success_publishes_path_and_approves_route(node):
    node.plan_current_waypoints()
    assert len(node.path_pub.messages[-1].poses) == 2
    assert [msg.data for msg in node.route_pub.messages] == [False, True]


def test_cancel_clears_route_and_motion_enable(node):
    node.on_cancel(None)
    assert node.goal_pose is None and node.start_pose is None
    assert node.disable_pub.messages[-1].data is False
    assert node.path_pub.messages[-1].poses == []
