"""Map/goal lifecycle regressions with fake ROS transport, not live ROS tests."""

import importlib.util
import json
from pathlib import Path
import queue
import sys
import time
from types import ModuleType, SimpleNamespace as NS

import numpy as np
import pytest

from go2_pct_adapter.online_map import validate_settings


class Recorder:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


class FakePath:
    def __init__(self):
        self.header = NS(frame_id='', stamp=None)
        self.poses = []


class FakeLive:
    def invalidate_route(self):
        self.disable_pub.publish(NS(data=False))
        self.route_pub.publish(NS(data=False))
        self.path_pub.publish(FakePath())

    def on_cancel(self, msg):
        self.invalidate_route()
        self.start_pose = self.goal_pose = None


@pytest.fixture
def node(monkeypatch):
    def module(name, **symbols):
        result = ModuleType(name)
        result.__dict__.update(symbols)
        monkeypatch.setitem(sys.modules, name, result)

    module('rclpy')
    module('rclpy.duration', Duration=lambda **kw: None)
    module('rclpy.time', Time=NS(from_msg=lambda msg: None))
    module('rclpy.qos', QoSProfile=object, ReliabilityPolicy=NS(BEST_EFFORT=1))
    module('nav_msgs.msg', Path=FakePath)
    module('sensor_msgs.msg', PointCloud2=object)
    module('std_msgs.msg', Bool=lambda **kw: NS(**kw), String=lambda **kw: NS(**kw))
    module('tf2_ros', Buffer=object, TransformListener=object)
    module('pct_planner.utils', traj2ros=lambda route, **kw: NS(poses=list(route)))
    module('go2_pct_adapter.live_pct_bridge', LivePCTBridge=FakeLive)
    module('go2_pct_adapter.state_adapter',
           stamp_seconds=lambda stamp: stamp.sec,
           xyz=lambda p: (p.x, p.y, p.z), xyzw=lambda p: (p.x, p.y, p.z, p.w))
    path = Path(__file__).parents[1] / 'go2_pct_adapter/online_pct_bridge.py'
    spec = importlib.util.spec_from_file_location('go2_pct_adapter._online_regression', path)
    module_value = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module_value)
    result = module_value.OnlinePCTBridge.__new__(module_value.OnlinePCTBridge)
    for name in ('route_pub', 'disable_pub', 'path_pub', 'status_pub'):
        setattr(result, name, Recorder())
    result.frame_id = 'odom'
    result.online = validate_settings({})
    result.requested_goal = np.array([3.0, 0, 0])
    result.route_active = True
    result.needs_plan = False
    result.reason, result.backend, result.last_error = '', '', ''
    result.active_map_received = time.monotonic()
    result.active_map_stamp = 100.0
    result.latest_map = None
    result.map_sequence = result.built_sequence = 1
    result.fault_epoch = 0
    result.building = False
    result.build_results = queue.Queue()
    result.tomogram_pub = None
    result.get_clock = lambda: NS(now=lambda: NS(nanoseconds=100000000000,
                                                to_msg=lambda: NS(sec=100)))
    result.get_logger = lambda: NS(warning=lambda msg: None)
    result.robot_ground = lambda: np.array([0., 0., 0.])
    result.planner = NS(resolution=0.2, plan=lambda *args, **kw: [[0, 0, 0], [3, 0, 0]])
    result.snap_online = lambda p: (p.copy(), 0)
    return result


def goal(frame='odom', x=3.):
    return NS(header=NS(frame_id=frame), pose=NS(position=NS(x=x, y=0, z=0)))


def status(node):
    return json.loads(node.status_pub.messages[-1].data)


def test_map_refresh_replans_from_robot_without_reenabling_motion(node):
    candidate = node.planner
    node.build_results.put((candidate, {'backend': 'cpu'}, 2, time.monotonic(), 100., 0, None))
    node.online_tick()
    assert node.planner is candidate and node.route_active
    assert node.disable_pub.messages == []
    assert [m.data for m in node.route_pub.messages] == [False, False, True]
    assert status(node)['built_sequence'] == 2


def test_stale_build_never_approves_a_route(node):
    node.build_results.put((node.planner, {'backend': 'cpu'}, 2,
                            time.monotonic()-20, 80., 0, None))
    node.online_tick()
    assert not node.route_active
    assert not any(m.data for m in node.route_pub.messages)
    assert node.path_pub.messages[-1].poses == []


def test_worker_result_after_invalid_map_is_rejected(node):
    node.fault_epoch = 1
    node.build_results.put((node.planner, {'backend': 'cpu'}, 2,
                            time.monotonic(), 100., 0, None))
    node.online_tick()
    assert not node.route_active


def test_unknown_goal_is_retained_without_substitution(node):
    expected = node.requested_goal.copy()
    node.snap_online = lambda p: None if p[0] > 1 else (p, 0)
    node.plan_online()
    assert node.reason == 'goal_not_observed'
    assert not node.route_active
    np.testing.assert_array_equal(node.requested_goal, expected)
    assert node.disable_pub.messages == []


def test_cancel_prevents_a_completed_build_from_reviving_goal(node):
    node.on_cancel(None)
    node.build_results.put((node.planner, {'backend': 'cpu'}, 2,
                            time.monotonic(), 100., 0, None))
    node.online_tick()
    assert node.requested_goal is None and not node.route_active
    assert node.disable_pub.messages[-1].data is False


def test_new_invalid_goal_cancels_previous_target(node):
    node.goal_pose_cb(goal('wrong'))
    assert node.requested_goal is None
    assert node.reason == 'invalid_goal'
    assert node.disable_pub.messages[-1].data is False


def test_goal_can_arrive_before_first_map(node):
    node.active_map_received = -float('inf')
    node.goal_pose_cb(goal())
    assert node.needs_plan and node.requested_goal[0] == 3
    node.online_tick()
    assert not node.route_active and status(node)['have_goal']


def test_bad_map_invalidates_old_snapshot_even_if_it_was_fresh(node):
    node.on_map(NS(header=NS(stamp=NS(sec=0), frame_id='odom')))
    assert not node.map_fresh(node.active_map_received, node.active_map_stamp)
    node.needs_plan = True
    node.online_tick()
    assert not node.route_active


def test_no_native_route_clears_retained_path(node):
    node.planner.plan = lambda *a, **kw: None
    node.plan_online()
    assert node.reason == 'no_known_route'
    assert node.path_pub.messages[-1].poses == []
    assert not any(m.data for m in node.route_pub.messages)


def test_goal_reached_stops_and_clears_target(node):
    node.robot_ground = lambda: node.requested_goal.copy()
    node.online_tick()
    assert node.reason == 'goal_reached' and node.requested_goal is None
    assert node.disable_pub.messages[-1].data is False


def maintained(node):
    node.online.update(map_source='maintained', map_min_points=2)
    node.maintained_epoch = None
    node.retired_map_epochs = set()
    node.maintained_version = -1
    node.maintained_digest = None
    node.maintained_healthy = False
    node.maintained_received = -float('inf')
    node.maintained_stamp = 0.
    node.active_map_sequence = 0
    node.map_sequence = node.built_sequence = 0
    node.active_map_received = -float('inf')
    node.route_active = False
    return node


def snapshot(version=1, epoch='first', stamp=100, healthy=True, x=0.):
    fields = [NS(name=key, offset=i*4, datatype=7, count=1) for i, key in enumerate('xyz')]
    header = NS(frame_id='odom', stamp=NS(sec=stamp))
    points = np.asarray([[x, 0, 0], [1, 1, 0]], dtype=np.float32)
    cloud = NS(header=header, fields=fields, width=2, height=1, point_step=12,
               row_step=24, is_bigendian=False, data=points.tobytes())
    return NS(header=header, epoch=epoch, version=version, healthy=healthy,
              reason='input_timeout', cloud=cloud)


def install_snapshot(node, error=None):
    value = node.latest_map
    node.build_results.put((node.planner, {'backend': 'cpu'}, *value[3:], error))
    node.online_tick()


def test_identical_healthy_revision_renews_without_rebuild_or_disable(node):
    maintained(node)
    node.on_maintained_map(snapshot())
    install_snapshot(node)
    assert node.route_active
    sequence = node.map_sequence
    published = len(node.route_pub.messages)
    node.on_maintained_map(snapshot())
    node.online_tick()
    assert node.map_sequence == sequence == node.built_sequence
    assert len(node.route_pub.messages) == published
    assert node.disable_pub.messages == []


def test_missing_heartbeat_blocks_otherwise_fresh_geometry(node):
    maintained(node)
    node.on_maintained_map(snapshot())
    install_snapshot(node)
    node.maintained_received -= 2
    node.online_tick()
    assert not node.route_active
    assert not node.map_fresh(time.monotonic(), 100.)


def test_heartbeat_gap_recovery_rebuilds_and_replans_same_geometry(node):
    maintained(node)
    node.on_maintained_map(snapshot())
    install_snapshot(node)
    sequence = node.map_sequence
    node.maintained_received -= 2
    node.online_tick()
    assert not node.route_active and node.latest_map is None
    node.on_maintained_map(snapshot())
    install_snapshot(node)
    assert node.route_active and not node.needs_plan
    assert node.map_sequence > sequence


def test_build_completing_during_heartbeat_gap_is_retried_after_fresh_input(node):
    maintained(node)
    node.on_maintained_map(snapshot())
    value = node.latest_map
    node.build_results.put((node.planner, {'backend': 'cpu'}, *value[3:], None))
    node.maintained_received -= 2
    node.online_tick()
    assert not node.route_active and node.latest_map is None
    node.on_maintained_map(snapshot())
    install_snapshot(node)
    assert node.route_active and node.map_sequence > value[3]


def test_maintenance_fault_invalidates_map_and_inflight_result(node):
    maintained(node)
    node.on_maintained_map(snapshot())
    value = node.latest_map
    node.on_maintained_map(snapshot(healthy=False))
    node.build_results.put((node.planner, {'backend': 'cpu'}, *value[3:], None))
    node.online_tick()
    assert not node.route_active and node.latest_map is None
    assert node.active_map_sequence == 0


def test_failed_build_cannot_be_revived_by_unchanged_revision_heartbeat(node):
    maintained(node)
    node.on_maintained_map(snapshot())
    install_snapshot(node, error='native failure')
    node.on_maintained_map(snapshot())
    node.online_tick()
    assert not node.route_active
    assert node.active_map_sequence == 0
    assert node.active_map_received == -float('inf')


def test_reset_cancels_old_goal_and_old_session_packets_are_ignored(node):
    maintained(node)
    node.on_maintained_map(snapshot())
    install_snapshot(node)
    node.on_maintained_map(snapshot(epoch='second'))
    assert node.requested_goal is None
    assert node.disable_pub.messages[-1].data is False
    sequence = node.map_sequence
    node.on_maintained_map(snapshot(epoch='first'))
    assert node.maintained_epoch == 'second' and node.map_sequence == sequence


def test_version_regression_and_geometry_change_without_revision_fail_closed(node):
    maintained(node)
    node.on_maintained_map(snapshot(version=2))
    install_snapshot(node)
    node.on_maintained_map(snapshot(version=2, x=0.1))
    assert not node.route_active and not node.maintained_healthy
    assert 'without a revision' in node.last_error
    node.on_maintained_map(snapshot(version=1))
    assert not node.route_active and 'regressed' in node.last_error


def test_stale_source_cannot_be_hidden_by_fresh_heartbeat_receipt(node):
    maintained(node)
    node.on_maintained_map(snapshot(stamp=95))
    assert node.latest_map is None and not node.maintained_healthy


def test_same_revision_fault_recovery_requires_a_new_build(node):
    maintained(node)
    node.on_maintained_map(snapshot())
    install_snapshot(node)
    first = node.map_sequence
    node.on_maintained_map(snapshot(healthy=False))
    node.on_maintained_map(snapshot())
    assert node.map_sequence > first and not node.route_active
    install_snapshot(node)
    assert node.route_active


def test_unchanged_fresh_input_can_renew_a_slow_build_of_the_same_geometry(node):
    maintained(node)
    node.on_maintained_map(snapshot())
    value = node.latest_map
    node.build_results.put((node.planner, {'backend': 'cpu'}, value[3],
                            time.monotonic()-20, 80., value[6], None))
    node.on_maintained_map(snapshot())
    node.online_tick()
    assert node.route_active and node.active_map_sequence == node.map_sequence
