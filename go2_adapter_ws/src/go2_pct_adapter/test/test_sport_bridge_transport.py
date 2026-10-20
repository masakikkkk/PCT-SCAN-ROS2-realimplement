"""Actuation-boundary regressions with fake publishers, not a live ROS graph."""

import importlib.util
from pathlib import Path
import sys
import time
from types import ModuleType, SimpleNamespace as NS

import pytest

from go2_pct_adapter.core import MotionGate, NavigationReadiness


class Recorder:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


@pytest.fixture
def bridge(monkeypatch):
    def module(name, **symbols):
        value = ModuleType(name)
        value.__dict__.update(symbols)
        monkeypatch.setitem(sys.modules, name, value)

    module('rclpy')
    module('rclpy.node', Node=object)
    module('rclpy.qos', qos_profile_sensor_data=object(), QoSProfile=object,
           DurabilityPolicy=NS(TRANSIENT_LOCAL=1))
    module('geometry_msgs.msg', Twist=object)
    module('nav_msgs.msg', Odometry=object)
    module('sensor_msgs.msg', PointCloud2=object)
    module('std_msgs.msg', Bool=lambda **kw: NS(**kw), Int64=object,
           String=lambda **kw: NS(**kw))
    module('unitree_api.msg', Request=lambda: NS(
        header=NS(identity=NS(id=0, api_id=0), lease=NS(id=0)), parameter=''))
    module('go2_pct_adapter.state_adapter', stamp_seconds=lambda t: t.sec,
           xyz=lambda p: (p.x, p.y, p.z), xyzw=lambda q: (q.x, q.y, q.z, q.w))
    path = Path(__file__).parents[1] / 'go2_pct_adapter/sport_bridge.py'
    spec = importlib.util.spec_from_file_location('go2_pct_adapter._sport_regression', path)
    sport = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(sport)
    node = sport.SportBridge.__new__(sport.SportBridge)
    node.cfg = {'require_lease': False, 'lease_timeout_sec': 1.0,
                'require_route_valid': True}
    node.gate = MotionGate()
    now = time.monotonic()
    node.gate.odom_at = node.gate.cloud_at = now
    node.gate.update_command((0.1, 0, 0, 0, 0, 0), now)
    node.readiness = NavigationReadiness(True, True)
    node.readiness.route(True, now)
    node.readiness.tracking(True, now)
    node.lease_id, node.lease_at = 0, -float('inf')
    node.have_published_motion = False
    node.last_status = None
    node.preview_pub, node.request_pub = Recorder(), Recorder()
    node.execution_pub, node.status_pub = Recorder(), Recorder()
    node.get_logger = lambda: NS(info=lambda msg: None)
    return node


def enable(node):
    now = time.monotonic()
    node.gate.enable(True, now)
    node.gate.update_command((0.1, 0, 0, 0, 0, 0), now)


def test_preview_never_publishes_to_sport_topic(bridge):
    bridge.tick()
    assert bridge.preview_pub.messages[-1].header.identity.api_id == 1008
    assert bridge.request_pub.messages == []
    assert bridge.execution_pub.messages[-1].data is False


def test_invalid_tracking_replaces_motion_with_stop(bridge):
    enable(bridge)
    bridge.tick()
    assert bridge.request_pub.messages[-1].header.identity.api_id == 1008
    bridge.readiness.tracking(False, time.monotonic())
    bridge.tick()
    assert bridge.request_pub.messages[-1].header.identity.api_id == 1003
    assert bridge.execution_pub.messages[-1].data is False


def test_missing_lease_blocks_move_even_with_valid_route_and_command(bridge):
    enable(bridge)
    bridge.cfg['require_lease'] = True
    bridge.tick()
    assert bridge.request_pub.messages[-1].header.identity.api_id == 1003
    assert bridge.execution_pub.messages[-1].data is False


def test_disable_sends_stop_then_stops_sending_requests(bridge):
    enable(bridge)
    bridge.tick()
    bridge.on_enable(NS(data=False))
    assert bridge.request_pub.messages[-1].header.identity.api_id == 1003
    count = len(bridge.request_pub.messages)
    bridge.tick()
    assert len(bridge.request_pub.messages) == count
