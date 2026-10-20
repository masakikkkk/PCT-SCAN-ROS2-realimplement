import json
import math

import pytest

from go2_pct_adapter.core import (
    MotionGate, move_command, normalize_quaternion, rotate, sensor_pose, stamp_is_fresh,
)


def ready_gate(enabled=False):
    gate = MotionGate()
    gate.enable(enabled, 10.0)
    gate.odom_at = gate.cloud_at = 10.01
    gate.update_command((0.1, 0.02, 0.0, 0.0, 0.0, 0.15), 10.01)
    return gate


def test_sport_payload_z_is_yaw_rate_and_nan_is_rejected():
    command = move_command(0.1, -0.02, 0.3)
    assert command.api_id == 1008
    assert json.loads(command.parameter) == {"x": 0.1, "y": -0.02, "z": 0.3}
    with pytest.raises(ValueError):
        move_command(math.nan, 0, 0)


def test_disabled_gate_can_preview_without_enabling_motion():
    gate = ready_gate()
    command, reason = gate.evaluate(10.02)
    assert not gate.enabled
    assert command.api_id == 1008
    assert reason == "preview"


def test_enable_and_reenable_require_a_new_command():
    gate = ready_gate()
    gate.enable(True, 10.02)
    assert gate.evaluate(10.03)[0].api_id == 1003
    gate.update_command((0.1, 0, 0, 0, 0, 0), 10.04)
    assert gate.evaluate(10.05)[0].api_id == 1008
    gate.enable(False, 10.06)
    gate.enable(True, 10.07)
    assert gate.evaluate(10.08)[0].api_id == 1003


def test_repeated_true_does_not_erase_a_fresh_command():
    gate = ready_gate(True)
    gate.enable(True, 10.02)
    assert gate.evaluate(10.03)[0].api_id == 1008


@pytest.mark.parametrize("source", ["cmd", "odom", "cloud"])
def test_each_input_has_its_own_watchdog(source):
    gate = ready_gate(True)
    setattr(gate, source + "_at", 0.0)
    command, reason = gate.evaluate(10.02)
    assert command.api_id == 1003
    assert reason == source + "_timeout"


@pytest.mark.parametrize("values", [
    (math.nan, 0, 0, 0, 0, 0), (0, math.inf, 0, 0, 0, 0),
    (0.1, 0, 0.1, 0, 0, 0), (0.1, 0, 0, 0.2, 0, 0),
])
def test_invalid_or_unsupported_command_invalidates_previous_motion(values):
    gate = ready_gate(True)
    assert not gate.update_command(values, 10.02)
    assert gate.evaluate(10.03)[0].api_id == 1003


def test_velocity_limits_and_zero_stop():
    gate = ready_gate(True)
    gate.update_command((100, -100, 0, 0, 0, 100), 10.02)
    assert json.loads(gate.evaluate(10.03)[0].parameter) == {"x": 0.2, "y": -0.1, "z": 0.3}
    gate.update_command((0, 0, 0, 0, 0, 0), 10.04)
    assert gate.evaluate(10.05)[0].api_id == 1003


def test_invalid_sensor_immediately_blocks_motion():
    gate = ready_gate(True)
    gate.invalidate("odom")
    assert gate.evaluate(10.02)[1] == "odom_timeout"


def test_monotonic_time_regression_does_not_authorize_motion():
    gate = ready_gate(True)
    assert gate.evaluate(9.0)[0].api_id == 1003


def test_sensor_extrinsic_rotates_with_body():
    q = (0, 0, math.sin(math.pi / 4), math.cos(math.pi / 4))
    position, orientation = sensor_pose((2, 3, 1), q, (0.2, 0, 0.1), (0, 0, 0, 1))
    assert position == pytest.approx((2, 3.2, 1.1))
    assert orientation == pytest.approx(q)
    assert rotate(q, (1, 0, 0)) == pytest.approx((0, 1, 0))


def test_bad_quaternion_and_nonfinite_extrinsic_fail():
    with pytest.raises(ValueError):
        normalize_quaternion((0, 0, 0, 0))
    with pytest.raises(ValueError):
        sensor_pose((0, 0, 0), (0, 0, 0, 1), (math.inf, 0, 0), (0, 0, 0, 1))


@pytest.mark.parametrize("stamp, expected", [(0, False), (99, False), (100.01, True),
                                              (101, False), (99.9, True)])
def test_header_stamp_validation(stamp, expected):
    assert stamp_is_fresh(100, stamp, 0.25) is expected


def test_invalid_timeout_config_fails():
    with pytest.raises(ValueError):
        MotionGate(cmd_timeout=-1)
    with pytest.raises(ValueError):
        MotionGate(limits=(math.nan, 0.1, 0.3))
