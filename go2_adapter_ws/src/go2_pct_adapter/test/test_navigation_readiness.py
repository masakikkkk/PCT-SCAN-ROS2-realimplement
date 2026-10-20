import math

import pytest

from go2_pct_adapter.core import NavigationReadiness, normalize_quaternion


def test_interface_mapping_does_not_require_planner_messages():
    assert NavigationReadiness().reason(10) is None


def test_navigation_requires_route_and_tracking():
    gate = NavigationReadiness(True, True)
    assert gate.reason(10) == "route_unavailable"
    gate.route(True, 10)
    assert gate.reason(10.01) == "tracking_unavailable"
    gate.tracking(True, 10.01)
    assert gate.reason(10.02) is None


def test_new_route_invalidates_old_tracker_approval():
    gate = NavigationReadiness(True, True)
    gate.route(True, 10)
    gate.tracking(True, 10.1)
    assert gate.reason(10.2) is None
    gate.route(False, 10.21)
    assert gate.reason(10.22) == "route_unavailable"
    gate.route(True, 10.23)
    assert gate.reason(10.24) == "tracking_unavailable"


def test_explicit_tracker_stop_and_missing_heartbeat_block_execution():
    gate = NavigationReadiness(True, True)
    gate.route(True, 10)
    gate.tracking(True, 10.01)
    assert gate.reason(10.3) == "tracking_timeout"
    gate.tracking(False, 10.31)
    assert gate.reason(10.32) == "tracking_unavailable"


def test_stale_tracker_approval_cannot_approve_a_new_route():
    gate = NavigationReadiness(True, True)
    gate.route(True, 10)
    gate.tracking(True, 9.99)
    assert gate.reason(10.01) == "waiting_for_tracking"


def test_tracking_clock_regression_blocks_execution():
    gate = NavigationReadiness(False, True)
    gate.tracking(True, 10)
    assert gate.reason(9) == "tracking_timeout"


@pytest.mark.parametrize("timeout", [0, -1, math.inf, math.nan])
def test_invalid_tracking_timeout_rejected(timeout):
    with pytest.raises(ValueError):
        NavigationReadiness(timeout=timeout)


def test_large_finite_quaternion_does_not_overflow_to_zero():
    q = normalize_quaternion((1e308, 0, 0, 1e308))
    assert sum(value*value for value in q) == pytest.approx(1.0)
