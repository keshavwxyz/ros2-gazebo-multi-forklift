"""Reverse safety with the rear LiDAR: thresholds, stopping distance, hysteresis, fail-safe."""

import math

from forklift_control.rear_safety_logic import (decide, max_safe_speed, nearest_behind,
                                                RearSafetyConfig, required_clearance,
                                                scan_points)
import pytest

CFG = RearSafetyConfig()


def behind(d, y=0.0):
    """One obstacle point at d metres behind the rear face."""
    return [(CFG.rear_x - d, y)]


def test_required_clearance_and_max_speed_are_inverse():
    for v in (0.0, 0.05, 0.1, 0.2, 0.4):
        assert max_safe_speed(required_clearance(v, CFG), CFG) == pytest.approx(v)
    assert max_safe_speed(CFG.stop_distance, CFG) == 0.0
    assert max_safe_speed(0.1, CFG) == 0.0


def test_clear_path_passes_reverse_unchanged():
    d = decide(-0.2, 0.0, behind(5.0), True, False, CFG)
    assert (d.state, d.linear_x) == ('CLEAR', -0.2)
    d = decide(-0.2, 0.0, [], True, False, CFG)
    assert d.state == 'CLEAR' and math.isinf(d.nearest)


def test_stop_zone_blocks_reverse_but_not_forward():
    d = decide(-0.2, 0.0, behind(0.30), True, False, CFG)
    assert d.state == 'STOP' and d.linear_x == 0.0 and 'behind' in d.reason
    d = decide(0.2, 0.0, behind(0.30), True, True, CFG)
    assert d.linear_x == 0.2, 'forward motion away from the obstacle must pass'


def test_warning_zone_caps_speed():
    d = decide(-0.3, 0.0, behind(1.0), True, False, CFG)
    assert d.state in ('WARN', 'SLOW') and -d.linear_x <= CFG.warn_speed + 1e-9


def test_speed_is_limited_to_what_can_stop_in_the_gap():
    # 0.37 m: outside the stop zone, but too near for the speed limit of the warning zone (0.39 m).
    d = decide(-0.4, 0.0, behind(0.37), True, False, CFG)
    assert d.state == 'SLOW'
    assert required_clearance(d.linear_x, CFG) <= 0.37 + 1e-9
    assert d.linear_x < 0.0, 'still allowed to creep while outside the stop zone'


def test_hysteresis_needs_extra_clearance_to_leave_stop():
    gap = CFG.stop_distance + CFG.resume_margin / 2
    assert decide(-0.05, 0.0, behind(gap), True, True, CFG).state == 'STOP'
    assert decide(-0.05, 0.0, behind(gap), True, False, CFG).state != 'STOP'
    clear = CFG.stop_distance + CFG.resume_margin + 0.05
    assert decide(-0.05, 0.0, behind(clear), True, True, CFG).state != 'STOP'


def test_obstacles_outside_the_corridor_are_ignored():
    lim = CFG.half_width + CFG.side_margin
    assert math.isinf(nearest_behind(behind(0.2, y=lim + 0.05), CFG))
    assert nearest_behind(behind(0.2, y=lim - 0.05), CFG) == pytest.approx(0.2)
    assert math.isinf(nearest_behind([(0.5, 0.0)], CFG)), 'points ahead are not behind'
    assert math.isinf(nearest_behind(behind(CFG.lookahead + 0.5), CFG))


def test_rotation_blocked_when_the_rear_would_swing_into_an_obstacle():
    side = [(-0.9, 0.85)]               # adjacent to the rear corner, outside the corridor
    d = decide(0.0, 0.4, side, True, False, CFG)
    assert d.angular_z == 0.0 and 'rotation' in d.reason
    assert decide(0.0, 0.4, [(-3.0, 0.0)], True, False, CFG).angular_z == 0.4


def test_missing_scan_is_fail_safe():
    d = decide(-0.1, 0.0, [], False, False, CFG)
    assert d.state == 'FAULT' and d.linear_x == 0.0
    d = decide(0.2, 0.3, [], False, False, CFG)
    assert d.state == 'FAULT' and d.linear_x == 0.2 and d.angular_z == 0.0


def test_scan_points_use_the_sensor_mount():
    # The rear sensor is at x=-1.19 and points rearward. A 1 m return directly in front of the
    # sensor is 1 m behind the sensor in the base frame.
    pts = scan_points([1.0, float('inf'), 0.05], -0.1, 0.1, 0.1, 8.0, (-1.19, 0.0, math.pi))
    assert len(pts) == 1
    x, y = pts[0]
    assert x == pytest.approx(-1.19 - math.cos(0.1)) and y == pytest.approx(math.sin(0.1))


def test_creeping_below_the_minimum_speed_becomes_a_stop():
    gap = CFG.stop_distance + 0.005           # immediately outside the stop zone
    d = decide(-0.2, 0.0, behind(gap), True, False, CFG)
    assert d.state == 'STOP' and d.linear_x == 0.0 and 'too close' in d.reason
