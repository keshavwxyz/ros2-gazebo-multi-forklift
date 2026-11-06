"""Unit tests for the lift logic without ROS."""

import math

from forklift_control.lift_logic import (at_target, estimate_load_kg, joint_limits_from_urdf,
                                         JointLimits, moving_mass_from_urdf, TrackingMonitor,
                                         TrapezoidProfile, validate_goal)
import pytest

LIMITS = JointLimits(lower=0.0, upper=1.0, velocity=0.2, effort=10000.0)

URDF = """<robot name="r">
  <link name="a"/><link name="b"/>
  <joint name="fork_lift_joint" type="prismatic">
    <parent link="a"/><child link="b"/><axis xyz="0 0 1"/>
    <limit lower="0.0" upper="1.0" velocity="0.2" effort="10000"/>
  </joint>
</robot>"""


def test_limits_parsed_from_urdf():
    assert joint_limits_from_urdf(URDF, 'fork_lift_joint') == LIMITS


def test_missing_joint_raises():
    with pytest.raises(ValueError, match='not found'):
        joint_limits_from_urdf(URDF, 'nope')


@pytest.mark.parametrize('height', [0.0, 0.5, 1.0])
def test_goals_inside_limits_accepted(height):
    assert validate_goal(height, 0.15, LIMITS) == (True, '')


@pytest.mark.parametrize('height', [-0.01, 1.01, math.nan, math.inf])
def test_goals_outside_limits_rejected_not_clamped(height):
    ok, reason = validate_goal(height, 0.15, LIMITS)
    assert not ok and reason


def test_velocity_above_joint_limit_rejected():
    ok, reason = validate_goal(0.5, 0.5, LIMITS)
    assert not ok and 'exceeds joint limit' in reason


def test_goal_rejected_without_limits():
    ok, reason = validate_goal(0.5, 0.15, None)
    assert not ok and 'unknown' in reason


@pytest.mark.parametrize('start,target', [(0.0, 1.0), (1.0, 0.2), (0.3, 0.32), (0.5, 0.5)])
def test_trapezoid_respects_limits_and_reaches_goal(start, target):
    vmax, amax = 0.15, 0.3
    prof = TrapezoidProfile(start, target, vmax, amax)
    times = [prof.duration * k / 400 for k in range(401)]
    samples = [prof.at(t) for t in times]
    assert samples[0] == pytest.approx((start, 0.0))
    assert samples[-1][0] == pytest.approx(target)
    assert samples[-1][1] == pytest.approx(0.0, abs=1e-9)
    assert all(abs(v) <= vmax + 1e-9 for _, v in samples)
    dt = times[1] - times[0] if prof.duration > 0 else 1.0
    accels = [abs(b[1] - a[1]) / dt for a, b in zip(samples, samples[1:])]
    assert all(a <= amax * 1.01 for a in accels)
    # the value always moves to the target (no overshoot)
    sign = 1 if target >= start else -1
    assert all(sign * (b[0] - a[0]) >= -1e-12 for a, b in zip(samples, samples[1:]))


def test_trapezoid_durations():
    # long move: 0.5 s acceleration + cruise + 0.5 s
    # deceleration; distance 1.0 -> cruise (1-0.075)/0.15 s
    long_move = TrapezoidProfile(0.0, 1.0, 0.15, 0.3)
    assert long_move.vpeak == pytest.approx(0.15)
    assert long_move.duration == pytest.approx(1.0 + 0.925 / 0.15)
    short = TrapezoidProfile(0.0, 0.03, 0.15, 0.3)     # triangle profile: vpeak < vmax
    assert short.vpeak < 0.15
    assert short.duration == pytest.approx(2 * (0.03 / 0.3) ** 0.5)


def test_trapezoid_sample_ends_at_goal():
    pts = TrapezoidProfile(0.0, 0.4, 0.15, 0.3).sample(0.1)
    assert pts[-1][1:] == pytest.approx((0.4, 0.0))
    assert all(b[0] > a[0] for a, b in zip(pts, pts[1:]))
    assert TrapezoidProfile(0.2, 0.2, 0.15, 0.3).sample(0.1) == [(0.0, 0.2, 0.0)]


def test_trapezoid_rejects_bad_limits():
    with pytest.raises(ValueError):
        TrapezoidProfile(0.0, 1.0, 0.0, 0.3)


def test_at_target_requires_position_and_rest():
    assert at_target(0.501, 0.0, 0.5, 0.005, 0.01)
    assert not at_target(0.51, 0.0, 0.5, 0.005, 0.01)
    assert not at_target(0.5, 0.05, 0.5, 0.005, 0.01)


def test_tracking_monitor_needs_sustained_error():
    m = TrackingMonitor(error_limit=0.05, max_duration=1.0)
    assert not m.update(0.5, 0.4, now=0.0)
    assert not m.update(0.5, 0.4, now=0.9)
    assert not m.update(0.5, 0.49, now=1.0)    # the error is gone -> the timer starts again
    assert not m.update(0.5, 0.4, now=1.5)
    assert m.update(0.5, 0.4, now=2.6)


def test_load_estimate():
    assert estimate_load_kg(1000.0 + 30 * 9.81, 1000.0) == pytest.approx(30.0)
    assert estimate_load_kg(900.0, 1000.0) == 0.0


def test_moving_mass_sums_the_lift_subtree():
    urdf = """<robot name="r">
      <link name="base"><inertial><mass value="500"/></inertial></link>
      <link name="carriage"><inertial><mass value="60"/></inertial></link>
      <link name="fork_l"><inertial><mass value="25"/></inertial></link>
      <link name="fork_r"><inertial><mass value="25"/></inertial></link>
      <link name="tip"/>
      <joint name="fork_lift_joint" type="prismatic"><parent link="base"/>
        <child link="carriage"/><limit lower="0" upper="1" velocity="0.2" effort="1"/></joint>
      <joint name="jl" type="fixed"><parent link="carriage"/><child link="fork_l"/></joint>
      <joint name="jr" type="fixed"><parent link="carriage"/><child link="fork_r"/></joint>
      <joint name="jt" type="fixed"><parent link="fork_l"/><child link="tip"/></joint>
    </robot>"""
    assert moving_mass_from_urdf(urdf, 'fork_lift_joint') == pytest.approx(110.0)


def test_moving_mass_of_the_real_robot():
    try:
        from forklift_description.urdf_kinematics import generate_urdf
    except ImportError:
        pytest.skip('forklift_description not available')
    # carriage 60 kg + 2 forks x 25 kg: this agrees with the
    # 1079.1 N that Gazebo gives with empty forks
    assert moving_mass_from_urdf(generate_urdf(), 'fork_lift_joint') == pytest.approx(110.0)
