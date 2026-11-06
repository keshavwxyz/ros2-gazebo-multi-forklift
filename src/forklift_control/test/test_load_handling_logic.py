"""Unit tests for the pickup and placement logic (geometry, controller, heights, checks)."""

import math
import pathlib

from forklift_control.load_handling_logic import (approach_command, ApproachGains,
                                                  docking_pose, fork_heights, is_aligned,
                                                  line_errors, load_pose_from_qr,
                                                  load_supported, nominal_load_in_base,
                                                  placement_poses, plausible_load_in_base,
                                                  Pose2D, RobustTarget)
import pytest


def mission():
    from forklift_task_manager.mission_config import load_mission
    path = (pathlib.Path(__file__).resolve().parents[2] / 'forklift_task_manager' / 'config' /
            'warehouse_mission.yaml')
    return load_mission(path)


def test_pose_algebra():
    a = Pose2D(1.0, 2.0, math.pi / 2)
    b = Pose2D(0.5, 0.0, 0.0)
    c = a.compose(b)
    assert (c.x, c.y, c.yaw) == pytest.approx((1.0, 2.5, math.pi / 2))
    back = a.relative(c)
    assert (back.x, back.y, back.yaw) == pytest.approx((0.5, 0.0, 0.0), abs=1e-12)
    ident = a.compose(a.inverse())
    assert (ident.x, ident.y, ident.yaw) == pytest.approx((0, 0, 0), abs=1e-12)


def test_load_pose_from_qr_and_docking():
    # the QR code is 0.30 m in front of the load center, and its face points to +y
    load = load_pose_from_qr(1.0, -4.3, math.pi / 2, 0.30)
    assert (load.x, load.y, load.yaw) == pytest.approx((1.0, -4.6, math.pi / 2))
    dock = docking_pose(load, 0.80)
    assert (dock.x, dock.y) == pytest.approx((1.0, -3.8))
    assert dock.yaw == pytest.approx(-math.pi / 2)


def test_line_error_signs():
    target = Pose2D(0.0, 0.0, 0.0)            # approach along +x and stop at the origin
    e = line_errors(Pose2D(-1.0, 0.2, 0.1), target)
    assert e.remaining == pytest.approx(1.0)
    assert e.lateral == pytest.approx(0.2)    # the robot is to the left of the line
    assert e.heading == pytest.approx(0.1)
    e_rev = line_errors(Pose2D(1.0, 0.0, 0.0), target, reverse=True)
    assert e_rev.remaining == pytest.approx(1.0)


def simulate(start, target, reverse=False, steps=1500, dt=0.05, gains=ApproachGains()):
    x, y, yaw = start.x, start.y, start.yaw
    for _ in range(steps):
        err = line_errors(Pose2D(x, y, yaw), target, reverse)
        v, w = approach_command(err, gains, reverse)
        if v == 0.0:
            break
        x += v * math.cos(yaw) * dt
        y += v * math.sin(yaw) * dt
        yaw += w * dt
    return line_errors(Pose2D(x, y, yaw), target, reverse)


@pytest.mark.parametrize('lateral,heading_deg', [(0.0, 0.0), (0.12, 0.0), (-0.12, 6.0),
                                                 (0.08, -8.0), (0.15, 10.0)])
def test_forward_approach_converges_on_the_line(lateral, heading_deg):
    """With the staging-level errors, the controller is in the insertion tolerance after 0.7 m."""
    target = Pose2D(0.0, 0.0, 0.0)
    final = simulate(Pose2D(-0.7, lateral, math.radians(heading_deg)), target)
    assert abs(final.remaining) < 0.02
    assert is_aligned(final, 0.03, math.radians(2.0)), final


def test_reverse_follows_the_line_back():
    target = Pose2D(-1.0, 0.0, 0.0)            # move rearward 1 m along the same heading
    final = simulate(Pose2D(0.0, 0.02, math.radians(1.0)), target, reverse=True)
    assert abs(final.remaining) < 0.02
    assert abs(final.lateral) < 0.03


def test_command_limits():
    g = ApproachGains()
    v, w = approach_command(line_errors(Pose2D(-5, 2.0, 1.0), Pose2D(0, 0, 0)), g)
    assert 0 < v <= g.max_speed and abs(w) <= g.max_angular
    assert approach_command(line_errors(Pose2D(0.1, 0, 0), Pose2D(0, 0, 0)), g) == (0.0, 0.0)


def test_robust_target_rejects_an_outlier():
    t = RobustTarget(window=5)
    for x in (1.00, 1.01, 0.99, 1.00):
        t.add(Pose2D(x, 0.0, 0.0))
    t.add(Pose2D(3.0, 1.0, 1.0))                # one bad detection
    est = t.estimate()
    assert est.x == pytest.approx(1.0, abs=0.011) and est.y == pytest.approx(0.0)
    assert t.spread() > 1.0


def test_robust_target_yaw_across_pi():
    t = RobustTarget()
    for yaw in (math.pi - 0.01, -math.pi + 0.01, math.pi - 0.02):
        t.add(Pose2D(0, 0, yaw))
    assert abs(abs(t.estimate().yaw) - math.pi) < 0.02


def test_fork_heights_for_the_mission_load():
    m = mission()
    h = fork_heights(m.load, surface_height=0.25, fork_min_height=0.05, fork_thickness=0.04,
                     insert_fork_top=0.07, transport_clearance=0.18, place_clearance=0.05,
                     release_gap=0.05)
    assert h.insert == pytest.approx(0.02)            # fork top 0.07, the pocket is from 0 to 0.12
    assert h.transport == pytest.approx(0.25)         # pallet bottom 0.18 > LiDAR plane 0.12
    assert h.place_raised == pytest.approx(0.37)      # pallet bottom 0.30 above a 0.25 deck
    assert h.release == pytest.approx(0.27)           # fork top 0.32: 5 cm below the deck,
    #                                                   fork bottom 0.28 above the platform


def test_fork_heights_reject_impossible_geometry():
    m = mission()
    with pytest.raises(ValueError):
        fork_heights(m.load, 0.25, 0.05, 0.04, insert_fork_top=0.13,
                     transport_clearance=0.18, place_clearance=0.05, release_gap=0.05)
    with pytest.raises(ValueError):
        fork_heights(m.load, 0.25, 0.05, 0.04, 0.07, 0.18, 0.05, release_gap=0.10)


def test_load_supported_window():
    assert load_supported(29.0, 30.0, 0.5, 2.0)
    assert not load_supported(3.0, 30.0, 0.5, 2.0)    # the forks are not below the deck
    assert not load_supported(90.0, 30.0, 0.5, 2.0)   # the forks are caught on an object


def test_nominal_placement_matches_docking_poses():
    dest = Pose2D(-3.0, 5.3, -math.pi / 2)
    pre, dock = placement_poses(dest, nominal_load_in_base(0.8), 0.5)
    for got, want in ((dock, docking_pose(dest, 0.8)), (pre, docking_pose(dest, 1.3))):
        assert (got.x, got.y) == pytest.approx((want.x, want.y), abs=1e-9)
        assert math.cos(got.yaw - want.yaw) == pytest.approx(1.0)


@pytest.mark.parametrize('depth,lateral,yaw_deg', [(0.864, 0.0, 0.0), (0.80, 0.02, 0.0),
                                                   (0.83, -0.01, 2.0)])
def test_placement_compensates_where_the_load_sits(depth, lateral, yaw_deg):
    dest = Pose2D(3.0, 5.3, -math.pi / 2)
    carried = Pose2D(depth, lateral, math.pi + math.radians(yaw_deg))
    _, dock = placement_poses(dest, carried, 0.5)
    placed = dock.compose(carried)                 # the final position of the load
    assert (placed.x, placed.y) == pytest.approx((dest.x, dest.y), abs=1e-9)
    assert math.cos(placed.yaw - dest.yaw) == pytest.approx(1.0)


def test_plausibility_rejects_impossible_load_poses():
    nominal = nominal_load_in_base(0.8)
    assert plausible_load_in_base(nominal, 0.8, 0.05, 0.2, 0.04, 0.09)
    assert plausible_load_in_base(Pose2D(0.95, 0.03, math.pi), 0.8, 0.05, 0.2, 0.04, 0.09)
    assert not plausible_load_in_base(Pose2D(0.70, 0.0, math.pi), 0.8, 0.05, 0.2, 0.04, 0.09)
    assert not plausible_load_in_base(Pose2D(0.8, 0.06, math.pi), 0.8, 0.05, 0.2, 0.04, 0.09)
    assert not plausible_load_in_base(Pose2D(0.8, 0.0, 0.0), 0.8, 0.05, 0.2, 0.04, 0.09)
