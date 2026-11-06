"""
Pickup and placement logic without ROS. Unit tests: test/test_load_handling_logic.py.

Conventions: planar poses (x, y, yaw) are in one fixed frame (odom during a manoeuvre). The
load frame is the frame of the mission (center of the pallet footprint on the floor, +x = QR
face). A "docking" pose puts the robot base on the +x axis of the load, with the robot
pointed at the load.
"""

from dataclasses import dataclass
import math
import statistics


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float

    def compose(self, other):
        """Return self ∘ other (other is in the frame of self)."""
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return Pose2D(self.x + c * other.x - s * other.y, self.y + s * other.x + c * other.y,
                      wrap(self.yaw + other.yaw))

    def inverse(self):
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return Pose2D(-c * self.x - s * self.y, s * self.x - c * self.y, wrap(-self.yaw))

    def relative(self, other):
        """Return other in the frame of self."""
        return self.inverse().compose(other)


def nominal_load_in_base(dock_standoff):
    """Load pose in the base frame when the forks go in exactly as planned."""
    return Pose2D(dock_standoff, 0.0, math.pi)


def placement_poses(dest_load, load_in_base, pre_back):
    """
    Robot (pre, dock) poses that put a carried load down exactly at dest_load.

    load_in_base is the real position of the load on the forks (measured at the pickup).
    Thus, a change in the insertion depth does not cause a placement error. pre is
    pre_back metres behind dock, along the heading of the robot.
    """
    dock = dest_load.compose(load_in_base.inverse())
    return dock.compose(Pose2D(-pre_back, 0.0, 0.0)), dock


def plausible_load_in_base(load_in_base, dock_standoff, short_tol, long_tol, lateral_tol,
                           yaw_tol):
    """Return True if a measured pose of the load on the forks is physically possible."""
    return (dock_standoff - short_tol <= load_in_base.x <= dock_standoff + long_tol and
            abs(load_in_base.y) <= lateral_tol and
            abs(wrap(load_in_base.yaw - math.pi)) <= yaw_tol)


def load_pose_from_qr(qr_x, qr_y, normal_yaw, qr_offset_x):
    """Load frame from the QR center and the normal of its face (the +x axis of the load)."""
    return Pose2D(qr_x - qr_offset_x * math.cos(normal_yaw),
                  qr_y - qr_offset_x * math.sin(normal_yaw), wrap(normal_yaw))


def docking_pose(load, standoff):
    """Robot base pose at `standoff` metres along the +x axis of the load, pointed at the load."""
    return load.compose(Pose2D(standoff, 0.0, math.pi))


@dataclass(frozen=True)
class LineErrors:
    remaining: float   # remaining distance along the target heading (for reverse motion: rearward)
    lateral: float     # offset of the robot to the left of the target line, m
    heading: float     # robot yaw - target yaw, rad


def line_errors(robot, target, reverse=False):
    rel = target.relative(robot)          # robot in the target frame
    remaining = rel.x if reverse else -rel.x
    return LineErrors(remaining, rel.y, rel.yaw)


@dataclass(frozen=True)
class ApproachGains:
    max_speed: float = 0.12
    min_speed: float = 0.03
    speed_gain: float = 0.8           # v = gain * remaining distance, with limits
    heading_gain: float = 2.0
    lateral_gain: float = 5.0         # necessary heading = -atan(lateral_gain * lateral)
    max_angular: float = 0.4          # (the gains remove 15 cm / 10 deg in 0.7 m)


def approach_command(err, gains, reverse=False):
    """
    (v, w) to get to a target pose along its line.

    The robot moves along the target line. It turns its heading to -atan(k * lateral)
    (for reverse motion: +atan). For a unicycle, this converges on the line. The speed
    decreases with the remaining distance. The speed is zero when the robot is at the target.
    """
    if err.remaining <= 0.0:
        return 0.0, 0.0
    speed = min(gains.max_speed, max(gains.min_speed, gains.speed_gain * err.remaining))
    v = -speed if reverse else speed
    desired = math.atan(gains.lateral_gain * err.lateral) * (1.0 if reverse else -1.0)
    w = gains.heading_gain * wrap(desired - err.heading)
    w = max(-gains.max_angular, min(gains.max_angular, w))
    return v, w


def is_aligned(err, lateral_tol, heading_tol):
    return abs(err.lateral) <= lateral_tol and abs(err.heading) <= heading_tol


class RobustTarget:
    """Median of the latest load-pose estimates (this rejects one bad detection)."""

    def __init__(self, window=10):
        self.window = window
        self.samples = []

    def add(self, pose):
        self.samples.append(pose)
        self.samples = self.samples[-self.window:]

    def __len__(self):
        return len(self.samples)

    def estimate(self):
        if not self.samples:
            return None
        ref = self.samples[-1].yaw
        # unwrap the angle near the latest value
        yaws = [ref + wrap(p.yaw - ref) for p in self.samples]
        return Pose2D(statistics.median(p.x for p in self.samples),
                      statistics.median(p.y for p in self.samples),
                      wrap(statistics.median(yaws)))

    def spread(self):
        """Maximum distance from a sample to the median (consistency check)."""
        est = self.estimate()
        return max(math.hypot(p.x - est.x, p.y - est.y) for p in self.samples) if est else 0.0


@dataclass(frozen=True)
class ForkHeights:
    """Lift joint positions for each phase (joint q = fork top height - fork_min_height)."""

    insert: float
    transport: float
    place_raised: float
    release: float


def fork_heights(load, surface_height, fork_min_height, fork_thickness, insert_fork_top,
                 transport_clearance, place_clearance, release_gap):
    """
    Calculate the lift set-points from the load geometry and the robot geometry.

    insert:       fork top at insert_fork_top (it must go below the deck and above the floor)
    transport:    pallet bottom at transport_clearance above the floor
    place_raised: pallet bottom at place_clearance above the destination surface
    release:      fork top at release_gap below the deck underside when the pallet is on the
                  surface (the fork bottom must stay above that surface)
    """
    pocket = load.pocket_height                  # pallet bottom -> deck underside
    if not fork_thickness < insert_fork_top < pocket:
        raise ValueError('insertion fork top must be between the floor and the deck underside')
    release_top = surface_height + pocket - release_gap
    if release_top - fork_thickness <= surface_height:
        raise ValueError('release height would put the forks into the surface')
    return ForkHeights(
        insert=insert_fork_top - fork_min_height,
        transport=transport_clearance + pocket - fork_min_height,
        place_raised=surface_height + place_clearance + pocket - fork_min_height,
        release=release_top - fork_min_height)


def load_supported(estimated_kg, expected_kg, low_fraction, high_fraction):
    return low_fraction * expected_kg <= estimated_kg <= high_fraction * expected_kg
