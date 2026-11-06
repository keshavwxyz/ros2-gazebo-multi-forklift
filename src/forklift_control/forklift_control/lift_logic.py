"""
Lift-control logic without ROS. The unit tests are in test/test_lift_logic.py.

The lift node plans a trapezoidal velocity profile from the start height to the goal. The
joint velocity limit is the maximum speed. The node sends the profile to the trajectory
controller. Then it compares the measured joint position with the profile.
"""

from dataclasses import dataclass
import math
import xml.etree.ElementTree as ET


@dataclass(frozen=True)
class JointLimits:
    lower: float
    upper: float
    velocity: float
    effort: float


def joint_limits_from_urdf(urdf_xml, joint_name):
    """Read the limits of a prismatic or revolute joint from a URDF string."""
    root = ET.fromstring(urdf_xml)
    for joint in root.findall('joint'):
        if joint.get('name') == joint_name:
            limit = joint.find('limit')
            if limit is None:
                raise ValueError(f'joint {joint_name} has no <limit>')
            return JointLimits(lower=float(limit.get('lower', 0.0)),
                               upper=float(limit.get('upper', 0.0)),
                               velocity=float(limit.get('velocity', 0.0)),
                               effort=float(limit.get('effort', 0.0)))
    raise ValueError(f'joint {joint_name} not found in robot_description')


def validate_goal(height, max_velocity, limits, eps=1e-9):
    """Return (accepted, reason). Goals out of range are rejected. They are never clamped."""
    if limits is None:
        return False, 'joint limits unknown (no /robot_description received yet)'
    if not math.isfinite(height):
        return False, 'height is not a finite number'
    if height < limits.lower - eps or height > limits.upper + eps:
        return False, (f'height {height:.3f} m outside joint limits '
                       f'[{limits.lower:.3f}, {limits.upper:.3f}] m')
    if not math.isfinite(max_velocity) or max_velocity < 0.0:
        return False, 'max_velocity must be >= 0'
    if max_velocity > limits.velocity + eps:
        return False, (f'max_velocity {max_velocity:.3f} m/s exceeds joint limit '
                       f'{limits.velocity:.3f} m/s')
    return True, ''


class TrapezoidProfile:
    """Minimum-time 1-D move from rest to rest with |v| <= vmax and |a| <= amax."""

    def __init__(self, start, target, vmax, amax):
        if vmax <= 0 or amax <= 0:
            raise ValueError('vmax and amax must be positive')
        self.start, self.target = start, target
        self.direction = math.copysign(1.0, target - start)
        distance = abs(target - start)
        t_acc = vmax / amax
        # the speed does not get to vmax: triangle profile
        if amax * t_acc * t_acc >= distance:
            t_acc = math.sqrt(distance / amax)
            self.vpeak = amax * t_acc
            t_cruise = 0.0
        else:
            self.vpeak = vmax
            t_cruise = (distance - amax * t_acc * t_acc) / vmax
        self.amax, self.t_acc, self.t_cruise = amax, t_acc, t_cruise
        self.duration = 2 * t_acc + t_cruise

    def at(self, t):
        """Return (position, velocity) at time t after the start."""
        a, ta, tc = self.amax, self.t_acc, self.t_cruise
        t = min(max(t, 0.0), self.duration)
        if t < ta:
            s, v = 0.5 * a * t * t, a * t
        elif t < ta + tc:
            s, v = 0.5 * a * ta * ta + self.vpeak * (t - ta), self.vpeak
        else:
            td = self.duration - t
            s = abs(self.target - self.start) - 0.5 * a * td * td
            v = a * td
        return self.start + self.direction * s, self.direction * v

    def sample(self, dt):
        """Waypoints (t, position, velocity) at each dt seconds. The last one is the goal."""
        n = max(1, math.ceil(self.duration / dt))
        return [(self.duration * k / n, *self.at(self.duration * k / n))
                for k in range(1, n + 1)]


def at_target(position, velocity, target, tolerance, settle_velocity):
    return abs(position - target) <= tolerance and abs(velocity) <= settle_velocity


class TrackingMonitor:
    """Find a fault when |set-point - measured| stays above a limit for too long."""

    def __init__(self, error_limit, max_duration):
        self.error_limit = error_limit
        self.max_duration = max_duration
        self._exceeded_since = None

    def update(self, setpoint, measured, now):
        """Return True when the tracking error was too large for max_duration."""
        if abs(setpoint - measured) <= self.error_limit:
            self._exceeded_since = None
            return False
        if self._exceeded_since is None:
            self._exceeded_since = now
        return now - self._exceeded_since >= self.max_duration


def estimate_load_kg(effort, empty_effort, gravity=9.81):
    """Supported mass from the effort of the lift actuator (equivalent to hydraulic pressure)."""
    return max(0.0, (effort - empty_effort) / gravity)


def moving_mass_from_urdf(urdf_xml, joint_name):
    """Total mass of all links that `joint_name` carries (its child link and descendants)."""
    root = ET.fromstring(urdf_xml)
    children = {}
    child_of_joint = None
    for joint in root.findall('joint'):
        parent = joint.find('parent').get('link')
        child = joint.find('child').get('link')
        children.setdefault(parent, []).append(child)
        if joint.get('name') == joint_name:
            child_of_joint = child
    if child_of_joint is None:
        raise ValueError(f'joint {joint_name} not found in robot_description')
    masses = {}
    for link in root.findall('link'):
        mass = link.find('inertial/mass')
        masses[link.get('name')] = float(mass.get('value')) if mass is not None else 0.0
    total, stack = 0.0, [child_of_joint]
    while stack:
        link = stack.pop()
        total += masses.get(link, 0.0)
        stack.extend(children.get(link, []))
    return total
