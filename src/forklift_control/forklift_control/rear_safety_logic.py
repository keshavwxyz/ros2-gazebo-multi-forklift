"""
Reverse-motion safety with the rear LiDAR (logic only, no ROS).

The unit tests are in test/test_rear_safety_logic.py.

The robot base frame is base_footprint (x is forward). The protected area is the corridor
behind the rear face: x < rear_x, |y| <= half_width + side_margin. For a reverse command,
the necessary clearance is the stopping distance at that speed plus a fixed margin:

    d_required(v) = stop_distance + |v| * latency + v^2 / (2 * decel)

  * nearest obstacle <= stop_distance                    -> STOP (reverse speed 0)
  * nearest obstacle <  d_required(|v_cmd|)               -> SLOW: largest |v| that
                                                             gives d_required(|v|) <= d
  * nearest obstacle <  warn_distance                     -> WARN: |v| limit is warn_speed
  * all other conditions                                  -> CLEAR
When the robot turns in place, the rear corners move through a circle with the radius
swing_radius around the axle. If an obstacle is in this circle (behind the axle), it also
blocks the rotation. This module never limits forward motion (the collision monitor of
Nav2 protects the front). If there is no new scan -> FAULT: reverse motion and rotation are
blocked (fail-safe). To leave STOP, the clearance must be `resume_margin` larger
(hysteresis). Thus, the robot does not change state quickly at the threshold.
"""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class RearSafetyConfig:
    rear_x: float = -1.23            # rear face (LiDAR housing) (base_footprint)
    half_width: float = 0.49
    side_margin: float = 0.15
    lookahead: float = 3.0           # length of the corridor behind the rear face
    stop_distance: float = 0.35      # m: at this distance or less, always stop reverse motion
    warn_distance: float = 1.2       # m: at this distance or less, decrease the speed
    warn_speed: float = 0.10         # m/s, maximum speed in the warning zone
    latency: float = 0.3             # s: time from the command to the brake reaction
    decel: float = 0.5               # m/s^2, the same as the limit of the drive controller
    resume_margin: float = 0.10      # m of extra clearance that is necessary to leave STOP
    swing_radius: float = 1.35       # m: rear footprint corner that is farthest from the axle
    swing_margin: float = 0.10
    min_creep: float = 0.02          # m/s: a smaller permitted reverse speed is a stop


@dataclass(frozen=True)
class RearDecision:
    state: str                       # CLEAR | WARN | SLOW | STOP | FAULT
    linear_x: float
    angular_z: float
    nearest: float                   # m behind the rear face (inf if there is no obstacle)
    reason: str


def required_clearance(speed, cfg):
    s = abs(speed)
    return cfg.stop_distance + s * cfg.latency + s * s / (2.0 * cfg.decel)


def max_safe_speed(clearance, cfg):
    """Largest reverse speed with a necessary clearance that is not more than `clearance`."""
    room = clearance - cfg.stop_distance
    if room <= 0.0:
        return 0.0
    # s^2 / (2a) + s * latency - room = 0
    a = 1.0 / (2.0 * cfg.decel)
    b = cfg.latency
    return (-b + math.sqrt(b * b + 4.0 * a * room)) / (2.0 * a)


def scan_points(ranges, angle_min, angle_increment, range_min, range_max, sensor_pose):
    """Return the valid scan returns as (x, y) in the base frame. sensor_pose = (x, y, yaw)."""
    sx, sy, syaw = sensor_pose
    pts = []
    for i, r in enumerate(ranges):
        if not (math.isfinite(r) and range_min <= r <= range_max):
            continue
        a = syaw + angle_min + i * angle_increment
        pts.append((sx + r * math.cos(a), sy + r * math.sin(a)))
    return pts


def nearest_behind(points, cfg):
    """Distance from the rear face to the nearest point in the rear corridor (inf if none)."""
    lim = cfg.half_width + cfg.side_margin
    best = math.inf
    for x, y in points:
        if x < cfg.rear_x and abs(y) <= lim and cfg.rear_x - x <= cfg.lookahead:
            best = min(best, cfg.rear_x - x)
    return best


def in_swing_zone(points, cfg):
    r = cfg.swing_radius + cfg.swing_margin
    return any(x < 0.0 and math.hypot(x, y) <= r for x, y in points)


def decide(cmd_v, cmd_w, points, scan_fresh, was_stopped, cfg):
    """Filter one velocity command with the latest rear scan."""
    if not scan_fresh:
        if cmd_v < 0.0 or cmd_w != 0.0:
            return RearDecision('FAULT', max(cmd_v, 0.0), 0.0, math.nan,
                                'no fresh rear LiDAR scan: reverse and rotation blocked')
        return RearDecision('FAULT', cmd_v, cmd_w, math.nan, 'no fresh rear LiDAR scan')
    d = nearest_behind(points, cfg)
    w = cmd_w
    reason = ''
    if w != 0.0 and abs(cmd_v) < 0.05 and in_swing_zone(points, cfg):
        w = 0.0
        reason = 'obstacle inside the rear swing radius: rotation blocked; '
    stop_at = cfg.stop_distance + (cfg.resume_margin if was_stopped else 0.0)
    if d <= stop_at:
        v = max(cmd_v, 0.0)
        return RearDecision('STOP', v, w, d, reason + f'obstacle {d:.2f} m behind '
                            f'(stop zone {cfg.stop_distance:.2f} m)')
    if cmd_v >= 0.0:
        state = 'WARN' if d < cfg.warn_distance else 'CLEAR'
        if reason:
            return RearDecision('STOP', cmd_v, w, d, reason.rstrip('; '))
        return RearDecision(state, cmd_v, w, d, '')
    v = cmd_v
    state = 'CLEAR'
    if d < cfg.warn_distance and -v > cfg.warn_speed:
        v, state = -cfg.warn_speed, 'WARN'
        reason += f'obstacle {d:.2f} m behind: reverse capped at {cfg.warn_speed:.2f} m/s; '
    elif d < cfg.warn_distance:
        state = 'WARN'
    if required_clearance(v, cfg) > d:
        v, state = -max_safe_speed(d, cfg), 'SLOW'
        if -v < cfg.min_creep:
            return RearDecision('STOP', 0.0, w, d, reason + f'obstacle {d:.2f} m behind: '
                                'too close to keep reversing')
        reason += f'obstacle {d:.2f} m behind: reverse limited to {-v:.2f} m/s; '
    return RearDecision(state, v, w, d, reason.rstrip('; '))
