"""
Conflict primitives for the fleet (Python only, no ROS).

* ReservationTable: exclusive resources for a limited time (zones around the pickup,
  destination and home places). The table removes expired reservations. Thus, a lost
  robot cannot block a zone permanently. The owner renews its reservation while it uses
  the zone.
* path_conflict: does the planned path of one robot (next `lookahead` m) come nearer than
  `clearance` to the other robot or to the planned path of the other robot?
* choose_priority: deterministic right of way.
    1. A robot that stands in the zone to which the other robot goes has priority. It must
       go out of the zone before the other robot can come in. (If not, the yielding robot
       waits in the path. Measured in Gazebo: a head-on meeting in the home zone of one
       robot caused a livelock.)
    2. A robot that carries a load has priority (a stop with a load is more dangerous).
    3. A robot that is already in the zone that it holds has priority (it is committed).
    4. A robot that moves has priority over a robot that is stopped (thus, only one
       robot must start again).
    5. Stable tie-breaker: the robot with the lower id has priority.
  The decision stays the same for the full conflict: the robots never yield in turns.
* DeadlockMonitor: a conflict is a deadlock if two conditions are true. The yielding
  robot waited longer than `timeout`. The priority robot moved less than `min_progress` m
  in the last `window` s (usually because the yielding robot is in its path).
"""

from dataclasses import dataclass
import math


@dataclass
class Reservation:
    resource: str
    owner: str
    stamp: float
    expires: float
    direction: str = ''


class ReservationTable:

    def __init__(self, ttl=30.0):
        self.ttl = ttl
        self.items = {}

    def holder(self, resource, now):
        r = self.items.get(resource)
        return r.owner if r is not None and r.expires > now else ''

    def grant(self, resource, robot, now, direction=''):
        """Reserve (or renew) a resource. Return (granted, current holder)."""
        holder = self.holder(resource, now)
        if holder and holder != robot:
            return False, holder
        old = self.items.get(resource)
        stamp = old.stamp if old is not None and old.owner == robot and holder else now
        self.items[resource] = Reservation(resource, robot, stamp, now + self.ttl, direction)
        return True, robot

    def release(self, resource, robot):
        r = self.items.get(resource)
        if r is not None and r.owner == robot:
            del self.items[resource]
            return True
        return False

    def release_all(self, robot):
        for k in [k for k, r in self.items.items() if r.owner == robot]:
            del self.items[k]

    def expire(self, now):
        """Remove expired reservations and return them."""
        gone = [r for r in self.items.values() if r.expires <= now]
        for r in gone:
            del self.items[r.resource]
        return gone

    def held_by(self, robot, now):
        return [r.resource for r in self.items.values() if r.owner == robot and r.expires > now]


def trim_path(path, start, lookahead):
    """Points of `path` in the next `lookahead` m of travel, from the point nearest to `start`."""
    if not path:
        return []
    i0 = min(range(len(path)), key=lambda i: math.dist(path[i], start))
    out, travelled = [path[i0]], 0.0
    for a, b in zip(path[i0:], path[i0 + 1:]):
        travelled += math.dist(a, b)
        if travelled > lookahead:
            break
        out.append(b)
    return out


def path_conflict(pose_a, path_a, pose_b, path_b, clearance, lookahead):
    """
    Return (conflict, min_distance, point) if the path of A goes near B or the path of B.

    Poses are (x, y). Paths are lists of (x, y) in the shared map frame.
    """
    ahead_a = trim_path(path_a, pose_a, lookahead)
    ahead_b = trim_path(path_b, pose_b, lookahead) or [pose_b]
    best = (math.inf, None)
    for p in ahead_a:
        for q in [pose_b] + ahead_b:
            d = math.dist(p, q)
            if d < best[0]:
                best = (d, p)
    return best[0] < clearance, best[0], best[1]


def in_zone(pose, zone):
    return math.hypot(pose[0] - zone[0], pose[1] - zone[1]) <= zone[2]


@dataclass
class Contender:
    robot_id: str
    carrying: bool = False
    inside_own_zone: bool = False
    moving: bool = False
    in_others_target: bool = False   # it stands in the zone to which the other robot goes


def choose_priority(a, b):
    """Return (priority_id, yielding_id, rule) for two robots in a conflict."""
    rules = (("must clear the other robot's destination zone", lambda c: c.in_others_target),
             ('carrying a load', lambda c: c.carrying),
             ('already inside its reserved zone', lambda c: c.inside_own_zone),
             ('already moving', lambda c: c.moving))
    for name, f in rules:
        if f(a) != f(b):
            return (a.robot_id, b.robot_id, name) if f(a) else (b.robot_id, a.robot_id, name)
    first, second = sorted((a.robot_id, b.robot_id))
    return first, second, 'robot-id tie-breaker'


class DeadlockMonitor:

    def __init__(self, timeout=40.0, window=15.0, min_progress=0.3):
        self.timeout, self.window, self.min_progress = timeout, window, min_progress
        self.started = None
        self.track = []                 # (t, (x, y)) of the priority robot

    def start(self, now):
        self.started, self.track = now, []

    def update(self, now, priority_pose):
        self.track.append((now, priority_pose))
        self.track = [(t, p) for t, p in self.track if now - t <= self.window]

    def waited(self, now):
        return 0.0 if self.started is None else now - self.started

    def deadlocked(self, now):
        if self.started is None or self.waited(now) < self.timeout or not self.track:
            return False
        if now - self.track[0][0] < self.window * 0.8:
            return False               # the history is not sufficient yet
        moved = max(math.dist(self.track[0][1], p) for _, p in self.track)
        return moved < self.min_progress
