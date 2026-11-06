"""
Cost-based task allocation for the fleet (Python only, no ROS).

There are many possible plans: each assignment of the unassigned tasks to the eligible
robots, in each sequence for each robot (3 tasks x 2 robots = 24 plans). For each plan,
the cost of the sequence of a robot is

  the sum for each of its tasks of
      route(robot position or previous drop-off -> pickup staging)    [m]
    + route(pickup staging -> destination staging)                    [m]
    + switch_penalty     if the task is now planned for a different robot
    + congestion_penalty if a different robot works at or goes to a place near it
    + recovery_penalty * the recent failures of the robot

The sequence starts from the ready position of the robot (its position now, or the end of
its current task) plus its remaining work. The selected plan has the smallest makespan (the
total of the robot with the most work). Then it has the smallest total cost. The units are
metres of travel. The configuration gives the penalties in metres.

The coordinator sends only the first task of the sequence of an idle robot. The other
tasks stay RESERVED. The coordinator plans them again when a robot becomes free. Stability:
the coordinator uses a plan that is different from the current reservations only if it is
better by more than switch_penalty (the cost already contains the switch penalty). Thus,
small changes of the cost do not cause tasks to change between robots. The positions come
from the localization of each robot. The routes come from the Nav2 planners of the robots
(or from a straight line multiplied by a detour factor, if no plan is available).
"""

from dataclasses import dataclass, field
import itertools
import math
from typing import Optional


@dataclass(frozen=True)
class AllocatorConfig:
    switch_penalty: float = 3.0
    congestion_penalty: float = 4.0
    congestion_radius: float = 3.5
    recovery_penalty: float = 5.0
    detour_factor: float = 1.3          # straight-line route length, if no plan is available


@dataclass
class RobotView:
    robot_id: str
    x: float
    y: float
    # the position where it will be free (default: its position now)
    ready_x: Optional[float] = None
    ready_y: Optional[float] = None
    remaining: float = 0.0              # m of work that remain on its current task
    failures: int = 0
    # (x, y) places where it works or to which it goes
    targets: list = field(default_factory=list)

    def ready(self):
        return ((self.x if self.ready_x is None else self.ready_x),
                (self.y if self.ready_y is None else self.ready_y))


@dataclass
class TaskView:
    task_id: str
    pickup: tuple                       # staging (x, y)
    drop: tuple                         # staging (x, y) at the destination
    reserved_for: str = ''


def straight_route(cfg):
    def route(_robot_id, a, b):
        return cfg.detour_factor * math.hypot(b[0] - a[0], b[1] - a[1])
    return route


def task_cost(robot, task, start, others, route, cfg):
    c = route(robot.robot_id, start, task.pickup) + route(robot.robot_id, task.pickup, task.drop)
    if task.reserved_for and task.reserved_for != robot.robot_id:
        c += cfg.switch_penalty
    for o in others:
        if any(math.hypot(t[0] - p[0], t[1] - p[1]) < cfg.congestion_radius
               for t in o.targets for p in (task.pickup, task.drop)):
            c += cfg.congestion_penalty
            break
    return c + cfg.recovery_penalty * robot.failures


def sequence_cost(robot, tasks, robots, route, cfg):
    others = [r for r in robots if r.robot_id != robot.robot_id]
    pos, total = robot.ready(), robot.remaining
    for t in tasks:
        total += task_cost(robot, t, pos, others, route, cfg)
        pos = t.drop
    return total


def plan(robots, tasks, route=None, cfg=AllocatorConfig()):
    """
    Return (assignment, makespan, total). assignment gives robot_id -> ordered list of tasks.

    robots: list of eligible RobotView items; tasks: list of unassigned TaskView items.
    """
    route = route or straight_route(cfg)
    if not robots or not tasks:
        return {r.robot_id: [] for r in robots}, 0.0, 0.0
    best = None
    ids = [r.robot_id for r in robots]
    for owners in itertools.product(range(len(robots)), repeat=len(tasks)):
        groups = [[t for t, o in zip(tasks, owners) if o == i] for i in range(len(robots))]
        seqs = []
        for i, g in enumerate(groups):
            options = [(sequence_cost(robots[i], list(p), robots, route, cfg), list(p))
                       for p in itertools.permutations(g)]
            seqs.append(min(options, key=lambda o: o[0]))
        makespan = max(c for c, _ in seqs)
        total = sum(c for c, _ in seqs)
        # owners: deterministic result when the values are equal
        key = (round(makespan, 6), round(total, 6), owners)
        if best is None or key < best[0]:
            best = (key, {ids[i]: s for i, (_, s) in enumerate(seqs)}, makespan, total)
    return best[1], best[2], best[3]
