"""
Task registry for the fleet. It is the authoritative source (Python only, no ROS).

There is one task for each box. The owner changes only through validated transitions.
Thus, a maximum of one robot owns a box at a time. If two claims for the same box occur
at the same time, the registry grants exactly one claim. (The coordinator node does all
calls in sequence on one executor thread.)

Task lifecycle:
  AVAILABLE -> RESERVED (planned for a robot) -> ASSIGNED (the robot accepted the goal)
  -> PICKUP_IN_PROGRESS -> PICKED_UP (lift load sensing verified) -> DELIVERY_IN_PROGRESS
  -> DELIVERED (the camera and the lift of the robot verified the placement)
Failure and recovery:
  * The task failed before the lift -> AVAILABLE again (attempts + 1; FAILED after
    max_attempts). A different robot can take it. The box is still at its pickup.
  * The robot is lost or failed while the load can be on its forks (PICKUP_IN_PROGRESS
    after the lift, PICKED_UP, DELIVERY_IN_PROGRESS) -> HELD. The registry never gives
    this task to a different robot automatically. An operator must examine the load
    state before the task can be released.
"""

from dataclasses import dataclass
import itertools

AVAILABLE, RESERVED, ASSIGNED = 'AVAILABLE', 'RESERVED', 'ASSIGNED'
PICKUP_IN_PROGRESS, PICKED_UP = 'PICKUP_IN_PROGRESS', 'PICKED_UP'
DELIVERY_IN_PROGRESS, DELIVERED = 'DELIVERY_IN_PROGRESS', 'DELIVERED'
FAILED, HELD = 'FAILED', 'HELD'

ACTIVE = {ASSIGNED, PICKUP_IN_PROGRESS, PICKED_UP, DELIVERY_IN_PROGRESS}
TERMINAL = {DELIVERED, FAILED}
ALLOWED = {
    AVAILABLE: {RESERVED, ASSIGNED, FAILED},
    RESERVED: {AVAILABLE, ASSIGNED, RESERVED},
    ASSIGNED: {PICKUP_IN_PROGRESS, AVAILABLE, FAILED},
    PICKUP_IN_PROGRESS: {PICKED_UP, AVAILABLE, HELD, FAILED},
    PICKED_UP: {DELIVERY_IN_PROGRESS, DELIVERED, HELD},
    DELIVERY_IN_PROGRESS: {DELIVERED, HELD},
    HELD: {AVAILABLE, FAILED},
    DELIVERED: set(),
    FAILED: set(),
}


class TransitionError(ValueError):
    pass


@dataclass
class Task:
    task_id: str
    box_id: str
    pickup_id: str
    destination_id: str
    state: str = AVAILABLE
    owner: str = ''
    reserved_for: str = ''
    assignment_id: str = ''
    attempts: int = 0
    qr_destination_id: str = ''
    last_confirmed: str = 'at pickup (initial configuration)'
    failure_reason: str = ''
    last_update: float = 0.0


class TaskRegistry:

    def __init__(self, boxes, max_attempts=2, clock=lambda: 0.0):
        """boxes: iterable of (box_id, pickup_id, destination_id), in the mission sequence."""
        self.tasks = {}
        self.order = []
        for box_id, pickup_id, destination_id in boxes:
            t = Task(f'task_{box_id}', box_id, pickup_id, destination_id)
            self.tasks[t.task_id] = t
            self.order.append(t.task_id)
        self.max_attempts = max_attempts
        self.clock = clock
        self._ids = itertools.count(1)
        self.log = []                   # (time, task_id, old, new, robot, reason)

    # ------------------------------------------------------------------ queries

    def by_box(self, box_id):
        for t in self.tasks.values():
            if t.box_id == box_id:
                return t
        raise KeyError(box_id)

    def owned_by(self, robot):
        return [t for t in self.tasks.values() if t.owner == robot and t.state in ACTIVE]

    def unassigned(self):
        """Tasks that the coordinator can still give to a robot (available or only planned)."""
        return [self.tasks[i] for i in self.order
                if self.tasks[i].state in (AVAILABLE, RESERVED)]

    def finished(self):
        return all(t.state in TERMINAL | {HELD} for t in self.tasks.values())

    def all_delivered(self):
        return all(t.state == DELIVERED for t in self.tasks.values())

    # ------------------------------------------------------------------ transitions

    def _set(self, t, new, robot, reason):
        if new not in ALLOWED[t.state]:
            raise TransitionError(f'{t.task_id}: {t.state} -> {new} not allowed')
        self.log.append((self.clock(), t.task_id, t.state, new, robot, reason))
        t.state = new
        t.last_update = self.clock()

    def reserve(self, task_id, robot):
        """Plan a task for a robot (not sent yet). This replaces a different plan."""
        t = self.tasks[task_id]
        if t.state not in (AVAILABLE, RESERVED):
            raise TransitionError(f'{task_id} is {t.state}: cannot reserve')
        if t.reserved_for != robot:
            self._set(t, RESERVED, robot, 'planned')
            t.reserved_for = robot

    def unreserve(self, task_id):
        t = self.tasks[task_id]
        if t.state == RESERVED:
            self._set(t, AVAILABLE, t.reserved_for, 'plan dropped')
            t.reserved_for = ''

    def claim(self, box_id, robot):
        """
        Give `robot` the ownership of the task of the box. Return (granted, task, reason).

        The registry grants the task only if two conditions are true. The task is available (or
        planned for this robot). The robot owns no other active task.
        """
        try:
            t = self.by_box(box_id)
        except KeyError:
            return False, None, f'unknown box {box_id!r}'
        if t.state in ACTIVE or t.state == HELD:
            return False, t, f'{box_id} already owned by {t.owner} ({t.state})'
        if t.state in TERMINAL:
            return False, t, f'{box_id} is {t.state}'
        if t.state == RESERVED and t.reserved_for != robot:
            return False, t, f'{box_id} is reserved for {t.reserved_for}'
        busy = self.owned_by(robot)
        if busy:
            return False, t, f'{robot} already owns {busy[0].task_id}'
        self._set(t, ASSIGNED, robot, 'claimed')
        t.owner, t.reserved_for = robot, ''
        t.assignment_id = f'{t.task_id}#{next(self._ids)}'
        t.failure_reason = ''
        return True, t, 'granted'

    def advance(self, task_id, new, robot, reason=''):
        """Progress that the owner reports (PICKUP_IN_PROGRESS, PICKED_UP, ..., DELIVERED)."""
        t = self.tasks[task_id]
        if t.owner != robot:
            raise TransitionError(f'{robot} does not own {task_id} (owner {t.owner!r})')
        if new == t.state:
            return
        # The registry can receive the progress late (for example, directly from
        # PICKUP_IN_PROGRESS to delivered). Go through each step of the chain in sequence.
        # Thus, the registry validates and records each intermediate step.
        chain = [ASSIGNED, PICKUP_IN_PROGRESS, PICKED_UP, DELIVERY_IN_PROGRESS, DELIVERED]
        if t.state in chain and new in chain and chain.index(new) > chain.index(t.state):
            for step in chain[chain.index(t.state) + 1:chain.index(new) + 1]:
                self._set(t, step, robot, reason)
        else:
            self._set(t, new, robot, reason)
        if new == DELIVERED:
            t.last_confirmed = f'placement verified by {robot}'
        elif new == PICKED_UP:
            t.last_confirmed = f'on the forks of {robot} (load sensed)'

    def release(self, task_id, robot, reason):
        """Set a task to AVAILABLE (or FAILED) if its owner stopped it before the lift."""
        t = self.tasks[task_id]
        if t.owner != robot:
            raise TransitionError(f'{robot} does not own {task_id}')
        if t.state not in (ASSIGNED, PICKUP_IN_PROGRESS):
            raise TransitionError(f'{task_id} is {t.state}: the load may be on the forks; '
                                  'use hold()')
        t.attempts += 1
        t.failure_reason = reason
        t.owner = ''
        if t.attempts >= self.max_attempts:
            self._set(t, FAILED, robot, f'{reason} (attempt {t.attempts})')
        else:
            self._set(t, AVAILABLE, robot, reason)

    def hold(self, task_id, reason):
        """Freeze the task if the load state is not known (it keeps its owner for the record)."""
        t = self.tasks[task_id]
        if t.state != HELD:
            self._set(t, HELD, t.owner, reason)
            t.failure_reason = reason

    def operator_release(self, task_id, reason, box_at_pickup):
        """After an operator check: set the task to AVAILABLE again if the box is at its pickup."""
        t = self.tasks[task_id]
        if t.state != HELD:
            raise TransitionError(f'{task_id} is not HELD')
        t.owner = ''
        self._set(t, AVAILABLE if box_at_pickup else FAILED, 'operator', reason)
