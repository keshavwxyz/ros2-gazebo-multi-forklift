"""
Mission state machine and mission record (Python only, no ROS).

The ROS node (mission_node.py) runs the current state and reports its result. This module
selects the next state, applies the policies for new tries and failures, and keeps the
mission record. The unit tests in test/test_mission_fsm.py simulate the results.

Normal flow:
  IDLE -> INITIALIZE -> CHECK_SYSTEMS -> LOCALIZE -> SELECT_NEXT_BOX
  -> NAVIGATE_TO_PICKUP -> DETECT_AND_IDENTIFY_BOX -> PICKUP
       (skill phases shown as APPROACH_BOX / ALIGN_FORKS / PICKUP / VERIFY_PICKUP)
  -> NAVIGATE_TO_DESTINATION -> PLACE_BOX
       (skill phases shown as ALIGN_FOR_PLACEMENT / PLACE_BOX / VERIFY_PLACEMENT)
  -> UPDATE_MISSION_STATE -> SELECT_NEXT_BOX ... -> RETURN_HOME -> MISSION_COMPLETE
Failure and recovery:
  - Each state has a number of new tries.
  - If the robot cannot pick up a box, the box is marked as failed. The mission continues
    with the next box.
  - If the load state is not known after a failure (failure during transport, actuator
    fault), the mission goes to MISSION_FAILED and the robot stops.
PAUSED:
  - PAUSED stops navigation states immediately. They run again after resume.
  - During a manipulation (PICKUP / PLACE_BOX), the pause occurs at the next state
    boundary. A stop with the forks in a pallet is more dangerous than the end of a
    short, supervised skill.
EMERGENCY_STOP:
  - EMERGENCY_STOP is always immediate. After an interrupted manipulation, resume is
    not permitted (MISSION_FAILED), because the load state is then not known.
"""

from dataclasses import dataclass, field
from enum import Enum


class S(str, Enum):
    IDLE = 'IDLE'
    INITIALIZE = 'INITIALIZE'
    CHECK_SYSTEMS = 'CHECK_SYSTEMS'
    LOCALIZE = 'LOCALIZE'
    SELECT_NEXT_BOX = 'SELECT_NEXT_BOX'
    NAVIGATE_TO_PICKUP = 'NAVIGATE_TO_PICKUP'
    DETECT_AND_IDENTIFY_BOX = 'DETECT_AND_IDENTIFY_BOX'
    PICKUP = 'PICKUP'
    NAVIGATE_TO_DESTINATION = 'NAVIGATE_TO_DESTINATION'
    PLACE_BOX = 'PLACE_BOX'
    UPDATE_MISSION_STATE = 'UPDATE_MISSION_STATE'
    RETURN_HOME = 'RETURN_HOME'
    MISSION_COMPLETE = 'MISSION_COMPLETE'
    MISSION_FAILED = 'MISSION_FAILED'
    PAUSED = 'PAUSED'
    EMERGENCY_STOP = 'EMERGENCY_STOP'
    CANCELED = 'CANCELED'


TERMINAL = {S.MISSION_COMPLETE, S.MISSION_FAILED, S.CANCELED}
INTERRUPTIBLE_NOW = {S.INITIALIZE, S.CHECK_SYSTEMS, S.LOCALIZE, S.NAVIGATE_TO_PICKUP,
                     S.DETECT_AND_IDENTIFY_BOX, S.NAVIGATE_TO_DESTINATION, S.RETURN_HOME}
MANIPULATION = {S.PICKUP, S.PLACE_BOX}

# Skill feedback phase -> more specific state name in the mission status.
PICKUP_PHASES = {'navigate_to_staging': 'NAVIGATE_TO_PICKUP',
                 'identify': 'DETECT_AND_IDENTIFY_BOX', 'align': 'APPROACH_BOX',
                 'back_off': 'APPROACH_BOX', 'insert': 'ALIGN_FORKS', 'lift': 'PICKUP',
                 'verify_load': 'VERIFY_PICKUP', 'back_out': 'VERIFY_PICKUP'}
PLACE_PHASES = {'navigate_to_staging': 'NAVIGATE_TO_DESTINATION',
                'forks_to_placement_height': 'ALIGN_FOR_PLACEMENT',
                'align': 'ALIGN_FOR_PLACEMENT', 'insert': 'PLACE_BOX', 'lower': 'PLACE_BOX',
                'retract': 'VERIFY_PLACEMENT', 'verify': 'VERIFY_PLACEMENT'}

# Failure codes for each state:
#   - Codes for temporary failures. A new try is possible.
#   - Codes after which the load state or the robot state is not known. The mission must stop.
RETRYABLE = {
    S.INITIALIZE: {'SYSTEMS_UNAVAILABLE', 'TIMEOUT'},
    S.CHECK_SYSTEMS: {'SENSOR_UNAVAILABLE', 'TIMEOUT'},
    S.LOCALIZE: {'LOCALIZATION_FAILED', 'TIMEOUT'},
    S.NAVIGATE_TO_PICKUP: {'NAVIGATION_FAILED', 'TIMEOUT'},
    S.DETECT_AND_IDENTIFY_BOX: {'BOX_NOT_VISIBLE', 'QR_UNREADABLE', 'TIMEOUT'},
    S.PICKUP: {'NOT_STATIONARY', 'BOX_NOT_VISIBLE', 'QR_UNREADABLE', 'ALIGNMENT_FAILED',
               'LOCALIZATION_UNAVAILABLE', 'NAVIGATION_FAILED', 'PICKUP_NOT_VERIFIED',
               'TIMEOUT'},
    S.NAVIGATE_TO_DESTINATION: {'NAVIGATION_FAILED', 'TIMEOUT'},
    S.PLACE_BOX: {'NOT_STATIONARY', 'ALIGNMENT_FAILED', 'DESTINATION_NOT_FOUND',
                  'LOCALIZATION_UNAVAILABLE', 'NAVIGATION_FAILED'},
    S.RETURN_HOME: {'NAVIGATION_FAILED', 'TIMEOUT'},
}
UNSAFE = {'LIFT_FAILED', 'INSERTION_BLOCKED', 'PLACEMENT_NOT_VERIFIED', 'NOT_CARRYING',
          'LOAD_STATE_UNKNOWN'}


@dataclass
class Outcome:
    ok: bool
    code: str = ''
    message: str = ''
    destination_id: str = ''       # from DETECT_AND_IDENTIFY_BOX (decoded QR payload)
    box_id: str = ''


@dataclass
class MissionRecord:
    boxes_total: int = 0
    boxes_detected: list = field(default_factory=list)
    boxes_picked: list = field(default_factory=list)
    boxes_delivered: list = field(default_factory=list)
    boxes_failed: dict = field(default_factory=dict)       # box_id -> reason
    tasks: dict = field(default_factory=dict)              # box_id -> destination (from QR)
    failure_reason: str = ''
    events: list = field(default_factory=list)             # (state, summary of the result)


class Mission:
    """Selects the transitions. This class never does I/O."""

    def __init__(self, box_ids, retries, return_home=True):
        # deterministic sequence (sequence of the config)
        self.queue = list(box_ids)
        self.retries = dict(retries)                    # state name -> maximum number of new tries
        self.record = MissionRecord(boxes_total=len(self.queue))
        self.state = S.IDLE
        self.current_box = ''
        self.carrying = False
        self.attempts = {}
        self.resume_state = None
        self.pause_pending = False
        self.return_home = return_home         # False: a fleet coordinator makes this decision

    # ------------------------------------------------------------------ queries

    @property
    def current_destination(self):
        return self.record.tasks.get(self.current_box, '')

    @property
    def finished(self):
        return self.state in TERMINAL

    @property
    def success(self):
        return (self.state == S.MISSION_COMPLETE and
                len(self.record.boxes_delivered) == self.record.boxes_total)

    def pending(self):
        done = set(self.record.boxes_delivered) | set(self.record.boxes_failed)
        return [b for b in self.queue if b not in done]

    # ------------------------------------------------------------------ control

    def start(self):
        if self.state != S.IDLE:
            raise RuntimeError(f'cannot start from {self.state.value}')
        self.state = S.INITIALIZE

    def interrupt(self, kind):
        """
        Pause ('pause') or emergency stop ('estop').

        Return True if it is necessary to cancel now the action that runs the current state.
        """
        if self.finished or self.state == S.IDLE:
            return False
        if self.state == S.EMERGENCY_STOP or (self.state == S.PAUSED and kind == 'pause'):
            return False
        if kind == 'pause' and self.state in MANIPULATION:
            self.pause_pending = True
            return False
        if self.state != S.PAUSED:                 # PAUSED -> ESTOP keeps the paused state
            self.resume_state = self.state
        self.state = S.PAUSED if kind == 'pause' else S.EMERGENCY_STOP
        return True

    def resume(self, interrupted_manipulation=False):
        """Go back to the interrupted state (it runs again from its start)."""
        self.pause_pending = False
        if self.state not in (S.PAUSED, S.EMERGENCY_STOP) or self.resume_state is None:
            return self.state
        target, self.resume_state = self.resume_state, None
        if interrupted_manipulation and target in MANIPULATION:
            self.state = S.MISSION_FAILED
            self.record.failure_reason = (f'{target.value} interrupted mid-way; load state '
                                          'unknown, operator check needed')
        else:
            self.state = target
        return self.state

    def cancel(self, reason='cancelled'):
        if not self.finished:
            self.state = S.CANCELED
            self.record.failure_reason = reason

    # ------------------------------------------------------------------ transitions

    def advance(self, outcome):
        """Apply the result of the current state. Return the new state."""
        state = self.state
        self.record.events.append((state.value, 'ok' if outcome.ok else
                                   f'{outcome.code}: {outcome.message}'))
        if outcome.ok:
            self.attempts.pop(state, None)
            new = self._next_on_success(state, outcome)
        else:
            new = self._next_on_failure(state, outcome)
        if self.pause_pending and new not in TERMINAL:
            self.pause_pending = False
            self.resume_state, new = new, S.PAUSED
        self.state = new
        return self.state

    def _next_on_success(self, state, outcome):
        if state == S.INITIALIZE:
            return S.CHECK_SYSTEMS
        if state == S.CHECK_SYSTEMS:
            return S.LOCALIZE
        if state == S.LOCALIZE:
            return S.SELECT_NEXT_BOX
        if state == S.SELECT_NEXT_BOX:
            pending = self.pending()
            if not pending:
                self.current_box = ''
                if not self.return_home:
                    return self._finish()
                return S.RETURN_HOME
            self.current_box = pending[0]
            return S.NAVIGATE_TO_PICKUP
        if state == S.NAVIGATE_TO_PICKUP:
            return S.DETECT_AND_IDENTIFY_BOX
        if state == S.DETECT_AND_IDENTIFY_BOX:
            if self.current_box not in self.record.boxes_detected:
                self.record.boxes_detected.append(self.current_box)
            # put the task in the queue
            self.record.tasks[self.current_box] = outcome.destination_id
            return S.PICKUP
        if state == S.PICKUP:
            self.carrying = True
            self.record.boxes_picked.append(self.current_box)
            return S.NAVIGATE_TO_DESTINATION
        if state == S.NAVIGATE_TO_DESTINATION:
            return S.PLACE_BOX
        if state == S.PLACE_BOX:
            self.carrying = False
            return S.UPDATE_MISSION_STATE
        if state == S.UPDATE_MISSION_STATE:
            self.record.boxes_delivered.append(self.current_box)
            return S.SELECT_NEXT_BOX
        if state == S.RETURN_HOME:
            return self._finish()
        raise RuntimeError(f'no success transition from {state.value}')

    def _finish(self):
        if self.record.boxes_failed:
            self.record.failure_reason = 'not delivered: ' + ', '.join(
                f'{b} ({r})' for b, r in self.record.boxes_failed.items())
            return S.MISSION_FAILED
        return S.MISSION_COMPLETE

    def _next_on_failure(self, state, outcome):
        reason = f'{state.value}: {outcome.code} {outcome.message}'.strip()
        if outcome.code in UNSAFE:
            self.record.failure_reason = reason + ' (unsafe to continue)'
            return S.MISSION_FAILED
        if outcome.code in RETRYABLE.get(state, set()):
            used = self.attempts.get(state, 0)
            if used < self.retries.get(state.value, 0):
                self.attempts[state] = used + 1
                return state                                # try the same state again
        self.attempts.pop(state, None)
        # No more new tries, or a new try is not applicable.
        if self.carrying or state in (S.INITIALIZE, S.CHECK_SYSTEMS, S.LOCALIZE,
                                      S.RETURN_HOME):
            self.record.failure_reason = reason
            return S.MISSION_FAILED
        # The robot carries no load: stop the work on this box and continue with the next box.
        self.record.boxes_failed[self.current_box] = f'{outcome.code} at {state.value}'
        return S.SELECT_NEXT_BOX
