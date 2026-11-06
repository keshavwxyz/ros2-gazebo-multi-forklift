"""Mission state machine: normal flow, new tries, recovery, pause / e-stop / cancel."""

from forklift_task_manager.mission_fsm import Mission, Outcome, S
import pytest

BOXES = ['BOX_A', 'BOX_B', 'BOX_C']
DEST = {'BOX_A': 'SHELF_01', 'BOX_B': 'SHELF_03', 'BOX_C': 'SHELF_02'}
RETRIES = {'INITIALIZE': 2, 'CHECK_SYSTEMS': 2, 'LOCALIZE': 2, 'NAVIGATE_TO_PICKUP': 2,
           'DETECT_AND_IDENTIFY_BOX': 2, 'PICKUP': 1, 'NAVIGATE_TO_DESTINATION': 2,
           'PLACE_BOX': 1, 'RETURN_HOME': 2}


def ok(m):
    """Return the result that a successful run of the current state would report."""
    if m.state == S.DETECT_AND_IDENTIFY_BOX:
        return Outcome(True, destination_id=DEST[m.current_box], box_id=m.current_box)
    return Outcome(True)


def run(m, faults=None, limit=200):
    """Run the mission until a terminal state. faults: {(state, box): [codes...]}."""
    faults = {k: list(v) for k, v in (faults or {}).items()}
    trace = []
    for _ in range(limit):
        if m.finished:
            return trace
        trace.append((m.state, m.current_box))
        pending = faults.get((m.state, m.current_box))
        m.advance(Outcome(False, pending.pop(0), 'injected') if pending else ok(m))
    raise AssertionError('mission did not terminate')


def started():
    m = Mission(BOXES, RETRIES)
    m.start()
    return m


def test_nominal_mission_delivers_all_boxes_in_order_and_returns_home():
    m = started()
    trace = run(m)
    assert m.state == S.MISSION_COMPLETE and m.success
    assert m.record.boxes_delivered == BOXES
    assert m.record.boxes_picked == BOXES and m.record.boxes_detected == BOXES
    assert m.record.tasks == DEST and not m.record.boxes_failed
    assert trace[:4] == [(S.INITIALIZE, ''), (S.CHECK_SYSTEMS, ''), (S.LOCALIZE, ''),
                         (S.SELECT_NEXT_BOX, '')]
    per_box = [S.NAVIGATE_TO_PICKUP, S.DETECT_AND_IDENTIFY_BOX, S.PICKUP,
               S.NAVIGATE_TO_DESTINATION, S.PLACE_BOX, S.UPDATE_MISSION_STATE,
               S.SELECT_NEXT_BOX]
    assert [s for s, b in trace if b == 'BOX_A'] == per_box[:-1] + [S.SELECT_NEXT_BOX]
    assert trace[-1] == (S.RETURN_HOME, '')
    assert not m.carrying


def test_destination_comes_from_the_detection_outcome():
    m = started()
    for _ in range(5):
        m.advance(ok(m))                       # ... -> DETECT_AND_IDENTIFY_BOX
    assert m.state == S.DETECT_AND_IDENTIFY_BOX
    m.advance(Outcome(True, destination_id='SHELF_02', box_id='BOX_A'))
    # the destination from the QR code, not from a table
    assert m.current_destination == 'SHELF_02'


def test_transient_failures_are_retried_then_succeed():
    m = started()
    trace = run(m, {(S.NAVIGATE_TO_PICKUP, 'BOX_B'): ['NAVIGATION_FAILED'] * 2,
                    (S.DETECT_AND_IDENTIFY_BOX, 'BOX_C'): ['BOX_NOT_VISIBLE'],
                    (S.PICKUP, 'BOX_A'): ['ALIGNMENT_FAILED']})
    assert m.success
    assert trace.count((S.NAVIGATE_TO_PICKUP, 'BOX_B')) == 3
    assert trace.count((S.PICKUP, 'BOX_A')) == 2


def test_retry_counter_resets_after_success():
    m = started()
    run(m, {(S.NAVIGATE_TO_PICKUP, 'BOX_A'): ['NAVIGATION_FAILED'] * 2,
            (S.NAVIGATE_TO_PICKUP, 'BOX_B'): ['NAVIGATION_FAILED'] * 2})
    assert m.success


def test_box_not_found_is_skipped_and_mission_continues():
    m = started()
    run(m, {(S.DETECT_AND_IDENTIFY_BOX, 'BOX_B'): ['BOX_NOT_VISIBLE'] * 3})
    assert m.state == S.MISSION_FAILED and not m.success
    assert m.record.boxes_delivered == ['BOX_A', 'BOX_C']
    assert 'BOX_B' in m.record.boxes_failed
    assert 'BOX_B' in m.record.failure_reason
    assert m.record.events[-1][0] == S.RETURN_HOME.value     # the robot went home


def test_identity_mismatch_is_not_retried():
    m = started()
    trace = run(m, {(S.DETECT_AND_IDENTIFY_BOX, 'BOX_A'): ['IDENTITY_MISMATCH']})
    assert trace.count((S.DETECT_AND_IDENTIFY_BOX, 'BOX_A')) == 1
    assert m.record.boxes_failed == {'BOX_A': 'IDENTITY_MISMATCH at DETECT_AND_IDENTIFY_BOX'}
    assert m.record.boxes_delivered == ['BOX_B', 'BOX_C']


def test_failure_while_carrying_stops_the_mission():
    m = started()
    run(m, {(S.NAVIGATE_TO_DESTINATION, 'BOX_A'): ['NAVIGATION_FAILED'] * 3})
    assert m.state == S.MISSION_FAILED and m.carrying
    assert m.record.boxes_delivered == [] and m.record.boxes_picked == ['BOX_A']
    assert 'NAVIGATE_TO_DESTINATION' in m.record.failure_reason


@pytest.mark.parametrize('state,code', [(S.PICKUP, 'LIFT_FAILED'),
                                        (S.PICKUP, 'INSERTION_BLOCKED'),
                                        (S.PLACE_BOX, 'PLACEMENT_NOT_VERIFIED'),
                                        (S.PLACE_BOX, 'NOT_CARRYING')])
def test_unsafe_failures_stop_immediately_without_retry(state, code):
    m = started()
    trace = run(m, {(state, 'BOX_A'): [code]})
    assert m.state == S.MISSION_FAILED
    assert trace.count((state, 'BOX_A')) == 1
    assert 'unsafe' in m.record.failure_reason


@pytest.mark.parametrize('state', [S.INITIALIZE, S.CHECK_SYSTEMS, S.LOCALIZE])
def test_startup_failures_fail_the_mission_after_retries(state):
    code = {S.INITIALIZE: 'SYSTEMS_UNAVAILABLE', S.CHECK_SYSTEMS: 'SENSOR_UNAVAILABLE',
            S.LOCALIZE: 'LOCALIZATION_FAILED'}[state]
    m = started()
    trace = run(m, {(state, ''): [code] * 3})
    assert m.state == S.MISSION_FAILED and trace.count((state, '')) == 3
    assert m.record.boxes_picked == []


def test_return_home_failure_fails_mission_even_if_all_delivered():
    m = started()
    run(m, {(S.RETURN_HOME, ''): ['NAVIGATION_FAILED'] * 3})
    assert m.state == S.MISSION_FAILED
    assert m.record.boxes_delivered == BOXES


def test_mission_subset_and_order():
    m = Mission(['BOX_C', 'BOX_A'], RETRIES)
    m.start()
    run(m)
    assert m.success and m.record.boxes_delivered == ['BOX_C', 'BOX_A']


def test_pause_during_navigation_is_immediate_and_resume_reexecutes():
    m = started()
    while m.state != S.NAVIGATE_TO_PICKUP:
        m.advance(ok(m))
    assert m.interrupt('pause') is True
    assert m.state == S.PAUSED and m.resume_state == S.NAVIGATE_TO_PICKUP
    assert m.resume() == S.NAVIGATE_TO_PICKUP
    run(m)
    assert m.success


def test_pause_during_manipulation_is_deferred_to_the_next_boundary():
    m = started()
    while m.state != S.PICKUP:
        m.advance(ok(m))
    assert m.interrupt('pause') is False          # the skill continues to run
    assert m.state == S.PICKUP and m.pause_pending
    m.advance(ok(m))                              # the pickup is complete
    assert m.state == S.PAUSED and m.resume_state == S.NAVIGATE_TO_DESTINATION
    assert m.carrying and m.record.boxes_picked == ['BOX_A']
    m.resume()
    run(m)
    assert m.success


def test_estop_during_navigation_resumes():
    m = started()
    while m.state != S.NAVIGATE_TO_DESTINATION:
        m.advance(ok(m))
    assert m.interrupt('estop') is True and m.state == S.EMERGENCY_STOP
    assert m.interrupt('pause') is False          # already stopped
    assert m.resume() == S.NAVIGATE_TO_DESTINATION
    run(m)
    assert m.success


def test_estop_during_manipulation_cannot_resume():
    m = started()
    while m.state != S.PLACE_BOX:
        m.advance(ok(m))
    assert m.interrupt('estop') is True
    assert m.resume(interrupted_manipulation=True) == S.MISSION_FAILED
    assert 'load state unknown' in m.record.failure_reason


def test_estop_while_paused_keeps_the_resume_target():
    m = started()
    while m.state != S.RETURN_HOME:
        m.advance(ok(m))
    m.interrupt('pause')
    m.interrupt('estop')
    assert m.state == S.EMERGENCY_STOP and m.resume_state == S.RETURN_HOME
    assert m.resume() == S.RETURN_HOME


def test_cancel_is_terminal():
    m = started()
    m.advance(ok(m))
    m.cancel('operator')
    assert m.state == S.CANCELED and m.finished and not m.success
    assert m.interrupt('pause') is False
    m.cancel('again')
    assert m.record.failure_reason == 'operator'


def test_cannot_start_twice_and_idle_ignores_interrupts():
    m = Mission(BOXES, RETRIES)
    assert m.interrupt('estop') is False and m.state == S.IDLE
    m.start()
    with pytest.raises(RuntimeError):
        m.start()


def test_event_log_records_every_transition():
    m = started()
    trace = run(m, {(S.PICKUP, 'BOX_A'): ['ALIGNMENT_FAILED']})
    assert len(m.record.events) == len(trace)
    assert m.record.events[0] == ('INITIALIZE', 'ok')
    assert ('PICKUP', 'ALIGNMENT_FAILED: injected') in m.record.events


def test_without_return_home_the_mission_ends_after_the_last_delivery():
    m = Mission(['BOX_B'], RETRIES, return_home=False)
    m.start()
    trace = run(m)
    assert m.success and (S.RETURN_HOME, '') not in trace
    assert m.record.boxes_delivered == ['BOX_B']


def test_rear_blocked_while_carrying_stops_the_mission():
    m = started()
    run(m, {(S.PICKUP, 'BOX_A'): ['REAR_BLOCKED']})
    # The skill raised an exception while the robot moved rearward. The box is not counted
    # as picked, and the mission has no load on the forks. The box is skipped, as with all
    # pickup failures that do not permit a new try.
    assert m.record.boxes_failed['BOX_A'] == 'REAR_BLOCKED at PICKUP'
    m2 = started()
    run(m2, {(S.PLACE_BOX, 'BOX_A'): ['REAR_BLOCKED']})
    assert m2.state == S.MISSION_FAILED and m2.carrying
