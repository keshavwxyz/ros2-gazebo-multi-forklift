"""Task registry: lifecycle, ownership, duplicate claims, release and hold policy."""

from forklift_fleet.registry import (ASSIGNED, AVAILABLE, DELIVERED, FAILED, HELD,
                                     PICKED_UP, PICKUP_IN_PROGRESS, RESERVED, TaskRegistry,
                                     TransitionError)
import pytest

BOXES = [('BOX_A', 'PICK_01', 'SHELF_01'), ('BOX_B', 'PICK_02', 'SHELF_03'),
         ('BOX_C', 'PICK_03', 'SHELF_02')]


def reg():
    return TaskRegistry(BOXES, max_attempts=2)


def test_initial_state_and_order():
    r = reg()
    assert [t.box_id for t in r.unassigned()] == ['BOX_A', 'BOX_B', 'BOX_C']
    assert all(t.state == AVAILABLE and not t.owner for t in r.tasks.values())


def test_duplicate_claims_get_exactly_one_grant():
    r = reg()
    results = [r.claim('BOX_A', 'forklift_1'), r.claim('BOX_A', 'forklift_2')]
    assert [g for g, _, _ in results] == [True, False]
    assert 'already owned by forklift_1' in results[1][2]
    assert r.by_box('BOX_A').owner == 'forklift_1'


def test_robot_cannot_own_two_tasks():
    r = reg()
    assert r.claim('BOX_A', 'forklift_1')[0]
    granted, _, reason = r.claim('BOX_B', 'forklift_1')
    assert not granted and 'already owns task_BOX_A' in reason


def test_reservation_blocks_other_robots_but_not_the_planned_one():
    r = reg()
    r.reserve('task_BOX_B', 'forklift_2')
    assert r.by_box('BOX_B').state == RESERVED
    assert not r.claim('BOX_B', 'forklift_1')[0]
    granted, t, _ = r.claim('BOX_B', 'forklift_2')
    assert granted and t.state == ASSIGNED and t.reserved_for == ''


def test_assignment_ids_are_unique():
    r = reg()
    _, t, _ = r.claim('BOX_A', 'forklift_1')
    first = t.assignment_id
    r.release('task_BOX_A', 'forklift_1', 'nav failed')
    _, t, _ = r.claim('BOX_A', 'forklift_2')
    assert t.assignment_id != first


def test_only_the_owner_reports_progress_and_the_chain_is_validated():
    r = reg()
    r.claim('BOX_A', 'forklift_1')
    with pytest.raises(TransitionError):
        r.advance('task_BOX_A', PICKUP_IN_PROGRESS, 'forklift_2')
    r.advance('task_BOX_A', PICKED_UP, 'forklift_1')        # late report: goes through the chain
    states = [new for _, _, _, new, _, _ in r.log]
    assert states[-2:] == [PICKUP_IN_PROGRESS, PICKED_UP]
    r.advance('task_BOX_A', DELIVERED, 'forklift_1')
    t = r.by_box('BOX_A')
    assert t.state == DELIVERED and 'placement verified' in t.last_confirmed
    with pytest.raises(TransitionError):
        # after DELIVERED, no transition is possible
        r.advance('task_BOX_A', PICKED_UP, 'forklift_1')


def test_release_before_lifting_makes_it_available_then_failed():
    r = reg()
    r.claim('BOX_C', 'forklift_2')
    r.release('task_BOX_C', 'forklift_2', 'box not visible')
    t = r.by_box('BOX_C')
    assert t.state == AVAILABLE and t.attempts == 1 and t.owner == ''
    r.claim('BOX_C', 'forklift_1')
    r.release('task_BOX_C', 'forklift_1', 'box not visible')
    assert r.by_box('BOX_C').state == FAILED


def test_load_on_forks_cannot_be_released_only_held():
    r = reg()
    r.claim('BOX_A', 'forklift_1')
    r.advance('task_BOX_A', PICKED_UP, 'forklift_1')
    with pytest.raises(TransitionError):
        r.release('task_BOX_A', 'forklift_1', 'robot lost')
    r.hold('task_BOX_A', 'forklift_1 unavailable while carrying')
    t = r.by_box('BOX_A')
    assert t.state == HELD and not r.claim('BOX_A', 'forklift_2')[0]
    r.operator_release('task_BOX_A', 'operator put the box back', box_at_pickup=True)
    assert r.by_box('BOX_A').state == AVAILABLE


def test_finished_and_all_delivered():
    r = reg()
    for box, robot in (('BOX_A', 'forklift_1'), ('BOX_B', 'forklift_2')):
        r.claim(box, robot)
        r.advance(f'task_{box}', DELIVERED, robot)
    assert not r.finished()
    r.claim('BOX_C', 'forklift_1')
    r.advance('task_BOX_C', DELIVERED, 'forklift_1')
    assert r.finished() and r.all_delivered()


def test_unknown_box_claim_is_refused():
    granted, t, reason = reg().claim('BOX_Z', 'forklift_1')
    assert not granted and t is None and 'unknown' in reason
