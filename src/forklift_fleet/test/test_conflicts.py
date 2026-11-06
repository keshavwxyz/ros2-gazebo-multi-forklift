"""Reservations, path conflicts, priority rules, deadlock detection."""

from forklift_fleet.conflicts import (choose_priority, Contender, DeadlockMonitor, in_zone,
                                      path_conflict, ReservationTable, trim_path)


def test_reservation_is_exclusive_renewable_and_expires():
    t = ReservationTable(ttl=10.0)
    assert t.grant('SHELF_01', 'forklift_1', 0.0) == (True, 'forklift_1')
    assert t.grant('SHELF_01', 'forklift_2', 1.0) == (False, 'forklift_1')
    assert t.grant('SHELF_01', 'forklift_1', 8.0)[0]            # renew
    assert t.items['SHELF_01'].stamp == 0.0 and t.items['SHELF_01'].expires == 18.0
    assert t.holder('SHELF_01', 17.0) == 'forklift_1'
    assert t.holder('SHELF_01', 18.5) == ''                       # expired: it does not block now
    assert t.grant('SHELF_01', 'forklift_2', 18.5)[0]
    assert [r.resource for r in t.expire(100.0)] == ['SHELF_01']


def test_release_only_by_owner():
    t = ReservationTable()
    t.grant('PICK_02', 'forklift_1', 0.0)
    assert not t.release('PICK_02', 'forklift_2')
    assert t.release('PICK_02', 'forklift_1') and t.holder('PICK_02', 0.0) == ''
    t.grant('A', 'forklift_1', 0.0)
    t.grant('B', 'forklift_1', 0.0)
    t.release_all('forklift_1')
    assert t.held_by('forklift_1', 0.0) == []


def test_trim_path_starts_near_the_robot_and_stops_at_lookahead():
    path = [(x * 0.5, 0.0) for x in range(20)]
    ahead = trim_path(path, (2.1, 0.0), 2.0)
    assert ahead[0] == (2.0, 0.0) and ahead[-1] == (4.0, 0.0)


def test_head_on_paths_conflict_and_parallel_paths_do_not():
    a_path = [(x * 0.25, 0.0) for x in range(0, 40)]                 # to the east along y=0
    b_path = [(10 - x * 0.25, 0.0) for x in range(0, 40)]           # to the west along y=0
    conflict, d, _ = path_conflict((0, 0), a_path, (6.0, 0), b_path, 1.8, 6.0)
    assert conflict and d < 1.8
    far = [(10 - x * 0.25, 4.0) for x in range(0, 40)]
    assert not path_conflict((0, 0), a_path, (6.0, 4.0), far, 1.8, 6.0)[0]


def test_robot_standing_on_the_path_conflicts():
    a_path = [(x * 0.25, 0.0) for x in range(0, 40)]
    assert path_conflict((0, 0), a_path, (3.0, 0.5), [], 1.8, 6.0)[0]
    # farther than lookahead
    assert not path_conflict((0, 0), a_path, (8.0, 0.5), [], 1.8, 6.0)[0]


def test_priority_rules_in_order():
    loaded = Contender('forklift_2', carrying=True)
    assert choose_priority(Contender('forklift_1', moving=True), loaded)[:2] == (
        'forklift_2', 'forklift_1')
    inside = Contender('forklift_2', inside_own_zone=True)
    assert choose_priority(Contender('forklift_1', moving=True), inside)[2] == \
        'already inside its reserved zone'
    assert choose_priority(Contender('forklift_1'), Contender('forklift_2', moving=True))[0] \
        == 'forklift_2'
    p, y, rule = choose_priority(Contender('forklift_2'), Contender('forklift_1'))
    assert (p, y, rule) == ('forklift_1', 'forklift_2', 'robot-id tie-breaker')


def test_robot_in_the_others_destination_zone_passes_first():
    # Gazebo, head-on: forklift_2 yielded in the home zone of forklift_1. forklift_1 (with or
    # without a load) could not arrive while forklift_2 waited there.
    blocker = Contender('forklift_2', in_others_target=True)
    loaded = Contender('forklift_1', carrying=True, moving=True)
    assert choose_priority(loaded, blocker) == (
        'forklift_2', 'forklift_1', "must clear the other robot's destination zone")


def test_priority_is_symmetric_no_ping_pong():
    a, b = Contender('forklift_1', moving=True), Contender('forklift_2', moving=True)
    assert choose_priority(a, b)[:2] == choose_priority(b, a)[:2]


def test_deadlock_needs_long_wait_and_no_progress():
    m = DeadlockMonitor(timeout=40.0, window=15.0, min_progress=0.3)
    m.start(0.0)
    for t in range(0, 60):
        m.update(float(t), (0.0, 0.0))
        if t < 40:
            assert not m.deadlocked(float(t))
    assert m.deadlocked(59.0)
    m2 = DeadlockMonitor(timeout=40.0, window=15.0, min_progress=0.3)
    m2.start(0.0)
    for t in range(0, 60):
        m2.update(float(t), (t * 0.1, 0.0))              # the priority robot moves
    assert not m2.deadlocked(59.0)


def test_in_zone():
    assert in_zone((1.0, 1.0), (0.0, 0.0, 1.5)) and not in_zone((2.0, 2.0), (0.0, 0.0, 1.5))
