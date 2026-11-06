"""Cost-based allocation: distances, sequence, penalties, stability."""

from forklift_fleet.allocator import (AllocatorConfig, plan, RobotView, straight_route,
                                      task_cost, TaskView)
from forklift_task_manager.mission_config import default_mission_path, load_mission
import pytest

M = load_mission(default_mission_path())
CFG = AllocatorConfig()


def tasks():
    out = []
    for b in M.boxes:
        p = M.pickup_staging(b.box_id)
        d = M.destination_staging(b.destination)
        out.append(TaskView(f'task_{b.box_id}', (p.x, p.y), (d.x, d.y)))
    return out


def robots():
    return [RobotView('forklift_1', -6.0, 0.0), RobotView('forklift_2', 6.0, 0.0)]


def test_cost_is_route_to_pickup_plus_delivery():
    t = tasks()[0]
    r = robots()[0]
    route = straight_route(CFG)
    expected = route('', (-6.0, 0.0), t.pickup) + route('', t.pickup, t.drop)
    assert task_cost(r, t, (-6.0, 0.0), [], route, CFG) == pytest.approx(expected)


def test_each_robot_gets_the_box_nearest_to_it_and_one_gets_two():
    assignment, makespan, total = plan(robots(), tasks())
    a = {r: [t.task_id for t in ts] for r, ts in assignment.items()}
    assert a['forklift_1'][0] == 'task_BOX_A'          # PICK_01 is on the side of forklift_1
    assert a['forklift_2'][0] == 'task_BOX_C'          # PICK_03 is on the side of forklift_2
    assert sorted(a['forklift_1'] + a['forklift_2']) == ['task_BOX_A', 'task_BOX_B',
                                                         'task_BOX_C']
    assert makespan <= total


def test_a_single_robot_takes_all_tasks_in_a_sensible_order():
    assignment, _, _ = plan([RobotView('forklift_1', -6.0, 0.0)], tasks())
    order = [t.task_id for t in assignment['forklift_1']]
    assert len(order) == 3 and order[0] == 'task_BOX_A'


def test_busy_robot_is_planned_from_where_its_current_task_ends():
    far = RobotView('forklift_2', 6.0, 0.0, ready_x=6.0, ready_y=0.0, remaining=200.0)
    assignment, _, _ = plan([robots()[0], far], tasks())
    assert len(assignment['forklift_1']) == 3, 'a robot busy for 200 m gets nothing'


def test_switching_penalty_keeps_reservations_stable():
    ts = tasks()
    ts[1].reserved_for = 'forklift_1'                  # BOX_B is planned for forklift_1
    r1 = RobotView('forklift_1', -0.2, -1.0)
    r2 = RobotView('forklift_2', 0.2, -1.0)            # a very small distance nearer to BOX_B
    assignment, _, _ = plan([r1, r2], [ts[1]])
    assert [t.task_id for t in assignment['forklift_1']] == ['task_BOX_B']
    far = RobotView('forklift_1', -7.0, 0.0)           # much farther: the plan changes
    assignment, _, _ = plan([far, r2], [ts[1]])
    assert [t.task_id for t in assignment['forklift_2']] == ['task_BOX_B']


def test_recovery_and_congestion_penalties_steer_work_away():
    t = tasks()[1]
    good = RobotView('forklift_1', 0.0, 0.0)
    bad = RobotView('forklift_2', 0.0, 0.0, failures=2)
    a, _, _ = plan([good, bad], [t])
    assert a['forklift_1'] and not a['forklift_2']
    busy_there = RobotView('forklift_2', 3.0, 3.0, targets=[t.pickup])
    c_free = task_cost(good, t, (0, 0), [], straight_route(CFG), CFG)
    c_cong = task_cost(good, t, (0, 0), [busy_there], straight_route(CFG), CFG)
    assert c_cong == pytest.approx(c_free + CFG.congestion_penalty)


def test_route_function_is_used():
    calls = []

    def route(robot_id, a, b):
        calls.append(robot_id)
        return 1.0
    plan(robots(), tasks()[:1], route=route)
    assert set(calls) == {'forklift_1', 'forklift_2'}


def test_empty_inputs():
    assert plan([], tasks())[0] == {}
    assert plan(robots(), [])[0] == {'forklift_1': [], 'forklift_2': []}
