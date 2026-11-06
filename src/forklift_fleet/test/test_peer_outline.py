import math

from forklift_fleet.fleet_agent import outline_points


def test_outline_covers_body_and_forks_with_margin():
    p = outline_points(0.0, 0.0, 0.0, margin=0.1)
    assert math.isclose(p[:, 0].min(), -1.33) and math.isclose(p[:, 0].max(), 1.48)
    assert math.isclose(p[:, 1].min(), -0.71) and math.isclose(p[:, 1].max(), 0.71)
    tines = p[p[:, 0] > 0.6]
    assert abs(tines[:, 1]).max() <= 0.38 + 1e-9        # the fork section is narrower


def test_outline_follows_pose_and_stretches_ahead():
    p = outline_points(1.0, 2.0, math.pi / 2, margin=0.0, ahead=0.5)
    assert math.isclose(p[:, 1].max(), 2.0 + 1.38 + 0.5, abs_tol=1e-6)   # the forks point to +y
    assert math.isclose(p[:, 1].min(), 2.0 - 1.23, abs_tol=1e-6)
