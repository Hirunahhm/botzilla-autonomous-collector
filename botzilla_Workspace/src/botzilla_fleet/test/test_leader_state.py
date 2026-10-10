import math

from botzilla_fleet.leader_state import (
    LeaderStateTracker, MOVING, PARKED, STILL_WITH_GOAL_S, STUCK, STUCK_HOLD_S,
    STUCK_RELEASE_S,
)


def _feed(tr, t0, t1, pose_at, dt=0.1, recoveries_at=None):
    t = t0
    while t <= t1 + 1e-9:
        tr.pose(t, *pose_at(t))
        if recoveries_at is not None:
            tr.feedback(t, recoveries_at(t))
        t += dt
    return t1


def test_still_is_parked():
    tr = LeaderStateTracker()
    t = _feed(tr, 0.0, 5.0, lambda t: (1.0, 1.0, 0.0))
    assert tr.state(t) == PARKED


def test_driving_is_moving():
    tr = LeaderStateTracker()
    t = _feed(tr, 0.0, 3.0, lambda t: (0.2 * t, 0.0, 0.0))
    assert tr.state(t) == MOVING


def test_turning_in_place_is_moving():
    # run 16: the leader spun to inspect a cube and looked parked by position alone
    tr = LeaderStateTracker()
    t = _feed(tr, 0.0, 3.0, lambda t: (1.0, 1.0, 0.5 * t))
    assert tr.state(t) == MOVING


def test_recovery_marks_stuck_even_while_backing_up():
    tr = LeaderStateTracker()
    _feed(tr, 0.0, 2.0, lambda t: (1.0, 1.0, 0.0), recoveries_at=lambda t: 0)
    # BackUp recovery: moves 0.1 m, which alone would read as MOVING
    t = _feed(tr, 2.1, 3.0, lambda t: (1.0 - 0.1 * (t - 2.0), 1.0, 0.0),
              recoveries_at=lambda t: 1)
    assert tr.state(t) == STUCK


def test_recovery_then_escape_is_not_stuck():
    tr = LeaderStateTracker()
    _feed(tr, 0.0, 1.0, lambda t: (0.0, 0.0, 0.0), recoveries_at=lambda t: 0)
    _feed(tr, 1.1, 1.2, lambda t: (0.0, 0.0, 0.0), recoveries_at=lambda t: 1)
    # a costmap clear, then real progress: 0.2 m/s for 2 s = 0.4 m
    t = _feed(tr, 1.3, 3.3, lambda t: (0.2 * (t - 1.3), 0.0, 0.0),
              recoveries_at=lambda t: 1)
    assert tr.state(t) == MOVING


def _states(tr, t0, t1, pose_at, dt=0.1, recoveries_at=None):
    out, t = [], t0
    while t <= t1 + 1e-9:
        tr.pose(t, *pose_at(t))
        if recoveries_at is not None:
            tr.feedback(t, recoveries_at(t))
        out.append(tr.state(t))
        t += dt
    return out


def test_stuck_stays_through_the_gap_between_goals():
    # run 17: the explorer cancels a stuck goal and sends the next a few seconds later;
    # in between there is no goal, no recovery and no motion — still stuck, not PARKED
    tr = LeaderStateTracker()
    _states(tr, 0.0, 1.0, lambda t: (0.0, 0.0, 0.0), recoveries_at=lambda t: 0)
    _states(tr, 1.1, 1.1, lambda t: (0.0, 0.0, 0.0), recoveries_at=lambda t: 1)
    gap = _states(tr, 1.2, 1.2 + STUCK_HOLD_S + 3.0, lambda t: (0.0, 0.0, 0.0))
    assert set(gap) == {STUCK}


def test_stuck_released_after_quiet_spell():
    tr = LeaderStateTracker()
    _states(tr, 0.0, 1.0, lambda t: (0.0, 0.0, 0.0), recoveries_at=lambda t: 0)
    _states(tr, 1.1, 1.1, lambda t: (0.0, 0.0, 0.0), recoveries_at=lambda t: 1)
    end = 1.2 + STUCK_HOLD_S + STUCK_RELEASE_S + 1.0
    states = _states(tr, 1.2, end, lambda t: (0.0, 0.0, 0.0))
    assert states[-1] == PARKED


def test_stuck_released_by_driving_away():
    tr = LeaderStateTracker()
    _states(tr, 0.0, 1.0, lambda t: (0.0, 0.0, 0.0), recoveries_at=lambda t: 0)
    _states(tr, 1.1, 1.1, lambda t: (0.0, 0.0, 0.0), recoveries_at=lambda t: 1)
    # no goal now, but it drives 0.4 m off (e.g. the next goal led it out)
    states = _states(tr, 1.2 + STUCK_HOLD_S, 3.2 + STUCK_HOLD_S,
                     lambda t: (0.2 * (t - 1.2 - STUCK_HOLD_S), 0.0, 0.0))
    assert states[-1] == MOVING


def test_holding_a_goal_without_moving_is_stuck():
    tr = LeaderStateTracker()
    t = _feed(tr, 0.0, STILL_WITH_GOAL_S + 0.5, lambda t: (0.0, 0.0, 0.0),
              recoveries_at=lambda t: 0)
    assert tr.state(t) == STUCK


def test_turning_to_line_up_with_a_goal_is_not_stuck():
    tr = LeaderStateTracker()
    t = _feed(tr, 0.0, 3.0, lambda t: (0.0, 0.0, math.radians(30) * t),
              recoveries_at=lambda t: 0)
    assert tr.state(t) == MOVING
