from types import SimpleNamespace as NS

import numpy as np

from openpilot.selfdrive.modeld import nav_intent
from openpilot.selfdrive.modeld.nav_intent import NONE, TURN_LEFT, TURN_RIGHT, NavIntent, Pulser, from_blinker, from_maneuver, from_route


def maneuver(distance, mtype="turn", modifier="left"):
  return NS(distance=distance, type=mtype, modifier=modifier)


class TestFromManeuver:
  def test_turns_within_distance(self):
    assert from_maneuver("turn", "left", 30) == TURN_LEFT
    assert from_maneuver("turn", "sharp right", 0) == TURN_RIGHT
    assert from_maneuver("end of road", "right", 59) == TURN_RIGHT

  def test_too_far_or_passed(self):
    assert from_maneuver("turn", "left", 61) == NONE
    assert from_maneuver("turn", "left", -1) == NONE

  def test_slight_bend_along_road_is_no_turn(self):
    for mtype in ("continue", "new name", "fork"):
      assert from_maneuver(mtype, "slight left", 20) == NONE
      assert from_maneuver(mtype, "slight right", 20) == NONE
    assert from_maneuver("turn", "slight left", 20) == TURN_LEFT

  def test_other_types(self):
    for mtype in ("depart", "arrive", "merge", "on ramp", "roundabout"):
      assert from_maneuver(mtype, "left", 10) == NONE
    assert from_maneuver("turn", "straight", 10) == NONE


class TestFromRoute:
  def test_next_maneuver_moved_by_travel(self):
    route = [maneuver(80, modifier="right")]
    assert from_route(route, 0) == NONE
    assert from_route(route, 25) == TURN_RIGHT

  def test_skips_depart_and_passed(self):
    route = [maneuver(0, "depart", "right"), maneuver(-5), maneuver(40, modifier="right")]
    assert from_route(route, 0) == TURN_RIGHT

  def test_only_the_next_maneuver_counts(self):
    # a far straight step hides a turn behind it, as in training
    assert from_route([maneuver(100, modifier="straight"), maneuver(110)], 50) == NONE

  def test_empty(self):
    assert from_route([], 0) == NONE


class TestFromBlinker:
  def test_junction_speed(self):
    assert from_blinker(True, False, 5.0, False) == TURN_LEFT
    assert from_blinker(False, True, 5.0, False) == TURN_RIGHT

  def test_lane_change_or_fast_or_hazards(self):
    assert from_blinker(True, False, 5.0, True) == NONE
    assert from_blinker(True, False, 9.0, False) == NONE
    assert from_blinker(True, True, 5.0, False) == NONE


class TestPulser:
  def test_rising_edge_and_repulse(self):
    p = Pulser(hz=20)
    out = [p(TURN_LEFT)[TURN_LEFT] for _ in range(201)]
    assert np.flatnonzero(out).tolist() == [0, 100, 200]

  def test_switch_direction_pulses_at_once(self):
    p = Pulser(hz=20)
    p(TURN_LEFT)
    p(TURN_LEFT)
    assert p(TURN_RIGHT)[TURN_RIGHT] == 1

  def test_none_sends_nothing(self):
    p = Pulser(hz=20)
    assert not p(NONE).any()
    p(TURN_LEFT)
    assert not p(NONE).any()
    assert p(TURN_LEFT)[TURN_LEFT] == 1


class TestNavIntent:
  car = NS(leftBlinker=False, rightBlinker=True)

  def test_route_wins_over_blinker(self):
    ni = NavIntent()
    nav = NS(allManeuvers=[maneuver(30, modifier="left")])
    assert ni.update(nav, 0.5, self.car, 5.0, False)[TURN_LEFT] == 1

  def test_stale_route_falls_back_to_blinker(self):
    ni = NavIntent()
    nav = NS(allManeuvers=[maneuver(30, modifier="left")])
    assert ni.update(nav, nav_intent.NAV_TIMEOUT + 1, self.car, 5.0, False)[TURN_RIGHT] == 1
    assert NavIntent().update(nav, None, self.car, 5.0, False)[TURN_RIGHT] == 1

  def test_no_route_no_blinker(self):
    ni = NavIntent()
    car = NS(leftBlinker=False, rightBlinker=False)
    assert not ni.update(NS(allManeuvers=[]), 0.1, car, 5.0, False).any()


class TestTunedModelState:
  def test_pending_and_installed(self, tmp_path, monkeypatch):
    from openpilot.selfdrive.modeld import tuned_model as tm
    pkl, info = tmp_path / "big.pkl", tmp_path / "big.pkl.info"
    monkeypatch.setattr(tm, "TUNED_ONNX", tmp_path / "tuned.onnx")
    monkeypatch.setattr(tm, "TUNED_SHA", tmp_path / "tuned.onnx.sha256")
    monkeypatch.setattr(tm, "BIG_PKL", pkl)
    monkeypatch.setattr(tm, "INFO", info)
    pkl.write_bytes(b"x" * 10)

    assert not tm.pending() and not tm.installed()  # nothing dropped in
    tm.TUNED_ONNX.write_bytes(b"onnx")
    tm.TUNED_SHA.write_text("abc  tuned.onnx\n")
    assert tm.pending() and not tm.installed()
    info.write_text("abc 10\n")
    assert tm.installed() and not tm.pending()
    pkl.write_bytes(b"y" * 11)  # a branch switch put the stock pkl back
    assert tm.pending()
    info.write_text("old 11\n")  # a new tuned onnx was dropped in
    assert tm.pending()
