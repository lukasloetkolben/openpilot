from types import SimpleNamespace

from openpilot.cereal import log
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.controls.lib.desire_helper import LANE_CHANGE_SPEED_MIN, DesireHelper

Desire = log.Desire


def make_cs(v_ego, left=False, right=False, left_blindspot=False, right_blindspot=False):
  return SimpleNamespace(vEgo=v_ego, leftBlinker=left, rightBlinker=right, leftBlindspot=left_blindspot, rightBlindspot=right_blindspot,
                         steeringPressed=False, steeringTorque=0.0)


def get_desire(cs, steps=5):
  dh = DesireHelper()
  for _ in range(steps):
    dh.update(cs, True, 0.0)
  return dh.desire


class TestTurnDesire(OpenpilotTestCase):
  def test_blinker_below_lane_change_speed_is_a_turn(self):
    v = LANE_CHANGE_SPEED_MIN - 2.0
    assert get_desire(make_cs(v, left=True)) == Desire.turnLeft
    assert get_desire(make_cs(v, right=True)) == Desire.turnRight

  def test_blinker_above_lane_change_speed_is_no_turn(self):
    # it arms a lane change there, which waits for a nudge on the wheel
    assert get_desire(make_cs(LANE_CHANGE_SPEED_MIN + 2.0, right=True)) == Desire.none

  def test_no_turn_into_blindspot(self):
    v = LANE_CHANGE_SPEED_MIN - 2.0
    assert get_desire(make_cs(v, right=True, right_blindspot=True)) == Desire.none
    assert get_desire(make_cs(v, right=True, left_blindspot=True)) == Desire.turnRight

  def test_no_turn_with_hazards_or_no_blinker(self):
    v = LANE_CHANGE_SPEED_MIN - 2.0
    assert get_desire(make_cs(v, left=True, right=True)) == Desire.none
    assert get_desire(make_cs(v)) == Desire.none
