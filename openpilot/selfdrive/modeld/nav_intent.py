"""Turn intent for the fine-tuned big model, sent through the desire input's turnLeft/turnRight slots.

Same mapping as training (openpilot-finetune opft/intent.py): navd's next maneuver within INTENT_DIST, or the blinker
at junction speeds when no route is active. The model only sees desire as a pulse and remembers it for 6.6 s
(state_desire_q), so while an intent is active the pulse is re-sent every REPULSE_S.
"""
import numpy as np

from openpilot.cereal import log
from openpilot.selfdrive.modeld.constants import ModelConstants

NONE, TURN_LEFT, TURN_RIGHT = log.Desire.none, log.Desire.turnLeft, log.Desire.turnRight
INTENT_DIST = 60.0      # m before the maneuver
REPULSE_S = 5.0         # < 6.6 s desire history
BLINKER_MAX_SPEED = 8.5  # m/s, above this a blinker means a lane change
NAV_TIMEOUT = 2.0       # s without navInstruction before the route is treated as gone

# mapbox maneuver modifiers that mean a real turn at a junction
_LEFT = {"left", "sharp left", "slight left"}
_RIGHT = {"right", "sharp right", "slight right"}
_TURN_TYPES = {"turn", "end of road", "fork", "continue", "new name"}
_SLIGHT_FOLLOW = {"continue", "new name", "fork"}  # a slight bend along these is just the road


def from_maneuver(mtype: str, modifier: str, distance: float) -> int:
  if distance < 0 or distance > INTENT_DIST or mtype not in _TURN_TYPES:
    return NONE
  if modifier in _LEFT and not (mtype in _SLIGHT_FOLLOW and modifier == "slight left"):
    return TURN_LEFT
  if modifier in _RIGHT and not (mtype in _SLIGHT_FOLLOW and modifier == "slight right"):
    return TURN_RIGHT
  return NONE


def from_route(maneuvers, traveled: float) -> int:
  """the next maneuver ahead, moved on by the distance driven since navd sent it"""
  for m in maneuvers:
    distance = m.distance - traveled
    if distance >= 0 and m.type != "depart":
      return from_maneuver(m.type, m.modifier, distance)
  return NONE


def from_blinker(left: bool, right: bool, v_ego: float, lane_change_active: bool) -> int:
  if lane_change_active or v_ego > BLINKER_MAX_SPEED:
    return NONE
  return TURN_LEFT if left and not right else TURN_RIGHT if right and not left else NONE


class Pulser:
  """desire per frame -> pulse vector: rising edge, then again every REPULSE_S while active"""
  def __init__(self, hz: float = ModelConstants.MODEL_RUN_FREQ):
    self.period = int(REPULSE_S * hz)
    self.prev, self.age = NONE, 0

  def __call__(self, desire: int) -> np.ndarray:
    v = np.zeros(ModelConstants.DESIRE_LEN, dtype=np.float32)
    if desire != NONE and (desire != self.prev or self.age >= self.period):
      v[desire], self.age = 1.0, 0
    self.age += 1
    self.prev = desire
    return v


class NavIntent:
  def __init__(self):
    self.pulser = Pulser()
    self.desire = NONE

  def update(self, nav, nav_age: float | None, car_state, v_ego: float, lane_change_active: bool) -> np.ndarray:
    """nav: navInstruction, nav_age: seconds since it was received (None if never)"""
    if nav_age is not None and nav_age < NAV_TIMEOUT and len(nav.allManeuvers):
      self.desire = from_route(nav.allManeuvers, v_ego * nav_age)
    else:
      self.desire = from_blinker(car_state.leftBlinker, car_state.rightBlinker, v_ego, lane_change_active)
    return self.pulser(self.desire)
