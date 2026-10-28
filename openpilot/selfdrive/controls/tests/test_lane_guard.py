import numpy as np

from openpilot.cereal import log
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.controls.lib.lane_guard import CAR_HALF_WIDTH, LANE_CHANGE_COOLDOWN, MAX_LAT_ACCEL, MAX_LAT_JERK, MAX_LEFT_LAT_ACCEL, \
                                                          LaneGuard
from openpilot.selfdrive.controls.lib.ldw import CAMERA_OFFSET
from openpilot.selfdrive.modeld.constants import ModelConstants

V_EGO = 20.0
LaneChangeState = log.LaneChangeState


def make_model(car_y=0.0, lane_width=3.0, left_prob=0.9, right_prob=0.9, lane_change=LaneChangeState.off, curve=0.0):
  # lane centered on the car with curvature curve (1/m, positive right), planned path held at car_y (car center, m to the right)
  md = log.ModelDataV2.new_message()
  path_x = V_EGO * np.array(ModelConstants.T_IDXS)
  md.position.x = path_x.tolist()
  md.position.y = (car_y + CAMERA_OFFSET + curve / 2 * path_x ** 2).tolist()
  lane_lines = md.init('laneLines', 4)
  line_x = np.array(ModelConstants.X_IDXS)
  for i, y in enumerate((-1.5 * lane_width, -lane_width / 2, lane_width / 2, 1.5 * lane_width)):
    lane_lines[i].x = line_x.tolist()
    lane_lines[i].y = (y + curve / 2 * line_x ** 2).tolist()
  md.laneLineProbs = [0.5, left_prob, right_prob, 0.5]
  md.laneLineStds = [0.1] * 4
  md.meta.laneChangeState = lane_change
  return md


def settle(guard, md, active=True, steps=500, blinker=False):
  for _ in range(steps):
    curvature = guard.update(V_EGO, md, active, blinker)
  return curvature


class TestLaneGuard(OpenpilotTestCase):
  def test_centered_does_nothing(self):
    assert settle(LaneGuard(), make_model()) == 0.0

  def test_pushes_away_from_close_line(self):
    # left body edge 5 cm over the left line, positive curvature steers right
    car_y = -(1.3 - CAR_HALF_WIDTH + 0.05)
    assert settle(LaneGuard(), make_model(car_y, lane_width=2.6)) > 0.0
    assert settle(LaneGuard(), make_model(-car_y, lane_width=2.6)) < 0.0

  def test_crossing_pushes_harder_than_close(self):
    close = settle(LaneGuard(), make_model(1.3 - CAR_HALF_WIDTH - 0.05, lane_width=2.6))
    over = settle(LaneGuard(), make_model(1.3 - CAR_HALF_WIDTH + 0.10, lane_width=2.6))
    assert 0.0 > close > over

  def test_left_line_unguarded_on_unmarked_road(self):
    # only the left line is seen, likely the imagined middle of the road
    car_y = -(1.3 - CAR_HALF_WIDTH + 0.10)
    assert settle(LaneGuard(), make_model(car_y, lane_width=2.6, right_prob=0.3)) == 0.0

  def test_left_line_gentle_outside_left_curves(self):
    car_y = -(1.3 - CAR_HALF_WIDTH + 0.5)
    straight = settle(LaneGuard(), make_model(car_y, lane_width=2.6))
    left_curve = settle(LaneGuard(), make_model(car_y, lane_width=2.6, curve=-1 / 300))
    assert 0.0 < straight <= MAX_LEFT_LAT_ACCEL / V_EGO ** 2 + 1e-12
    assert left_curve > 2 * straight

  def test_narrow_lane_centered(self):
    # both margins overlap, the pushes from both lines cancel when centered
    assert abs(settle(LaneGuard(), make_model(lane_width=2.1))) < 1e-9
    assert settle(LaneGuard(), make_model(-0.05, lane_width=2.1)) > 0.0

  def test_ignores_unsure_line(self):
    car_y = -(1.3 - CAR_HALF_WIDTH + 0.05)
    assert settle(LaneGuard(), make_model(car_y, lane_width=2.6, left_prob=0.3)) == 0.0

  def test_line_hysteresis(self):
    car_y = -(1.3 - CAR_HALF_WIDTH + 0.05)
    assert settle(LaneGuard(), make_model(car_y, lane_width=2.6, left_prob=0.65)) == 0.0
    guard = LaneGuard()
    settle(guard, make_model(car_y, lane_width=2.6, left_prob=0.8))
    assert settle(guard, make_model(car_y, lane_width=2.6, left_prob=0.65)) > 0.0
    assert settle(guard, make_model(car_y, lane_width=2.6, left_prob=0.55)) == 0.0

  def test_ignores_too_narrow_lane(self):
    assert settle(LaneGuard(), make_model(-0.3, lane_width=1.9)) == 0.0

  def test_wide_lane_guards_right_line_only(self):
    # no center line, the left line is the far road edge
    assert settle(LaneGuard(), make_model(-(2.5 - CAR_HALF_WIDTH + 0.05), lane_width=5.0)) == 0.0
    assert settle(LaneGuard(), make_model(2.5 - CAR_HALF_WIDTH + 0.05, lane_width=5.0)) < 0.0

  def test_off_during_and_after_lane_change(self):
    guard = LaneGuard()
    car_y = -(1.3 - CAR_HALF_WIDTH + 0.05)
    assert settle(guard, make_model(car_y, lane_width=2.6, lane_change=LaneChangeState.laneChangeStarting)) == 0.0
    steps = int(LANE_CHANGE_COOLDOWN / guard.dt) - 1
    assert settle(guard, make_model(car_y, lane_width=2.6), steps=steps) == 0.0
    assert settle(guard, make_model(car_y, lane_width=2.6)) > 0.0

  def test_off_with_blinker_and_after(self):
    # the blinker alone pauses it, e.g. below the lane change speed where laneChangeState stays off
    guard = LaneGuard()
    car_y = -(1.3 - CAR_HALF_WIDTH + 0.05)
    assert settle(guard, make_model(car_y, lane_width=2.6), blinker=True) == 0.0
    steps = int(LANE_CHANGE_COOLDOWN / guard.dt) - 1
    assert settle(guard, make_model(car_y, lane_width=2.6), steps=steps) == 0.0
    assert settle(guard, make_model(car_y, lane_width=2.6)) > 0.0

  def test_limits(self):
    guard = LaneGuard()
    md = make_model(1.0, lane_width=2.6)
    first = guard.update(V_EGO, md, True)
    assert 0.0 > first >= -MAX_LAT_JERK / V_EGO ** 2 * guard.dt - 1e-12
    assert settle(guard, md) >= -MAX_LAT_ACCEL / V_EGO ** 2 - 1e-12

  def test_ramps_out_when_inactive(self):
    guard = LaneGuard()
    md = make_model(-1.0, lane_width=2.6)
    settle(guard, md)
    assert settle(guard, md, active=False) == 0.0
