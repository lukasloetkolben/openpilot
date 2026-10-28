import numpy as np

from openpilot.cereal import log
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.controls.lib.ldw import CAMERA_OFFSET
from openpilot.selfdrive.modeld.constants import ModelConstants

LaneChangeState = log.LaneChangeState

# Adds a small curvature on top of the model's so the car body stays inside the lane lines.
# It stays at zero while the model's planned path keeps MARGIN clear of both lines.

CAR_HALF_WIDTH = 1.921 / 2  # m, Model Y body without mirrors
MARGIN = 0.15  # m, gap to keep between the body and a line
LOOKAHEAD_T = (1.0, 1.5, 2.0)  # s, points of the planned path that are checked, closer ones oscillate with the steering lag
MIN_LOOKAHEAD_DIST = 10.0  # m
MIN_SPEED = 8.0  # m/s
MIN_LINE_PROB = 0.7
MIN_LINE_PROB_KEEP = 0.6  # once trusted, a line stays trusted down to this
MAX_LINE_STD = 0.3  # m
MIN_LANE_WIDTH = 2.0  # m, narrower is a misdetection, the car does not fit
MAX_LANE_WIDTH = 4.5  # m, wider is a missing center line, a merge, an exit or a shoulder, only the right line is guarded
MAX_LAT_ACCEL = 1.0  # m/s^2
MAX_LAT_JERK = 4.0  # m/s^3
LANE_CHANGE_COOLDOWN = 1.0  # s, lines are reassigned after a lane change
MIN_LEFT_CURVE = 1 / 1000.  # 1/m, left curves tighter than this get the full push away from the left line, the model cuts it there
MAX_LEFT_LAT_ACCEL = 0.2  # m/s^2, push away from the left line everywhere else, weak enough for the model to pass parked cars
LANE_CURVE_T = 2.5  # s, how far ahead the lane curvature is measured


def get_lane_curvature(v_ego, left_x, left_y, right_x, right_y):
  # curvature of the lane center, positive to the right like desiredCurvature
  xs = np.linspace(5.0, max(30.0, LANE_CURVE_T * v_ego), 10)
  center = (np.interp(xs, left_x, left_y) + np.interp(xs, right_x, right_y)) / 2
  return 2 * np.polyfit(xs, center, 2)[0]


def get_lane_guard_curvature(v_ego, path_x, path_y, left_x, left_y, right_x, right_y, left_valid, right_valid):
  # The model crosses the left line on purpose to pass parked cars, and on unmarked roads that line is only the imagined
  # middle of the road. So it is only guarded on marked roads (right line seen too), and only gently outside of left curves.
  left_curve = left_valid and right_valid and get_lane_curvature(v_ego, left_x, left_y, right_x, right_y) < -MIN_LEFT_CURVE
  max_left_curvature = (MAX_LAT_ACCEL if left_curve else MAX_LEFT_LAT_ACCEL) / max(v_ego, MIN_SPEED) ** 2

  curvature = 0.0
  for t in LOOKAHEAD_T:
    x = np.interp(t, ModelConstants.T_IDXS, path_x)
    car_y = np.interp(t, ModelConstants.T_IDXS, path_y) - CAMERA_OFFSET
    left = np.interp(x, left_x, left_y)
    right = np.interp(x, right_x, right_y)
    guard_left = False
    if left_valid and right_valid:
      if right - left < MIN_LANE_WIDTH:
        continue
      # without a center line the left line is often the far road edge, keep guarding the right line (right-hand traffic)
      guard_left = right - left < MAX_LANE_WIDTH

    # Getting closer than MARGIN pushes away, and the part over the line counts twice.
    # pure pursuit: curvature that moves the path by push at distance x
    push_curvature = 2 / max(x, MIN_LOOKAHEAD_DIST) ** 2
    point_curvature = 0.0
    if guard_left:
      gap = (car_y - CAR_HALF_WIDTH) - left
      point_curvature += min(push_curvature * (max(MARGIN - gap, 0.0) + max(-gap, 0.0)), max_left_curvature)
    if right_valid:
      gap = right - (car_y + CAR_HALF_WIDTH)
      point_curvature -= push_curvature * (max(MARGIN - gap, 0.0) + max(-gap, 0.0))

    if abs(point_curvature) > abs(curvature):
      curvature = point_curvature

  max_curvature = MAX_LAT_ACCEL / max(v_ego, MIN_SPEED) ** 2
  return float(np.clip(curvature, -max_curvature, max_curvature))


class LaneGuard:
  def __init__(self, dt=DT_CTRL):
    self.dt = dt
    self.curvature = 0.0
    self.lane_change_timer = 0.0
    self.line_valid = [False, False]  # left, right

  def update(self, v_ego, model_v2, active):
    if model_v2.meta.laneChangeState != LaneChangeState.off:
      self.lane_change_timer = LANE_CHANGE_COOLDOWN
    else:
      self.lane_change_timer = max(self.lane_change_timer - self.dt, 0.0)

    target = 0.0
    lane_lines = model_v2.laneLines
    if self.lane_change_timer == 0.0 and len(lane_lines) == 4 and len(model_v2.laneLineStds) == 4:
      for i, idx in enumerate((1, 2)):
        min_prob = MIN_LINE_PROB_KEEP if self.line_valid[i] else MIN_LINE_PROB
        self.line_valid[i] = model_v2.laneLineProbs[idx] > min_prob and model_v2.laneLineStds[idx] < MAX_LINE_STD
    else:
      self.line_valid = [False, False]

    if active and v_ego > MIN_SPEED and self.lane_change_timer == 0.0 and \
       len(model_v2.position.x) == ModelConstants.IDX_N and len(lane_lines) == 4 and len(model_v2.laneLineStds) == 4:
      target = get_lane_guard_curvature(v_ego, model_v2.position.x, model_v2.position.y,
                                        lane_lines[1].x, lane_lines[1].y, lane_lines[2].x, lane_lines[2].y,
                                        *self.line_valid)

    max_change = MAX_LAT_JERK / max(v_ego, MIN_SPEED) ** 2 * self.dt
    self.curvature = float(np.clip(target, self.curvature - max_change, self.curvature + max_change))
    return self.curvature
