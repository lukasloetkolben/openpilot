import numpy as np

from openpilot.cereal import log
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.controls.lib.ldw import CAMERA_OFFSET
from openpilot.selfdrive.modeld.constants import ModelConstants

LaneChangeState = log.LaneChangeState

# Adds a small curvature on top of the model's so the car body stays inside the lane lines in curves and never
# crosses a road edge. It stays at zero while the model's planned path keeps MARGIN clear of them.

CAR_HALF_WIDTH = 1.921 / 2  # m, Model Y body without mirrors
MARGIN = 0.15  # m, gap to keep between the body and a line or road edge
LOOKAHEAD_T = (1.0, 1.5, 2.0)  # s, points of the planned path that are checked, closer ones oscillate with the steering lag
MIN_LOOKAHEAD_DIST = 10.0  # m
MIN_SPEED = 8.0  # m/s
MIN_LINE_PROB = 0.7
MIN_LINE_PROB_KEEP = 0.6  # once trusted, a line stays trusted down to this
MAX_LINE_STD = 0.3  # m
MAX_EDGE_STD = 1.0  # m
MIN_LANE_WIDTH = 2.0  # m, narrower is a misdetection, the car does not fit
MAX_LANE_WIDTH = 4.5  # m, wider is a missing center line, a merge, an exit or a shoulder, only the right line is guarded
MAX_LAT_ACCEL = 1.0  # m/s^2
MAX_LAT_JERK = 4.0  # m/s^3
LANE_CHANGE_COOLDOWN = 1.0  # s, off this long after a lane change or the blinker, lines are reassigned after a lane change
MIN_CURVE = 1 / 2000.  # 1/m, lane lines are only guarded in curves, where the model cuts them
LANE_CURVE_T = 2.5  # s, how far ahead the lane curvature is measured


def get_lane_curvature(v_ego, lines):
  # curvature of the mean of the lines, positive to the right like desiredCurvature
  xs = np.linspace(5.0, max(30.0, LANE_CURVE_T * v_ego), 10)
  center = np.mean([np.interp(xs, x, y) for x, y in lines], axis=0)
  return 2 * np.polyfit(xs, center, 2)[0]


def get_lane_guard_curvature(v_ego, path_x, path_y, left, right, left_edge, right_edge):
  # left, right, left_edge, right_edge are (x, y) or None when not trusted.
  # On straights the model leaves the lane on purpose, to pass parked cars or make room in a bike lane for oncoming traffic,
  # so lane lines are only guarded in curves. Without the right line, the left one is often only the imagined middle of the
  # road and is not guarded. Road edges are always guarded.
  lines = [line for line in (left, right) if line is not None]
  curve = right is not None and abs(get_lane_curvature(v_ego, lines)) > MIN_CURVE

  curvature = 0.0
  for t in LOOKAHEAD_T:
    x = np.interp(t, ModelConstants.T_IDXS, path_x)
    car_y = np.interp(t, ModelConstants.T_IDXS, path_y) - CAMERA_OFFSET
    gaps_left = [(car_y - CAR_HALF_WIDTH) - np.interp(x, *left_edge)] if left_edge is not None else []
    gaps_right = [np.interp(x, *right_edge) - (car_y + CAR_HALF_WIDTH)] if right_edge is not None else []
    if curve:
      right_y = np.interp(x, *right)
      if left is None:
        gaps_right.append(right_y - (car_y + CAR_HALF_WIDTH))
      else:
        left_y = np.interp(x, *left)
        if right_y - left_y >= MIN_LANE_WIDTH:
          gaps_right.append(right_y - (car_y + CAR_HALF_WIDTH))
          # without a center line the left line is often the far road edge, keep guarding the right line (right-hand traffic)
          if right_y - left_y < MAX_LANE_WIDTH:
            gaps_left.append((car_y - CAR_HALF_WIDTH) - left_y)

    # Getting closer than MARGIN pushes away, and the part over the line counts twice.
    # pure pursuit: curvature that moves the path by push at distance x
    push_curvature = 2 / max(x, MIN_LOOKAHEAD_DIST) ** 2
    point_curvature = 0.0
    if gaps_left:
      gap = min(gaps_left)
      point_curvature += push_curvature * (max(MARGIN - gap, 0.0) + max(-gap, 0.0))
    if gaps_right:
      gap = min(gaps_right)
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

  def update(self, v_ego, model_v2, active, blinker=False):
    # a blinker means the driver is about to leave the lane, also below the lane change speed
    if blinker or model_v2.meta.laneChangeState != LaneChangeState.off:
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

    road_edges = model_v2.roadEdges
    if active and v_ego > MIN_SPEED and self.lane_change_timer == 0.0 and len(model_v2.position.x) == ModelConstants.IDX_N and \
       len(lane_lines) == 4 and len(model_v2.laneLineStds) == 4 and len(road_edges) == 2 and len(model_v2.roadEdgeStds) == 2:
      left, right = ((lane_lines[idx].x, lane_lines[idx].y) if self.line_valid[i] else None for i, idx in enumerate((1, 2)))
      left_edge, right_edge = ((edge.x, edge.y) if std < MAX_EDGE_STD else None for edge, std in zip(road_edges, model_v2.roadEdgeStds, strict=True))
      target = get_lane_guard_curvature(v_ego, model_v2.position.x, model_v2.position.y, left, right, left_edge, right_edge)

    max_change = MAX_LAT_JERK / max(v_ego, MIN_SPEED) ** 2 * self.dt
    self.curvature = float(np.clip(target, self.curvature - max_change, self.curvature + max_change))
    return self.curvature
