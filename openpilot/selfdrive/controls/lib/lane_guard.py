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
MAX_EDGE_T = 1.5  # s, the edge 2 s ahead often runs straight on past a junction the model turns into
MIN_LANE_WIDTH = 2.0  # m, narrower is a misdetection, the car does not fit
MAX_LANE_WIDTH = 4.5  # m, wider is a missing center line, a merge, an exit or a shoulder, only the curb line is guarded
CURB_ROAD_MAX = 2.5  # m, more road than this beyond a lone curb line is a bike lane, parking lane or shoulder
MAX_LAT_ACCEL = 1.0  # m/s^2
MAX_LAT_JERK = 1.5  # m/s^3
LANE_CHANGE_COOLDOWN = 1.0  # s, lines are unguarded this long after a lane change or the blinker, they are reassigned after a lane change
MIN_CURVE = 1 / 1000.  # 1/m, lane lines are guarded in curves (radius under 1 km), where the model cuts them
MIN_STRAIGHT_SPEED = 50 / 3.6  # m/s, and on straights above this, nobody passes parked cars or makes room in a bike lane this fast
LANE_CURVE_T = 2.5  # s, how far ahead the lane curvature is measured
SHARP_BEND_EXCESS = 1 / 200.  # 1/m, a plan curving this much tighter than the lines, the same way, follows a sharp bend
SHARP_BEND_RATIO = 2.5  # and at least this many times tighter, so a cut of a gentle curve stays guarded

# De-cut: in curves the model plans the car toward the inside by a typical amount that grows with curvature (and, in right
# curves, with lateral acceleration). It is moved back by that amount, but never past the lane center and never closer than
# DECUT_OUTSIDE_ROOM to the outside line, so anything beyond the typical cut (passing, making room) stays the model's choice.
# Fit on 11 drives (right-hand traffic), plan offset at each check point:
#   right curves: a * curvature + c * curvature * v^2, left curves: b * curvature + d * curvature * v^2
DECUT_COEFFS = {1.0: (-2.672, 24.013, 0.153, 0.055), 1.5: (-0.187, 28.34, 0.119, 0.03), 2.0: (4.953, 35.264, 0.065, -0.023)}
DECUT_OUTSIDE_ROOM = MARGIN  # m, so it never moves the car into the guard's margin of the outside line
DECUT_MAX_LAT_ACCEL = 0.5  # m/s^2
DECUT_HEADING_T = (0.5, 2.0)  # s, the plan's direction in the lane is taken between these points
DECUT_HEADING_TOL = 0.02  # m, so it does not flicker on a plan running parallel to the lane


def get_typical_cut(t, curvature, v_ego, left_hand_traffic=False):
  # how far the model typically plans the car toward the inside of a curve at t, m, always >= 0
  k = -curvature if left_hand_traffic else curvature
  a, b, c, d = DECUT_COEFFS[t]
  cut = a * k + c * k * v_ego ** 2 if k > 0 else b * k + d * k * v_ego ** 2
  return max(cut * np.sign(k), 0.0)


def get_inside_offset(t, path_x, path_y, left, right, side):
  # how far the plan is toward the inside of the lane at t, m
  x = np.interp(t, ModelConstants.T_IDXS, path_x)
  car_y = np.interp(t, ModelConstants.T_IDXS, path_y) - CAMERA_OFFSET
  return side * (car_y - (np.interp(x, *left) + np.interp(x, *right)) / 2)


def get_decut_shifts(v_ego, path_x, path_y, left, right, lane_curvature, left_hand_traffic=False):
  # outward shift (m, >= 0) of the plan at each LOOKAHEAD_T, inside is the side the lane bends to
  side = np.sign(lane_curvature)
  # a plan already moving toward the outside, e.g. after earlier pushes the model has not taken back yet, is left alone,
  # or the pushes add up while the model is slow to follow
  near, far = (get_inside_offset(t, path_x, path_y, left, right, side) for t in DECUT_HEADING_T)
  if not far >= near - DECUT_HEADING_TOL:
    return side, [0.0] * len(LOOKAHEAD_T)
  shifts = []
  for t in LOOKAHEAD_T:
    x = np.interp(t, ModelConstants.T_IDXS, path_x)
    car_y = np.interp(t, ModelConstants.T_IDXS, path_y) - CAMERA_OFFSET
    left_y, right_y = np.interp(x, *left), np.interp(x, *right)
    shift = 0.0
    if MIN_LANE_WIDTH <= right_y - left_y < MAX_LANE_WIDTH:
      inside_offset = side * (car_y - (left_y + right_y) / 2)
      outside_gap = (car_y - CAR_HALF_WIDTH) - left_y if side > 0 else right_y - (car_y + CAR_HALF_WIDTH)
      shift = min(get_typical_cut(t, lane_curvature, v_ego, left_hand_traffic), max(inside_offset, 0.0), max(outside_gap - DECUT_OUTSIDE_ROOM, 0.0))
    shifts.append(float(shift) if np.isfinite(shift) else 0.0)
  return side, shifts


def get_curvature(v_ego, lines):
  # curvature of the mean of the lines (or the plan), positive to the right like desiredCurvature
  xs = np.linspace(5.0, max(30.0, LANE_CURVE_T * v_ego), 10)
  center = np.mean([np.interp(xs, x, y) for x, y in lines], axis=0)
  return 2 * np.polyfit(xs, center, 2)[0]


def get_push(gap, margin):
  # getting closer than margin pushes away, and the part over the line counts twice
  return max(margin - gap, 0.0) + max(-gap, 0.0)


def get_lane_guard_curvature(v_ego, path_x, path_y, left, right, left_edge, right_edge, left_hand_traffic=False, guard=True, decut=False):
  # left, right, left_edge, right_edge are (x, y) or None when not trusted.
  # On straights the model leaves the lane on purpose, to pass parked cars or make room in a bike lane for oncoming traffic,
  # so below MIN_STRAIGHT_SPEED lane lines are only guarded in curves. On straights above it only crossing them is pushed back.
  # Without the curb line (right, left in left-hand traffic), the other one is often only the imagined middle of the road and
  # is not guarded. A lone curb line with more road beyond it is the line of a bike lane or shoulder the model moves into to
  # make room, and is not guarded either. In sharp bends, mostly at night, the far end of the lines often runs too straight
  # while the plan follows the road, so lines are not guarded there. Road edges are always guarded.
  lines = [line for line in (left, right) if line is not None]
  curb, curb_edge, side = (left, left_edge, -1) if left_hand_traffic else (right, right_edge, 1)
  curb_x = np.interp(MAX_EDGE_T, ModelConstants.T_IDXS, path_x)
  road_beyond = curb is not None and curb_edge is not None and side * (np.interp(curb_x, *curb_edge) - np.interp(curb_x, *curb)) > CURB_ROAD_MAX
  guard_lines, line_margin = False, MARGIN
  lane_curvature, sharp_bend = 0.0, True
  if len(lines):
    lane_curvature = get_curvature(v_ego, lines)
    plan_curvature = get_curvature(v_ego, [(path_x, path_y)])
    sharp_bend = lane_curvature * plan_curvature > 0 and abs(plan_curvature) - abs(lane_curvature) > SHARP_BEND_EXCESS and \
                 abs(plan_curvature) > SHARP_BEND_RATIO * abs(lane_curvature)
  curve = abs(lane_curvature) > MIN_CURVE
  if curb is not None and (len(lines) == 2 or not road_beyond):
    guard_lines = (curve or v_ego > MIN_STRAIGHT_SPEED) and not sharp_bend
    line_margin = MARGIN if curve else 0.0  # on straight narrow lanes the body may come closer, the car barely fits with MARGIN

  # de-cut needs both lines, for the lane center and the outside line
  decut_curvature = 0.0
  if decut and len(lines) == 2 and curve and not sharp_bend:
    side, shifts = get_decut_shifts(v_ego, path_x, path_y, left, right, lane_curvature, left_hand_traffic)
    for t, shift in zip(LOOKAHEAD_T, shifts, strict=True):
      x = np.interp(t, ModelConstants.T_IDXS, path_x)
      point_curvature = -side * 2 * shift / max(x, MIN_LOOKAHEAD_DIST) ** 2
      if abs(point_curvature) > abs(decut_curvature):
        decut_curvature = point_curvature
    max_decut = DECUT_MAX_LAT_ACCEL / max(v_ego, MIN_SPEED) ** 2
    decut_curvature = float(np.clip(decut_curvature, -max_decut, max_decut))

  if not guard:
    return decut_curvature

  curvature = 0.0
  for t in LOOKAHEAD_T:
    x = np.interp(t, ModelConstants.T_IDXS, path_x)
    car_y = np.interp(t, ModelConstants.T_IDXS, path_y) - CAMERA_OFFSET
    near = t <= MAX_EDGE_T
    pushes_left = [get_push((car_y - CAR_HALF_WIDTH) - np.interp(x, *left_edge), MARGIN)] if left_edge is not None and near else []
    pushes_right = [get_push(np.interp(x, *right_edge) - (car_y + CAR_HALF_WIDTH), MARGIN)] if right_edge is not None and near else []
    if guard_lines:
      left_gap = (car_y - CAR_HALF_WIDTH) - np.interp(x, *left) if left is not None else None
      right_gap = np.interp(x, *right) - (car_y + CAR_HALF_WIDTH) if right is not None else None
      lane_width = left_gap + right_gap + 2 * CAR_HALF_WIDTH if left is not None and right is not None else None
      if lane_width is None or lane_width >= MIN_LANE_WIDTH:
        # without a center line the far line is often the other road edge, keep guarding the curb line
        wide = lane_width is not None and lane_width >= MAX_LANE_WIDTH
        if left_gap is not None and (left_hand_traffic or not wide):
          pushes_left.append(get_push(left_gap, line_margin))
        if right_gap is not None and (not left_hand_traffic or not wide):
          pushes_right.append(get_push(right_gap, line_margin))

    # pure pursuit: curvature that moves the path by push at distance x
    push_curvature = 2 / max(x, MIN_LOOKAHEAD_DIST) ** 2
    point_curvature = push_curvature * (max(pushes_left, default=0.0) - max(pushes_right, default=0.0))

    if abs(point_curvature) > abs(curvature):
      curvature = point_curvature

  # pushing the same way, the stronger one wins: the guard keeps its full push over a line, de-cut covers the cuts it ignores
  if curvature * decut_curvature > 0:
    curvature = curvature if abs(curvature) > abs(decut_curvature) else decut_curvature
  else:
    curvature += decut_curvature
  max_curvature = MAX_LAT_ACCEL / max(v_ego, MIN_SPEED) ** 2
  return float(np.clip(curvature, -max_curvature, max_curvature))


class LaneGuard:
  def __init__(self, dt=DT_CTRL):
    self.dt = dt
    self.curvature = 0.0
    self.lane_change_timer = 0.0
    self.line_valid = [False, False]  # left, right

  def update(self, v_ego, model_v2, active, blinker=False, left_hand_traffic=False, guard=True, decut=False):
    # a blinker means the driver is about to leave the lane, also below the lane change speed. Road edges stay guarded.
    if blinker or model_v2.meta.laneChangeState != LaneChangeState.off:
      self.lane_change_timer = LANE_CHANGE_COOLDOWN
    else:
      self.lane_change_timer = max(self.lane_change_timer - self.dt, 0.0)

    target = 0.0
    lane_lines, road_edges = model_v2.laneLines, model_v2.roadEdges
    has_lines = len(lane_lines) == 4 and len(model_v2.laneLineStds) == 4
    if self.lane_change_timer == 0.0 and has_lines:
      for i, idx in enumerate((1, 2)):
        min_prob = MIN_LINE_PROB_KEEP if self.line_valid[i] else MIN_LINE_PROB
        self.line_valid[i] = model_v2.laneLineProbs[idx] > min_prob and model_v2.laneLineStds[idx] < MAX_LINE_STD
    else:
      self.line_valid = [False, False]

    if active and v_ego > MIN_SPEED and has_lines and len(model_v2.position.x) == ModelConstants.IDX_N and \
       len(road_edges) == 2 and len(model_v2.roadEdgeStds) == 2:
      left, right = ((lane_lines[idx].x, lane_lines[idx].y) if self.line_valid[i] else None for i, idx in enumerate((1, 2)))
      left_edge, right_edge = ((edge.x, edge.y) if std < MAX_EDGE_STD else None for edge, std in zip(road_edges, model_v2.roadEdgeStds, strict=True))
      target = get_lane_guard_curvature(v_ego, model_v2.position.x, model_v2.position.y, left, right, left_edge, right_edge, left_hand_traffic,
                                        guard, decut)

    max_change = MAX_LAT_JERK / max(v_ego, MIN_SPEED) ** 2 * self.dt
    self.curvature = float(np.clip(target, self.curvature - max_change, self.curvature + max_change))
    return self.curvature
