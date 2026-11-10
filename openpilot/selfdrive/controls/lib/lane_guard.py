from collections import deque

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
TESLA_MIN_SPEED = 3.0  # m/s, Tesla's lane is guarded in roundabouts and junctions too
TESLA_MIN_WIDTH = 2.1  # m, the lowest width code (2.0 m) comes with lanes that are not where the car is
TESLA_MAX_START_OVER = 0.3  # m, the body may be this far outside Tesla's lane now, more and it is another lane
TESLA_MAX_OVER = 0.5  # m, a plan this far outside Tesla's lane goes another way, it is not drifting
TESLA_MAX_LAT_JERK = 0.75  # m/s^3, half the lines'
TESLA_MAX_CURVE_DIFF = 1 / 1000.  # 1/m, Tesla's lane is straighter than the road in bends (65-77 %, at most R 200 m) and lies outward
EDGE_MIN_SPEED = 3.0  # m/s, the road edge is also guarded at the car, where the plan already reads the curb too far
EDGE_MAX_SPEED = 60 / 3.6  # m/s
EDGE_MARGIN = 0.2  # m
EDGE_T = (0.0, 0.5)  # s, the car now and half a second ahead
EDGE_CLOSING_T = 0.5  # s, closing speed to the edge is measured over this and added for as long
EDGE_MIN_DIST = 5.0  # m
CURB_LINE_MAX_STD = 0.4  # m, a faint curb side line this sure is the curb face where the road edge lies behind a curb strip
CURB_LINE_STRIP = 0.6  # m, at most this far inside the road edge
CURB_LINE_MIN_CURVE = 1 / 200.  # 1/m, only in tight bends toward the curb, where the model cuts over it
CURB_LINE_MAX_DIST = 25.0  # m, the line has to be inside the edge somewhere this close
ROOM_T = (0.5, 1.0, 1.5, 2.0, 2.5)  # s, plan points checked for room on the side a push goes to
ROOM_MIN = 0.1  # m, no push toward a line or road edge the plan is this close to
ROOM_FULL = 0.3  # m, full push from this much room
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


def get_curvature(v_ego, lines):
  # curvature of the mean of the lines (or the plan), positive to the right like desiredCurvature
  xs = np.linspace(5.0, max(30.0, LANE_CURVE_T * v_ego), 10)
  center = np.mean([np.interp(xs, x, y) for x, y in lines], axis=0)
  return 2 * np.polyfit(xs, center, 2)[0]


def get_push(gap, margin):
  # getting closer than margin pushes away, and the part over the line counts twice
  return max(margin - gap, 0.0) + max(-gap, 0.0)


def get_lane_guard_curvature(v_ego, path_x, path_y, left, right, left_edge, right_edge, left_hand_traffic=False):
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
  if curb is not None and (len(lines) == 2 or not road_beyond):
    lane_curvature = get_curvature(v_ego, lines)
    plan_curvature = get_curvature(v_ego, [(path_x, path_y)])
    sharp_bend = lane_curvature * plan_curvature > 0 and abs(plan_curvature) - abs(lane_curvature) > SHARP_BEND_EXCESS and \
                 abs(plan_curvature) > SHARP_BEND_RATIO * abs(lane_curvature)
    curve = abs(lane_curvature) > MIN_CURVE
    guard_lines = (curve or v_ego > MIN_STRAIGHT_SPEED) and not sharp_bend
    line_margin = MARGIN if curve else 0.0  # on straight narrow lanes the body may come closer, the car barely fits with MARGIN

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

  max_curvature = MAX_LAT_ACCEL / max(v_ego, MIN_SPEED) ** 2
  return float(np.clip(curvature, -max_curvature, max_curvature))


def combine(a, b):
  # pushing the same way, the stronger one wins, otherwise they add up
  if a * b > 0:
    return a if abs(a) > abs(b) else b
  return a + b


def get_plan_curvature(path_x, path_y, x_max):
  xs = np.linspace(5.0, max(min(x_max, 40.0), 10.0), 8)
  return 2 * np.polyfit(xs, np.interp(xs, path_x, path_y), 2)[0]


def get_tesla_lane_curvature(v_ego, path_x, path_y, tesla_lane):
  # keeps the body inside Tesla's own lane (DAS_lanes), like the lane lines but also on straights, on wide roads and where the
  # center line is faded and openpilot sees none, and in roundabouts. Tesla's lane is skipped when it is not the one the car is
  # in: its lowest width code, the car already outside it, or a plan far outside it, which follows another way through a
  # junction or fork than Tesla's lane
  if tesla_lane.width < TESLA_MIN_WIDTH:
    return 0.0
  # on slow straights the model leaves the lane on purpose (parked cars), like the lines. In bends Tesla's lane is straighter than
  # the road and lies outward, it is only used while it bends like the plan
  plan_curvature = get_plan_curvature(path_x, path_y, tesla_lane.view_range)
  if v_ego < MIN_STRAIGHT_SPEED and abs(plan_curvature) < MIN_CURVE:
    return 0.0
  if abs(2 * tesla_lane.coeffs[2] - plan_curvature) > TESLA_MAX_CURVE_DIFF:
    return 0.0
  left, right = tesla_lane.edges()
  car_y0 = path_y[0] - CAMERA_OFFSET
  start_over = max(np.interp(path_x[0], *left) - (car_y0 - CAR_HALF_WIDTH), (car_y0 + CAR_HALF_WIDTH) - np.interp(path_x[0], *right))
  if not start_over <= TESLA_MAX_START_OVER:
    return 0.0
  margin = MARGIN if abs(get_curvature(v_ego, [(left[0], (left[1] + right[1]) / 2)])) > MIN_CURVE else 0.0
  curvature = 0.0
  for t in LOOKAHEAD_T:
    x = np.interp(t, ModelConstants.T_IDXS, path_x)
    if x > tesla_lane.view_range:
      continue
    car_y = np.interp(t, ModelConstants.T_IDXS, path_y) - CAMERA_OFFSET
    left_gap = (car_y - CAR_HALF_WIDTH) - np.interp(x, *left)
    right_gap = np.interp(x, *right) - (car_y + CAR_HALF_WIDTH)
    if not min(left_gap, right_gap) >= -TESLA_MAX_OVER:
      return 0.0
    point_curvature = 2 / max(x, MIN_LOOKAHEAD_DIST) ** 2 * (get_push(left_gap, margin) - get_push(right_gap, margin))
    if abs(point_curvature) > abs(curvature):
      curvature = point_curvature
  max_curvature = MAX_LAT_ACCEL / max(v_ego, MIN_SPEED) ** 2
  return float(np.clip(curvature, -max_curvature, max_curvature))


def get_edge_gaps(path_x, path_y, edge, side):
  # body gap to a road edge (side -1 left, 1 right) at the EDGE_T plan points, as (gap, x)
  gaps = []
  for t in EDGE_T:
    x = np.interp(t, ModelConstants.T_IDXS, path_x)
    car_y = np.interp(t, ModelConstants.T_IDXS, path_y) - CAMERA_OFFSET
    gap = (car_y - CAR_HALF_WIDTH) - np.interp(x, *edge) if side < 0 else np.interp(x, *edge) - (car_y + CAR_HALF_WIDTH)
    gaps.append((gap, x))
  return gaps


def get_edge_curvature(v_ego, gaps, closing):
  # gaps and closing (m/s toward the edge) per side. The model's road edge near the car is right, further ahead it often runs
  # too straight in bends, so the plan points 1 s ahead keep a gap the car then does not
  curvature = 0.0
  for side in (-1, 1):
    for gap, x in gaps.get(side, ()):
      push = get_push(min(gap, gap - closing.get(side, 0.0) * EDGE_CLOSING_T), EDGE_MARGIN)
      point_curvature = -side * 2 / max(x, EDGE_MIN_DIST) ** 2 * push
      if abs(point_curvature) > abs(curvature):
        curvature = point_curvature
  max_curvature = MAX_LAT_ACCEL / max(v_ego, MIN_SPEED) ** 2
  return float(np.clip(curvature, -max_curvature, max_curvature))


def get_curb_edge(path_x, path_y, line, line_prob, line_std, edge, edge_std, side):
  # In tight bends toward the curb (side -1 left, 1 right) the model often puts the road edge on the grass behind a curb strip,
  # 0.3-0.7 m past the curb face, while a faint lane line, too unsure to be guarded as a line, runs on the curb face itself.
  # Returns the road edge y with the line wherever it lies just inside the edge, or None
  if line_prob > MIN_LINE_PROB or not line_std < CURB_LINE_MAX_STD or not edge_std < MAX_EDGE_STD:
    return None
  if not side * get_plan_curvature(path_x, path_y, 30.0) > CURB_LINE_MIN_CURVE:
    return None
  edge_x, edge_y = np.asarray(edge[0], dtype=float), np.asarray(edge[1], dtype=float)
  line_y = np.interp(edge_x, *line)
  strip = side * (edge_y - line_y)
  inside = (strip >= 0.0) & (strip <= CURB_LINE_STRIP)
  if not (inside & (edge_x < CURB_LINE_MAX_DIST)).any():
    return None
  return np.where(inside, line_y, edge_y)


def get_room_scale(push, path_x, path_y, left, right):
  # a push toward a side shrinks as the plan's room to that side's line (or road edge) goes from ROOM_FULL to ROOM_MIN, so the
  # guards never squeeze the car toward the other side
  ref, side = (right, 1) if push > 0 else (left, -1)
  if push == 0.0 or ref is None:
    return 1.0
  room = np.inf
  for t in ROOM_T:
    x = np.interp(t, ModelConstants.T_IDXS, path_x)
    car_y = np.interp(t, ModelConstants.T_IDXS, path_y) - CAMERA_OFFSET
    room = min(room, np.interp(x, *ref) - (car_y + CAR_HALF_WIDTH) if side > 0 else (car_y - CAR_HALF_WIDTH) - np.interp(x, *ref))
  return float(np.clip((room - ROOM_MIN) / (ROOM_FULL - ROOM_MIN), 0.0, 1.0))


class LaneGuard:
  def __init__(self, dt=DT_CTRL):
    self.dt = dt
    self.curvature = 0.0
    self.tesla_curvature = 0.0
    self.edge_curvature = 0.0
    self.curb_line_active = False
    self.edge_gaps = {side: deque(maxlen=round(EDGE_CLOSING_T / dt) + 1) for side in (-1, 1)}  # gap at the car, for the closing speed
    self.lane_change_timer = 0.0
    self.line_valid = [False, False]  # left, right

  def update(self, v_ego, model_v2, active, blinker=False, left_hand_traffic=False, guard=True, tesla_lane=None, edge_guard=False,
             curb_line=False):
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

    has_plan = len(model_v2.position.x) == ModelConstants.IDX_N
    has_edges = len(road_edges) == 2 and len(model_v2.roadEdgeStds) == 2
    edges = [(edge.x, edge.y) for edge in road_edges]
    self.curb_line_active = False
    if curb_line and has_plan and has_lines and has_edges:
      idx, side = (1, -1) if left_hand_traffic else (2, 1)
      e = 0 if left_hand_traffic else 1
      curb_y = get_curb_edge(model_v2.position.x, model_v2.position.y, (lane_lines[idx].x, lane_lines[idx].y), model_v2.laneLineProbs[idx],
                             model_v2.laneLineStds[idx], edges[e], model_v2.roadEdgeStds[e], side)
      if curb_y is not None:
        edges[e] = (edges[e][0], curb_y)
        self.curb_line_active = True

    if active and guard and v_ego > MIN_SPEED and has_lines and has_plan and has_edges:
      left, right = ((lane_lines[idx].x, lane_lines[idx].y) if self.line_valid[i] else None for i, idx in enumerate((1, 2)))
      left_edge, right_edge = (edge if std < MAX_EDGE_STD else None for edge, std in zip(edges, model_v2.roadEdgeStds, strict=True))
      target = get_lane_guard_curvature(v_ego, model_v2.position.x, model_v2.position.y, left, right, left_edge, right_edge, left_hand_traffic)

    # Tesla's lane, paused like the lines around a blinker or lane change, ramps in and out more gently than the lines
    tesla_target = 0.0
    if active and tesla_lane is not None and v_ego > TESLA_MIN_SPEED and self.lane_change_timer == 0.0 and \
       has_plan:
      tesla_target = get_tesla_lane_curvature(v_ego, model_v2.position.x, model_v2.position.y, tesla_lane)
    max_tesla_change = TESLA_MAX_LAT_JERK / max(v_ego, MIN_SPEED) ** 2 * self.dt
    self.tesla_curvature = float(np.clip(tesla_target, self.tesla_curvature - max_tesla_change, self.tesla_curvature + max_tesla_change))
    if self.tesla_curvature != 0.0:
      max_curvature = MAX_LAT_ACCEL / max(v_ego, MIN_SPEED) ** 2
      target = float(np.clip(combine(target, self.tesla_curvature), -max_curvature, max_curvature))

    # road edges at the car, stays on with the blinker like the road edges
    edge_target = 0.0
    if active and edge_guard and EDGE_MIN_SPEED < v_ego < EDGE_MAX_SPEED and has_plan and has_edges:
      gaps, closing = {}, {}
      for side, edge, std in ((-1, edges[0], model_v2.roadEdgeStds[0]), (1, edges[1], model_v2.roadEdgeStds[1])):
        history = self.edge_gaps[side]
        if not std < MAX_EDGE_STD:
          history.clear()
          continue
        gaps[side] = get_edge_gaps(model_v2.position.x, model_v2.position.y, edge, side)
        history.append(gaps[side][0][0])
        if len(history) == history.maxlen:
          closing[side] = max((history[0] - history[-1]) / ((len(history) - 1) * self.dt), 0.0)
      edge_target = get_edge_curvature(v_ego, gaps, closing)
    else:
      for history in self.edge_gaps.values():
        history.clear()
    max_edge_change = MAX_LAT_JERK / max(v_ego, MIN_SPEED) ** 2 * self.dt
    self.edge_curvature = float(np.clip(edge_target, self.edge_curvature - max_edge_change, self.edge_curvature + max_edge_change))
    if self.edge_curvature != 0.0:
      max_curvature = MAX_LAT_ACCEL / max(v_ego, MIN_SPEED) ** 2
      target = float(np.clip(combine(target, self.edge_curvature), -max_curvature, max_curvature))

    if target != 0.0 and has_plan and has_lines and has_edges:
      sides = []
      for idx, edge, std in ((1, edges[0], model_v2.roadEdgeStds[0]), (2, edges[1], model_v2.roadEdgeStds[1])):
        if model_v2.laneLineProbs[idx] > MIN_LINE_PROB and model_v2.laneLineStds[idx] < MAX_LINE_STD:
          sides.append((lane_lines[idx].x, lane_lines[idx].y))
        else:
          sides.append(edge if std < MAX_EDGE_STD else None)
      target *= get_room_scale(target, model_v2.position.x, model_v2.position.y, *sides)

    max_change = MAX_LAT_JERK / max(v_ego, MIN_SPEED) ** 2 * self.dt
    self.curvature = float(np.clip(target, self.curvature - max_change, self.curvature + max_change))
    return self.curvature
