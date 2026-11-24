import numpy as np

from openpilot.cereal import log
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.controls.lib.ldw import CAMERA_OFFSET
from openpilot.selfdrive.modeld.constants import ModelConstants

LaneChangeState = log.LaneChangeState

# Adds a small lateral acceleration on top of the model's so the car body stays inside the lane lines in curves and never
# crosses a road edge. It only moves the plan to the edge of the room it has, never past the lane centre, and steers smoothly:
# the push is the mean of what the plan needs from 0.75 to 2.5 s ahead, it starts with a soft knee instead of a corner, lines
# fade in and out instead of switching, and the output is filtered and jerk limited. Replayed drives (2026-11-24) show the model
# steers back against a push within about 1.5 s, so for 2 s after a push the plan may also not carry the car past the centre.

CAR_HALF_WIDTH = 1.921 / 2  # m, Model Y body without mirrors
MARGIN = 0.15  # m, gap to keep between the body and a line or road edge
LOOKAHEAD_T = np.array([0.75, 1.0, 1.25, 1.5, 1.75, 2.0, 2.5])  # s, plan points the push is averaged over
KNEE = 0.03  # m, the push grows quadratically over the first 2 * KNEE of encroachment
MIN_SPEED = 8.0  # m/s
ROOM_MIN = 0.1  # m, no push closer than this toward a line or road edge on the other side
MIN_LINE_PROB = 0.7
MIN_LINE_PROB_KEEP = 0.6  # once trusted, a line stays trusted down to this
MAX_LINE_STD = 0.3  # m
CENTRE_LINE_PROB = 0.3  # a line this sure is enough to find the lane centre
CENTRE_LINE_STD = 0.5  # m
DEFAULT_LANE_WIDTH = 3.3  # m, until one is measured
MAX_EDGE_STD = 1.0  # m
MAX_EDGE_T = 1.5  # s, the edge 2 s ahead often runs straight on past a junction the model turns into
MIN_LANE_WIDTH = 2.0  # m, narrower is a misdetection, the car does not fit
MAX_LANE_WIDTH = 4.5  # m, wider is a missing center line, a merge, an exit or a shoulder, only the curb line is guarded
CURB_ROAD_MAX = 2.5  # m, more road than this beyond a lone curb line is a bike lane, parking lane or shoulder
MAX_LAT_ACCEL = 1.0  # m/s^2
MAX_LAT_JERK = 1.0  # m/s^3
ATTACK_TAU = 0.1  # s, filter while the push grows
RELEASE_TAU = 0.3  # s, and while it shrinks
LINE_FADE_T = 0.25  # s, lines fade in and out over this, four times faster out for a lane change
LANE_CHANGE_COOLDOWN = 1.0  # s, lines are unguarded this long after a lane change or the blinker, they are reassigned after a lane change
CENTRE_HOLD = 2.0  # s, the plan may not cross the lane centre this long after a push, fading out over the last second
MIN_CURVE = 1 / 1000.  # 1/m, lane lines are guarded in curves (radius under 1 km), where the model cuts them
MIN_STRAIGHT_SPEED = 50 / 3.6  # m/s, and on straights above this, nobody passes parked cars or makes room in a bike lane this fast
LANE_CURVE_T = 2.5  # s, how far ahead the lane curvature is measured
SHARP_BEND_EXCESS = 1 / 200.  # 1/m, a plan curving this much tighter than the lines, the same way, follows a sharp bend
SHARP_BEND_RATIO = 2.5  # and at least this many times tighter, so a cut of a gentle curve stays guarded


def ramp(x, lo, hi):
  return float(np.clip((x - lo) / (hi - lo), 0.0, 1.0))


def knee(e):
  # a hinge without a corner: 0 below 0, quadratic up to 2 * KNEE, then e - KNEE
  e = np.maximum(e, 0.0)
  return np.where(e < 2 * KNEE, e ** 2 / (4 * KNEE), e - KNEE)


def get_curvature(v_ego, lines):
  # curvature of the mean of the lines (or the plan), positive to the right like desiredCurvature
  xs = np.linspace(5.0, max(30.0, LANE_CURVE_T * v_ego), 10)
  center = np.mean([np.interp(xs, x, y) for x, y in lines], axis=0)
  return 2 * np.polyfit(xs, center, 2)[0]


def get_line_weight(v_ego, path_x, path_y, left, right, left_edge, right_edge, left_hand_traffic=False):
  # how much the lane lines are guarded (0-1) and the margin kept to them.
  # On straights the model leaves the lane on purpose, to pass parked cars or make room in a bike lane for oncoming traffic,
  # so below MIN_STRAIGHT_SPEED lane lines are only guarded in curves. On straights above it only crossing them is pushed back.
  # Without the curb line (right, left in left-hand traffic), the other one is often only the imagined middle of the road and
  # is not guarded. A lone curb line with more road beyond it is the line of a bike lane or shoulder the model moves into to
  # make room, and is not guarded either. In sharp bends, mostly at night, the far end of the lines often runs too straight
  # while the plan follows the road, so lines are not guarded there.
  lines = [line for line in (left, right) if line is not None]
  curb, curb_edge, side = (left, left_edge, -1) if left_hand_traffic else (right, right_edge, 1)
  curb_x = np.interp(MAX_EDGE_T, ModelConstants.T_IDXS, path_x)
  road_beyond = curb is not None and curb_edge is not None and side * (np.interp(curb_x, *curb_edge) - np.interp(curb_x, *curb)) > CURB_ROAD_MAX
  if curb is None or (len(lines) < 2 and road_beyond):
    return 0.0, MARGIN
  lane_curvature = get_curvature(v_ego, lines)
  plan_curvature = get_curvature(v_ego, [(path_x, path_y)])
  sharp_bend = lane_curvature * plan_curvature > 0 and abs(plan_curvature) - abs(lane_curvature) > SHARP_BEND_EXCESS and \
               abs(plan_curvature) > SHARP_BEND_RATIO * abs(lane_curvature)
  curve = ramp(abs(lane_curvature), 0.7 * MIN_CURVE, MIN_CURVE)
  fast = ramp(v_ego, MIN_STRAIGHT_SPEED - 1.0, MIN_STRAIGHT_SPEED)
  # on straight narrow lanes the body may come closer, the car barely fits with MARGIN
  return (0.0 if sharp_bend else max(curve, fast)), MARGIN * curve


def get_shift(path_x, path_y, left, right, left_edge, right_edge, room_left, room_right, centre, line_w, line_margin,
              left_hand_traffic=False):
  # lateral shift of the plan (m, positive right) needed at each LOOKAHEAD_T to keep the body clear of the lines (weighted by
  # line_w) and road edges, the plan's car position and the lane centre there. The shift moves the plan to the edge of the
  # room it has, never past the lane centre and never closer than ROOM_MIN to room_left / room_right.
  T = ModelConstants.T_IDXS
  x = np.interp(LOOKAHEAD_T, T, path_x)
  car = np.interp(LOOKAHEAD_T, T, path_y) - CAMERA_OFFSET

  def at(line):
    return np.interp(x, *line) if line is not None else None

  l, r = at(left), at(right)

  push_right, push_left = np.zeros_like(x), np.zeros_like(x)
  near = LOOKAHEAD_T <= MAX_EDGE_T
  if left_edge is not None:
    push_right = np.where(near, knee(at(left_edge) + CAR_HALF_WIDTH + MARGIN - car), 0.0)
  if right_edge is not None:
    push_left = np.where(near, knee(car - (at(right_edge) - CAR_HALF_WIDTH - MARGIN)), 0.0)

  width = r - l if l is not None and r is not None else np.full_like(x, np.nan)
  fits = ~(width < MIN_LANE_WIDTH)
  # without a center line the far line is often the other road edge, keep guarding the curb line
  wide = width >= MAX_LANE_WIDTH
  if l is not None and line_w[0] > 0:
    # getting closer than the margin pushes away, and the part over the line counts twice
    push = knee(l + CAR_HALF_WIDTH + line_margin - car) + knee(l + CAR_HALF_WIDTH - car)
    push_right = np.maximum(push_right, np.where(fits & (~wide | left_hand_traffic), line_w[0] * push, 0.0))
  if r is not None and line_w[1] > 0:
    push = knee(car - (r - CAR_HALF_WIDTH - line_margin)) + knee(car - (r - CAR_HALF_WIDTH))
    push_left = np.maximum(push_left, np.where(fits & (~wide | (not left_hand_traffic)), line_w[1] * push, 0.0))

  # squeezed from both sides, aim for the middle between them
  shift = np.where((push_right > 0) & (push_left > 0), (push_right - push_left) / 2, push_right - push_left)

  c = centre(x) if centre is not None else None
  if c is not None:
    shift = np.where(shift > 0, np.minimum(shift, np.maximum(c - car, 0.0)), np.maximum(shift, np.minimum(c - car, 0.0)))
  if room_right is not None:
    shift = np.where(shift > 0, np.minimum(shift, np.maximum(at(room_right) - CAR_HALF_WIDTH - ROOM_MIN - car, 0.0)), shift)
  if room_left is not None:
    shift = np.where(shift < 0, np.maximum(shift, np.minimum(at(room_left) + CAR_HALF_WIDTH + ROOM_MIN - car, 0.0)), shift)
  return shift, car, c


def get_centre_hold(path_x, car, shift, c, room_left, room_right, push_side):
  # shift that keeps the pushed plan from carrying the car past the lane centre c, without taking it back closer than
  # MARGIN to the line (or road edge) it was pushed away from
  x = np.interp(LOOKAHEAD_T, ModelConstants.T_IDXS, path_x)
  hold = -push_side * knee(push_side * (car + shift - c))
  room = room_left if push_side > 0 else room_right
  if room is not None:
    limit = np.interp(x, *room) + push_side * (CAR_HALF_WIDTH + MARGIN) - (car + shift)
    hold = np.maximum(hold, np.minimum(limit, 0.0)) if push_side > 0 else np.minimum(hold, np.maximum(limit, 0.0))
  return hold


def shift_to_accel(shift):
  # mean of the constant lateral accelerations that would move the plan by shift at each lookahead, twice pure pursuit
  return float(np.mean(4 * shift / np.maximum(LOOKAHEAD_T, 0.5) ** 2))


class LaneGuard:
  def __init__(self, dt=DT_CTRL):
    self.dt = dt
    self.accel = 0.0  # m/s^2, output
    self.filtered = 0.0
    self.lane_change_timer = 0.0
    self.line_valid = [False, False]  # left, right
    self.line_w = np.zeros(2)
    self.lines_w = 0.0
    self.lane_width = DEFAULT_LANE_WIDTH
    self.push_side = 0.0
    self.push_timer = 0.0

  def get_centre(self, model_v2):
    # lane centre: between both lines when they are seen at all, else half a lane width from the one trusted line
    probs, stds, lines = model_v2.laneLineProbs, model_v2.laneLineStds, model_v2.laneLines
    left, right = (lines[1].x, lines[1].y), (lines[2].x, lines[2].y)
    seen = [probs[idx] > CENTRE_LINE_PROB and stds[idx] < CENTRE_LINE_STD for idx in (1, 2)]
    good = [probs[idx] > MIN_LINE_PROB and stds[idx] < MAX_LINE_STD for idx in (1, 2)]
    if all(seen):
      width = np.interp(10.0, *right) - np.interp(10.0, *left)
      if all(good) and MIN_LANE_WIDTH < width < MAX_LANE_WIDTH:
        self.lane_width += (width - self.lane_width) * min(self.dt / 2.0, 1.0)
      return lambda x: (np.interp(x, *left) + np.interp(x, *right)) / 2
    if good[0]:
      return lambda x: np.interp(x, *left) + self.lane_width / 2
    if good[1]:
      return lambda x: np.interp(x, *right) - self.lane_width / 2
    return None

  def update(self, v_ego, model_v2, active, blinker=False, left_hand_traffic=False):
    # a blinker means the driver is about to leave the lane, also below the lane change speed. Road edges stay guarded.
    if blinker or model_v2.meta.laneChangeState != LaneChangeState.off:
      self.lane_change_timer = LANE_CHANGE_COOLDOWN
    else:
      self.lane_change_timer = max(self.lane_change_timer - self.dt, 0.0)

    lane_lines, road_edges = model_v2.laneLines, model_v2.roadEdges
    has_lines = len(lane_lines) == 4 and len(model_v2.laneLineStds) == 4
    if self.lane_change_timer == 0.0 and has_lines:
      for i, idx in enumerate((1, 2)):
        min_prob = MIN_LINE_PROB_KEEP if self.line_valid[i] else MIN_LINE_PROB
        self.line_valid[i] = bool(model_v2.laneLineProbs[idx] > min_prob and model_v2.laneLineStds[idx] < MAX_LINE_STD)
    else:
      self.line_valid = [False, False]
    fade = self.dt / LINE_FADE_T
    for i in range(2):
      fade_out = 4 * fade if self.lane_change_timer > 0.0 else fade
      self.line_w[i] = float(np.clip(self.line_valid[i], self.line_w[i] - fade_out, self.line_w[i] + fade))

    target = 0.0
    has_plan = len(model_v2.position.x) == ModelConstants.IDX_N
    has_edges = len(road_edges) == 2 and len(model_v2.roadEdgeStds) == 2
    if active and v_ego > MIN_SPEED and has_lines and has_plan and has_edges:
      path_x, path_y = np.asarray(model_v2.position.x), np.asarray(model_v2.position.y)
      lines = [(lane_lines[idx].x, lane_lines[idx].y) for idx in (1, 2)]
      edges = [(edge.x, edge.y) if std < MAX_EDGE_STD else None for edge, std in zip(road_edges, model_v2.roadEdgeStds, strict=True)]
      trusted = [line if valid else None for line, valid in zip(lines, self.line_valid, strict=True)]
      line_w, line_margin = get_line_weight(v_ego, path_x, path_y, *trusted, *edges, left_hand_traffic)
      if not np.isfinite(line_w):
        line_w = 0.0
      self.lines_w = float(np.clip(line_w, self.lines_w - fade, self.lines_w + fade))

      # never push toward a line (or the road edge where there is none) the plan is already close to
      room = [line if model_v2.laneLineProbs[idx] > MIN_LINE_PROB and model_v2.laneLineStds[idx] < MAX_LINE_STD else edge
              for idx, line, edge in zip((1, 2), lines, edges, strict=True)]
      centre = self.get_centre(model_v2)
      guarded = [line if w > 0 else None for line, w in zip(lines, self.line_w, strict=True)]
      shift, car, c = get_shift(path_x, path_y, *guarded, *edges, *room, centre, self.line_w * self.lines_w, line_margin,
                                left_hand_traffic)
      target = shift_to_accel(shift)
      if not np.isfinite(target):
        target = 0.0

      if abs(target) > 0.02:
        self.push_side, self.push_timer = float(np.sign(target)), CENTRE_HOLD
      else:
        self.push_timer = max(self.push_timer - self.dt, 0.0)
      # a push gives the car lateral speed toward the centre, and the model often keeps it: hold the plan at the centre
      if self.push_timer > 0.0 and c is not None:
        hold = get_centre_hold(path_x, car, shift, c, *room, self.push_side)
        target += shift_to_accel(hold) * min(self.push_timer, 1.0)
    else:
      self.lines_w = max(self.lines_w - fade, 0.0)
      self.push_timer = max(self.push_timer - self.dt, 0.0)

    if not np.isfinite(target):
      target = 0.0
    target = float(np.clip(target, -MAX_LAT_ACCEL, MAX_LAT_ACCEL))
    tau = ATTACK_TAU if abs(target) > abs(self.filtered) and target * self.filtered >= 0.0 else RELEASE_TAU
    self.filtered += (target - self.filtered) * min(self.dt / tau, 1.0)
    if target == 0.0 and abs(self.filtered) < 1e-3:
      self.filtered = 0.0
    max_change = MAX_LAT_JERK * self.dt
    self.accel = float(np.clip(self.filtered, self.accel - max_change, self.accel + max_change))
    return self.accel / max(v_ego, MIN_SPEED) ** 2
