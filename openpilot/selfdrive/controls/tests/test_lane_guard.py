from types import SimpleNamespace

import numpy as np

from openpilot.cereal import log
from openpilot.common.realtime import DT_CTRL
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.controls.lib.lane_guard import CAR_HALF_WIDTH, LANE_CHANGE_COOLDOWN, MAX_LAT_ACCEL, MAX_LAT_JERK, TESLA_MAX_LAT_JERK, \
                                                       LaneGuard, combine
from openpilot.selfdrive.controls.lib.ldw import CAMERA_OFFSET
from openpilot.selfdrive.controls.lib.tesla_lanes import MAX_AGE, TeslaLaneReader, parse_das_lanes
from openpilot.selfdrive.modeld.constants import ModelConstants

V_EGO = 20.0
SLOW = 12.0  # m/s, below MIN_STRAIGHT_SPEED, lane lines are only guarded in curves
CURVE = 1 / 500.  # 1/m
LaneChangeState = log.LaneChangeState


def make_model(car_y=0.0, lane_width=3.0, left_prob=0.9, right_prob=0.9, lane_change=LaneChangeState.off, curve=CURVE, edge=1.0, edge_std=0.2):
  # lane centered on the car with curvature curve (1/m, positive right), planned path held at car_y (car center, m to the right),
  # road edges edge (m) outside the lane lines
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
  road_edges = md.init('roadEdges', 2)
  for i, y in enumerate((-lane_width / 2 - edge, lane_width / 2 + edge)):
    road_edges[i].x = line_x.tolist()
    road_edges[i].y = (y + curve / 2 * line_x ** 2).tolist()
  md.roadEdgeStds = [edge_std] * 2
  md.meta.laneChangeState = lane_change
  return md


def settle(guard, md, active=True, steps=500, blinker=False, left_hand_traffic=False, v_ego=V_EGO, lines=True, tesla_lane=None):
  for _ in range(steps):
    curvature = guard.update(v_ego, md, active, blinker, left_hand_traffic, lines, tesla_lane)
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

  def test_lone_curb_line_unguarded_with_road_beyond(self):
    # a bike lane or shoulder beyond the right line, the model moves into it to make room
    car_y = 1.3 - CAR_HALF_WIDTH + 0.10
    assert settle(LaneGuard(), make_model(car_y, lane_width=2.6, left_prob=0.3, edge=2.0)) < 0.0
    assert settle(LaneGuard(), make_model(car_y, lane_width=2.6, left_prob=0.3, edge=3.0)) == 0.0
    assert settle(LaneGuard(), make_model(car_y, lane_width=2.6, edge=3.0)) < 0.0
    assert settle(LaneGuard(), make_model(-car_y, lane_width=2.6, right_prob=0.3, edge=3.0), left_hand_traffic=True) == 0.0

  def test_lines_unguarded_on_slow_straights(self):
    # the model leaves the lane on purpose there, e.g. into a bike lane to make room for oncoming traffic
    car_y = 1.3 - CAR_HALF_WIDTH + 0.3
    assert settle(LaneGuard(), make_model(car_y, lane_width=2.6, curve=0.0), v_ego=SLOW) == 0.0
    assert settle(LaneGuard(), make_model(-car_y, lane_width=2.6, curve=0.0), v_ego=SLOW) == 0.0
    assert settle(LaneGuard(), make_model(car_y, lane_width=2.6, curve=-CURVE), v_ego=SLOW) < 0.0
    assert settle(LaneGuard(), make_model(-car_y, lane_width=2.6, curve=-CURVE), v_ego=SLOW) > 0.0

  def test_fast_straights_push_only_when_crossing(self):
    # no margin on straights, the car barely fits in narrow lanes with it
    inside = 1.3 - CAR_HALF_WIDTH - 0.05
    assert settle(LaneGuard(), make_model(inside, lane_width=2.6, curve=0.0)) == 0.0
    assert settle(LaneGuard(), make_model(-inside, lane_width=2.6, curve=0.0)) == 0.0
    assert settle(LaneGuard(), make_model(inside, lane_width=2.6)) < 0.0
    over = 1.3 - CAR_HALF_WIDTH + 0.05
    assert settle(LaneGuard(), make_model(over, lane_width=2.6, curve=0.0)) < 0.0
    assert settle(LaneGuard(), make_model(-over, lane_width=2.6, curve=0.0)) > 0.0
    # the lone far line is still not guarded
    assert settle(LaneGuard(), make_model(-over, lane_width=2.6, curve=0.0, right_prob=0.3)) == 0.0

  def test_lines_unguarded_in_sharp_bend_lines_miss(self):
    # the plan follows a sharp right bend whose lines run too straight, it crosses the right line only on the model's map.
    # Road edges unsure, they are guarded regardless
    car_y = 1.3 - CAR_HALF_WIDTH - 0.10
    md = make_model(car_y, lane_width=2.6, curve=1 / 200., edge_std=1.5)
    path_x = np.array(md.position.x)
    md.position.y = (car_y + CAMERA_OFFSET + 1 / 70. / 2 * path_x ** 2).tolist()
    assert settle(LaneGuard(), md) == 0.0
    # a plan cutting a gentle curve, only somewhat tighter than the lines, stays guarded
    md.position.y = (car_y + CAMERA_OFFSET + 1 / 150. / 2 * path_x ** 2).tolist()
    assert settle(LaneGuard(), md) < 0.0

  def test_road_edge_always_guarded(self):
    # body 5 cm over the road edge, on a straight and without trusted lane lines
    car_y = 1.3 + 0.1 - CAR_HALF_WIDTH + 0.05
    for curve, prob in ((0.0, 0.9), (CURVE, 0.1)):
      assert settle(LaneGuard(), make_model(car_y, lane_width=2.6, curve=curve, edge=0.1, left_prob=prob, right_prob=prob)) < 0.0
      assert settle(LaneGuard(), make_model(-car_y, lane_width=2.6, curve=curve, edge=0.1, left_prob=prob, right_prob=prob)) > 0.0

  def test_road_edge_guarded_with_blinker_and_lane_change(self):
    car_y = 1.3 + 0.1 - CAR_HALF_WIDTH + 0.05
    assert settle(LaneGuard(), make_model(car_y, lane_width=2.6, edge=0.1), blinker=True) < 0.0
    assert settle(LaneGuard(), make_model(car_y, lane_width=2.6, edge=0.1, lane_change=LaneChangeState.laneChangeStarting)) < 0.0

  def test_road_edge_ignores_far_crossing(self):
    # a turn whose plan only crosses the edge 2 s ahead, where the edge often runs straight on past the junction
    md = make_model(lane_width=2.6, curve=0.0, edge=0.5, left_prob=0.3, right_prob=0.3)
    path_x = np.array(md.position.x)
    md.position.y = (np.array(md.position.y) + 0.0006 * path_x ** 2).tolist()
    assert settle(LaneGuard(), md) == 0.0

  def test_ignores_unsure_road_edge(self):
    car_y = 1.3 + 0.1 - CAR_HALF_WIDTH + 0.05
    assert settle(LaneGuard(), make_model(car_y, lane_width=2.6, curve=0.0, edge=0.1, edge_std=1.5), v_ego=SLOW) == 0.0

  def test_narrow_lane_centered(self):
    # both margins overlap, the pushes from both lines cancel when centered
    # up to interpolating the curved path and lines on their different grids
    assert abs(settle(LaneGuard(), make_model(lane_width=2.1))) < 0.02 / V_EGO ** 2
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

  def test_left_hand_traffic_mirrors_curb_line(self):
    # the curb line is on the left: a lone right line is the imagined middle of the road, a lone left line is guarded
    car_y = 1.3 - CAR_HALF_WIDTH + 0.10
    assert settle(LaneGuard(), make_model(car_y, lane_width=2.6, left_prob=0.3), left_hand_traffic=True) == 0.0
    assert settle(LaneGuard(), make_model(-car_y, lane_width=2.6, right_prob=0.3), left_hand_traffic=True) > 0.0
    # a wide lane guards only the left line
    assert settle(LaneGuard(), make_model(2.5 - CAR_HALF_WIDTH + 0.05, lane_width=5.0), left_hand_traffic=True) == 0.0
    assert settle(LaneGuard(), make_model(-(2.5 - CAR_HALF_WIDTH + 0.05), lane_width=5.0), left_hand_traffic=True) > 0.0
    # a normal lane still guards both lines
    assert settle(LaneGuard(), make_model(car_y, lane_width=2.6), left_hand_traffic=True) < 0.0

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

  def test_nan_does_not_stick(self):
    # a NaN anywhere in the model output must not get stuck in the guard's curvature
    car_y = -(1.3 - CAR_HALF_WIDTH + 0.05)
    for name in ('line', 'path', 'edge'):
      bad = make_model(car_y, lane_width=2.6)
      if name == 'line':
        bad.laneLines[1].y = [float('nan')] * len(bad.laneLines[1].y)
      elif name == 'path':
        bad.position.y = [float('nan')] * len(bad.position.y)
      else:
        bad.roadEdges[0].y = [float('nan')] * len(bad.roadEdges[0].y)
      guard = LaneGuard()
      assert np.isfinite(settle(guard, bad, steps=5))
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


def das_lanes_bytes(width_idx=3, view=80, c0=0.0, c1=0.0, c2=0.0, c3=0.0, left_usage=2, right_usage=2):
  return bytes([width_idx << 4, view, round((c0 + 3.5) / 0.035), round((c1 + 0.2) / 0.0016), round((c2 + 0.0025) / 2e-5),
                round((c3 + 3e-5) / 2.4e-7), left_usage | (right_usage << 2), 0])


def can_event(dat, src=2, address=0x239):
  return SimpleNamespace(can=[SimpleNamespace(src=src, address=address, dat=dat)])


def tesla_only(md, lane, **kwargs):
  # Tesla lane guard alone, lane guard off
  return settle(LaneGuard(), md, lines=False, tesla_lane=lane, **kwargs)


class TestTeslaLanes(OpenpilotTestCase):
  def test_parse(self):
    lane = parse_das_lanes(das_lanes_bytes(width_idx=3, view=80, c0=0.35, c2=1 / 1000.))
    assert lane.width == 2.9375 and lane.view_range == 80.0
    assert abs(lane.coeffs[0] - 0.35) < 0.02 and abs(lane.coeffs[1]) < 0.001 and abs(lane.coeffs[2] - 1 / 1000.) < 2e-5
    assert lane.left_usage == 2 and lane.right_usage == 2
    assert parse_das_lanes(b'\x00' * 7) is None

  def test_reader_keeps_newest_and_ages_out(self):
    reader = TeslaLaneReader(sock=False)
    reader.feed([can_event(das_lanes_bytes(c0=0.35)), can_event(das_lanes_bytes(c0=-0.35))])  # newest first
    assert reader.get().coeffs[0] > 0.0
    for _ in range(int(MAX_AGE / DT_CTRL) + 1):
      reader.feed([])
    assert reader.get() is None

  def test_reader_ignores_other_bus_and_short_view(self):
    reader = TeslaLaneReader(sock=False)
    reader.feed([can_event(das_lanes_bytes(), src=0), can_event(das_lanes_bytes(), address=0x238)])
    assert reader.get() is None
    reader.feed([can_event(das_lanes_bytes(view=5))])
    assert reader.get() is None

  def test_pushes_off_tesla_lane_edge(self):
    # openpilot sees no lines at all, Tesla's 2.94 m lane is straight and centered on the car
    lane = parse_das_lanes(das_lanes_bytes(width_idx=3))
    over = 2.9375 / 2 - CAR_HALF_WIDTH + 0.10
    md = make_model(over, curve=0.0, left_prob=0.1, right_prob=0.1, edge_std=1.5)
    assert settle(LaneGuard(), md) == 0.0
    assert tesla_only(md, lane) < 0.0
    assert tesla_only(make_model(-over, curve=0.0, left_prob=0.1, right_prob=0.1, edge_std=1.5), lane) > 0.0

  def test_inside_tesla_lane_does_nothing(self):
    lane = parse_das_lanes(das_lanes_bytes(width_idx=3))
    assert tesla_only(make_model(0.1, curve=0.0), lane) == 0.0

  def test_guards_at_low_speed_and_slow_straights(self):
    # roundabouts and town roads, where the line guard is off
    lane = parse_das_lanes(das_lanes_bytes(width_idx=3))
    md = make_model(2.9375 / 2 - CAR_HALF_WIDTH + 0.10, curve=0.0)
    assert settle(LaneGuard(), md, v_ego=5.0) == 0.0
    assert tesla_only(md, lane, v_ego=5.0) < 0.0
    assert tesla_only(md, lane, v_ego=2.0) == 0.0

  def test_paused_with_blinker(self):
    lane = parse_das_lanes(das_lanes_bytes(width_idx=3))
    assert tesla_only(make_model(2.9375 / 2 - CAR_HALF_WIDTH + 0.10, curve=0.0), lane, blinker=True) == 0.0

  def test_beyond_view_range_ignored(self):
    lane = parse_das_lanes(das_lanes_bytes(width_idx=3, view=12))
    assert tesla_only(make_model(2.9375 / 2 - CAR_HALF_WIDTH + 0.10, curve=0.0), lane) == 0.0

  def test_follows_tesla_lane_in_curve(self):
    # Tesla's lane curves right with the road, a plan holding the middle of it is fine, one cutting to the inside is pushed out
    lane = parse_das_lanes(das_lanes_bytes(width_idx=3, c2=CURVE / 2))
    assert tesla_only(make_model(0.0), lane) == 0.0
    assert tesla_only(make_model(2.9375 / 2 - CAR_HALF_WIDTH), lane) < 0.0

  def test_combines_with_line_guard(self):
    # both push away from the same line: the stronger wins, not the sum
    lane = parse_das_lanes(das_lanes_bytes(width_idx=3))
    md = make_model(1.5 - CAR_HALF_WIDTH + 0.1, lane_width=3.0, curve=0.0)
    guard_only, tesla, both = settle(LaneGuard(), md), tesla_only(md, lane), settle(LaneGuard(), md, tesla_lane=lane)
    assert both == min(guard_only, tesla)
    assert combine(1.0, -0.4) == 0.6 and combine(-1.0, -0.4) == -1.0 and combine(0.0, 0.3) == 0.3

  def test_skips_a_lane_the_car_is_not_in(self):
    edge_over = 2.9375 / 2 - CAR_HALF_WIDTH
    # the lowest width code
    assert tesla_only(make_model(2.0 / 2 - CAR_HALF_WIDTH + 0.1, curve=0.0), parse_das_lanes(das_lanes_bytes(width_idx=0))) == 0.0
    # the body is already 0.4 m outside Tesla's lane
    assert tesla_only(make_model(edge_over + 0.4, curve=0.0), parse_das_lanes(das_lanes_bytes(width_idx=3))) == 0.0

  def test_skips_a_plan_going_another_way(self):
    # starts inside Tesla's lane, 1.5 s ahead it is far outside: a turn or fork Tesla's lane does not take
    lane = parse_das_lanes(das_lanes_bytes(width_idx=3))
    md = make_model(0.0, curve=0.0)
    path_x = np.array(md.position.x)
    md.position.y = (np.array(md.position.y) + 1 / 150. / 2 * path_x ** 2).tolist()
    assert tesla_only(md, lane) == 0.0

  def test_ramps_with_lower_jerk(self):
    lane = parse_das_lanes(das_lanes_bytes(width_idx=3))
    md = make_model(2.9375 / 2 - CAR_HALF_WIDTH + 0.10, curve=0.0, left_prob=0.1, right_prob=0.1, edge_std=1.5)
    guard = LaneGuard()
    first = guard.update(V_EGO, md, True, guard=False, tesla_lane=lane)
    assert 0.0 > first >= -TESLA_MAX_LAT_JERK / V_EGO ** 2 * guard.dt - 1e-12
    settle(guard, md, lines=False, tesla_lane=lane)
    assert settle(guard, md, lines=False, tesla_lane=None, steps=1) < 0.0  # ramps out, does not drop
