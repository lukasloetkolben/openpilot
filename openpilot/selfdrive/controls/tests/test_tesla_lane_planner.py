import numpy as np

from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.controls.lib.tesla_lane_planner import (
  InvalidReason, MAX_C0, MIN_LOOKAHEAD, MIN_SPEED, MIN_VIEW_RANGE, STALE_TIME, TeslaLanePlanner,
)

V_EGO = 25.0


def lanes(c0=0.0, c1=0.0, c2=0.0, c3=0.0, view_range=45.0, width=3.7,
          left_usage=2, right_usage=2, counter=0):
  return {
    'DAS_virtualLaneC0': c0,
    'DAS_virtualLaneC1': c1,
    'DAS_virtualLaneC2': c2,
    'DAS_virtualLaneC3': c3,
    'DAS_virtualLaneViewRange': view_range,
    'DAS_virtualLaneWidth': width,
    'DAS_leftLineUsage': left_usage,
    'DAS_rightLineUsage': right_usage,
    'DAS_lanesCounter': counter,
  }


def run(planner, n, model_curvature=0.0, v_ego=V_EGO, lane_change=False, **kw):
  """Step the planner n times with a correctly advancing DAS_lanes counter."""
  plan = None
  for i in range(n):
    plan = planner.update(lanes(counter=i % 16, **kw), v_ego, model_curvature, lane_change, DT_MDL)
  return plan


class TestTeslaLanePlanner:
  def test_straight_lane_is_zero_curvature(self):
    plan = run(TeslaLanePlanner(), 200)
    assert plan.valid
    assert abs(plan.desired_curvature) < 1e-6

  def test_engages_fully_and_reports_no_invalid_reason(self):
    planner = TeslaLanePlanner()
    plan = run(planner, 200)
    assert plan.invalid_reason == InvalidReason.NONE
    assert planner.blend == 1.0

  def test_constant_curvature_recovered(self):
    # a lane curving right with constant curvature k has y = k/2 * x^2, and pure
    # pursuit on that polynomial must return k back
    for k in (-0.004, -0.001, 0.001, 0.004):
      plan = run(TeslaLanePlanner(), 400, c2=k / 2.0)
      assert plan.valid
      assert abs(plan.desired_curvature - k) < 1e-4, (k, plan.desired_curvature)

  def test_sign_is_not_flipped(self):
    # lane center to the right of the car must ask for positive (right) curvature,
    # matching modelV2/controlsd's convention. this is the safety critical one.
    plan = run(TeslaLanePlanner(), 400, c0=0.5)
    assert plan.desired_curvature > 0
    plan = run(TeslaLanePlanner(), 400, c0=-0.5)
    assert plan.desired_curvature < 0

  def test_lateral_offset_drives_recentering(self):
    off = run(TeslaLanePlanner(), 400, c0=0.0).desired_curvature
    on = run(TeslaLanePlanner(), 400, c0=0.4).desired_curvature
    assert on > off

  def test_lookahead_never_reaches_past_view_range(self):
    for vr in (MIN_VIEW_RANGE, 30.0, 47.0):
      plan = run(TeslaLanePlanner(), 200, view_range=vr)
      assert plan.valid
      assert MIN_LOOKAHEAD - 1e-6 <= plan.lookahead <= vr + 1e-6, (vr, plan.lookahead)

  # ---- validity gating ----

  def test_no_data(self):
    planner = TeslaLanePlanner()
    plan = planner.update(None, V_EGO, 0.0, False, DT_MDL)
    assert not plan.valid
    assert plan.invalid_reason == InvalidReason.NO_DATA

  def test_lines_not_fused(self):
    plan = run(TeslaLanePlanner(), 10, left_usage=0)
    assert plan.invalid_reason == InvalidReason.LINES_NOT_FUSED
    assert not plan.valid

  def test_short_range(self):
    plan = run(TeslaLanePlanner(), 10, view_range=MIN_VIEW_RANGE - 1.0)
    assert plan.invalid_reason == InvalidReason.SHORT_RANGE

  def test_low_speed(self):
    plan = run(TeslaLanePlanner(), 10, v_ego=MIN_SPEED - 1.0)
    assert plan.invalid_reason == InvalidReason.LOW_SPEED

  def test_lane_change_hands_back(self):
    plan = run(TeslaLanePlanner(), 10, lane_change=True)
    assert plan.invalid_reason == InvalidReason.LANE_CHANGE

  def test_implausible_railed_polynomial(self):
    plan = run(TeslaLanePlanner(), 10, c0=MAX_C0 + 1.0)
    assert plan.invalid_reason == InvalidReason.IMPLAUSIBLE

  def test_frozen_counter_goes_stale(self):
    planner = TeslaLanePlanner()
    run(planner, 200)
    assert planner.plan.valid
    # APS stops publishing but the parser keeps holding the last value
    n = int(STALE_TIME / DT_MDL) + 2
    for _ in range(n):
      plan = planner.update(lanes(counter=7), V_EGO, 0.0, False, DT_MDL)
    assert plan.invalid_reason == InvalidReason.NO_DATA

  def test_speed_gate_has_hysteresis(self):
    planner = TeslaLanePlanner()
    run(planner, 200, v_ego=MIN_SPEED + 5.0)
    assert planner.speed_ok
    # dipping just below the re-engage threshold must not drop out
    run(planner, 5, v_ego=MIN_SPEED + 1.0)
    assert planner.speed_ok
    run(planner, 5, v_ego=MIN_SPEED - 0.5)
    assert not planner.speed_ok
    # and coming back to just above MIN_SPEED must not immediately re-engage
    run(planner, 5, v_ego=MIN_SPEED + 1.0)
    assert not planner.speed_ok

  # ---- handoff ----

  def test_falls_back_to_model_when_invalid(self):
    planner = TeslaLanePlanner()
    model_curv = 0.01
    for _ in range(200):
      plan = planner.update(None, V_EGO, model_curv, False, DT_MDL)
    assert not plan.valid
    assert plan.desired_curvature == model_curv

  def test_handoff_is_continuous(self):
    """No step in commanded curvature when control transfers in either direction."""
    planner = TeslaLanePlanner()
    model_curv = 0.008
    out = []

    for _ in range(100):    # model only
      out.append(planner.update(None, V_EGO, model_curv, False, DT_MDL).desired_curvature)
    for i in range(100):    # tesla lanes appear, asking for straight
      out.append(planner.update(lanes(counter=i % 16), V_EGO, model_curv, False, DT_MDL).desired_curvature)
    for _ in range(100):    # and disappear again
      out.append(planner.update(None, V_EGO, model_curv, False, DT_MDL).desired_curvature)

    steps = np.abs(np.diff(np.array(out)))
    # blend ramps over >=0.5s, so no single 50ms frame may move more than a fraction
    assert steps.max() < abs(model_curv) / 5.0, steps.max()
    assert out[0] == model_curv
    assert abs(out[-1] - model_curv) < 1e-6

  def test_reset_returns_to_model(self):
    planner = TeslaLanePlanner()
    run(planner, 200, model_curvature=0.0, c2=0.001)
    assert planner.blend == 1.0
    planner.reset()
    assert planner.blend == 0.0
    plan = planner.update(None, V_EGO, 0.005, False, DT_MDL)
    assert plan.desired_curvature == 0.005

  def test_curvature_is_clamped(self):
    # an absurd polynomial must be rejected, not passed to the actuator
    plan = run(TeslaLanePlanner(), 50, c1=0.14, view_range=MIN_VIEW_RANGE + 1.0)
    assert abs(plan.desired_curvature) < 0.05
