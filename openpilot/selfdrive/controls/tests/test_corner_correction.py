import numpy as np
from types import SimpleNamespace

from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.controls.lib.corner_correction import MAX_LAT_JERK, CornerCorrection
from openpilot.selfdrive.modeld.constants import ModelConstants

V_EGO = 7.0  # m/s, a junction turn at 2.2 m/s^2
TURN = 1 / 22.  # 1/m, positive to the right


def make_model(curvature=TURN, v=V_EGO, inner_line_prob=0.1):
  # constant speed plan along a curve of curvature (1/m), positive to the right
  n = len(ModelConstants.T_IDXS)
  probs = [0.5, inner_line_prob, inner_line_prob, 0.5]
  return SimpleNamespace(velocity=SimpleNamespace(x=[v] * n), orientationRate=SimpleNamespace(z=[curvature * v] * n), laneLineProbs=probs)


def settle(correction, md, v_ego=V_EGO, active=True, steps=300):
  for _ in range(steps):
    curvature = correction.update(v_ego, md, active)
  return curvature


class TestCornerCorrection(OpenpilotTestCase):
  def test_steers_out_of_tight_turns(self):
    # positive curvature steers right, out of a left turn
    assert settle(CornerCorrection(), make_model(TURN)) < 0.0
    assert settle(CornerCorrection(), make_model(-TURN)) > 0.0

  def test_off_in_gentle_curves(self):
    assert settle(CornerCorrection(), make_model(1 / 60.)) == 0.0

  def test_off_where_inner_line_seen(self):
    # the lane guard keeps the car inside a seen line
    assert settle(CornerCorrection(), make_model(inner_line_prob=0.9)) == 0.0

  def test_off_near_lateral_accel_limit(self):
    # a fast turn the model plans near the limit, the car slides wide there instead of cutting
    v = 11.0
    assert settle(CornerCorrection(), make_model(1 / 22., v=v), v_ego=v) == 0.0

  def test_tighter_turns_shift_more(self):
    assert settle(CornerCorrection(), make_model(1 / 22.)) < settle(CornerCorrection(), make_model(1 / 32.)) < 0.0

  def test_limits_and_ramps_out(self):
    correction = CornerCorrection()
    md = make_model()
    first = correction.update(V_EGO, md, True)
    assert 0.0 > first >= -MAX_LAT_JERK / V_EGO ** 2 * correction.dt - 1e-12
    settle(correction, md)
    assert settle(correction, md, active=False) == 0.0

  def test_off_at_standstill(self):
    assert settle(CornerCorrection(), make_model(), v_ego=1.0) == 0.0

  def test_missing_plan(self):
    md = make_model()
    md.velocity.x = []
    assert np.isfinite(settle(CornerCorrection(), md))
    assert settle(CornerCorrection(), md) == 0.0
