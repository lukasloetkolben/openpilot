import numpy as np
from types import SimpleNamespace

from opendbc.car.interfaces import ACCEL_MAX
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.controls.lib.curve_speed import MAX_DECEL, MAX_JERK, MAX_LAT_ACCEL, CurveSpeed, get_curve_accel
from openpilot.selfdrive.modeld.constants import ModelConstants

T = np.array(ModelConstants.T_IDXS)


def make_plan(v, curvature, start_t=0.0):
  # constant speed plan, straight until start_t, then a curve of curvature (1/m)
  x = v * T
  curv = np.where(T >= start_t, curvature, 0.0)
  return SimpleNamespace(position=SimpleNamespace(x=x.tolist()), velocity=SimpleNamespace(x=[v] * len(T)),
                         orientationRate=SimpleNamespace(z=(curv * v).tolist()))


def settle(curve, v, plan, steps=100, active=True):
  a = 0.0
  for _ in range(steps):
    a = min(curve.update(v, plan, active, a), 0.5)
  return a


class TestCurveSpeed(OpenpilotTestCase):
  def test_straight_does_not_brake(self):
    assert settle(CurveSpeed(), 20.0, make_plan(20.0, 0.0)) > 0.0

  def test_curve_within_limit_does_not_brake(self):
    v = 13.0
    assert settle(CurveSpeed(), v, make_plan(v, 0.9 * MAX_LAT_ACCEL / v ** 2)) > 0.0

  def test_brakes_in_too_fast_curve(self):
    v = 13.0
    a = settle(CurveSpeed(), v, make_plan(v, 3.8 / v ** 2))
    assert MAX_DECEL <= a < -0.5

  def test_brakes_before_curve_ahead(self):
    # a curve 2 s ahead is braked for, less than one that is already reached
    v = 13.0
    ahead = settle(CurveSpeed(), v, make_plan(v, 3.8 / v ** 2, start_t=2.0))
    now = settle(CurveSpeed(), v, make_plan(v, 3.8 / v ** 2))
    assert now < ahead < 0.0

  def test_left_and_right_curves_alike(self):
    v = 13.0
    assert get_curve_accel(v, *self._arrays(make_plan(v, 3.8 / v ** 2))) == get_curve_accel(v, *self._arrays(make_plan(v, -3.8 / v ** 2)))

  def test_braking_builds_up_with_max_jerk(self):
    v = 20.0
    curve = CurveSpeed()
    first = curve.update(v, make_plan(v, 0.05), True, 0.0)
    assert abs(first - (-MAX_JERK * curve.dt)) < 1e-9

  def test_off_when_inactive_or_slow(self):
    v = 13.0
    plan = make_plan(v, 3.8 / v ** 2)
    assert CurveSpeed().update(v, plan, False, 0.0) == ACCEL_MAX
    assert CurveSpeed().update(3.0, plan, True, 0.0) == ACCEL_MAX

  @staticmethod
  def _arrays(plan):
    return plan.position.x, plan.velocity.x, plan.orientationRate.z
