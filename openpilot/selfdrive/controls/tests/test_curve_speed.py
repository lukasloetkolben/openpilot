from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.controls.lib.curve_speed import SAT_MAX_OFFSET, CurveSpeed


def run(cs, saturation, seconds):
  for _ in range(int(seconds / DT_MDL)):
    out = cs.update_saturation(saturation)
  return out


def test_below_start_does_nothing():
  assert run(CurveSpeed(DT_MDL), 0.75, 5.0) == 0.0


def test_starts_gently_and_keeps_braking_while_saturated():
  cs = CurveSpeed(DT_MDL)
  first = run(cs, 0.85, 1.0)
  assert -0.4 < first < 0.0  # gentle: well under 0.5 m/s^2 after a second
  later = run(cs, 0.85, 3.0)
  assert later < first  # still saturated: brakes harder
  assert run(cs, 1.0, 10.0) == -SAT_MAX_OFFSET


def test_closer_to_the_limit_grows_faster():
  near, at = CurveSpeed(DT_MDL), CurveSpeed(DT_MDL)
  assert run(at, 1.0, 1.5) < run(near, 0.82, 1.5)


def test_holds_while_saturation_comes_down_and_releases_below_start():
  cs = CurveSpeed(DT_MDL)
  run(cs, 0.95, 3.0)
  held = cs.sat_offset
  sat = 0.95
  for _ in range(10):  # coming down at ~0.4/s: braking works, don't add more
    sat -= 0.02
    cs.update_saturation(sat)
  assert cs.sat_offset >= held - 0.05
  assert run(cs, 0.5, 5.0) == 0.0


def test_works_at_any_speed():
  # no minimum speed: the planner passes saturation from lateral accel or torque, the offset only depends on it
  assert run(CurveSpeed(DT_MDL), 0.9, 2.0) < 0.0


def test_steer_lag():
  from openpilot.selfdrive.controls.lib.curve_speed import steer_lag
  assert steer_lag(60.0, 40.0) == 20.0 and steer_lag(-60.0, -40.0) == 20.0  # wheel behind
  assert steer_lag(40.0, 55.0) == 0.0  # past the request, unwinding: not saturated
  assert steer_lag(10.0, -5.0) == 15.0  # still on the other side


class Plan:
  def __init__(self, v, lat_accel):
    from openpilot.selfdrive.modeld.constants import ModelConstants
    self.position = type('P', (), {'t': list(ModelConstants.T_IDXS)})()
    self.velocity = type('V', (), {'x': [v] * len(ModelConstants.T_IDXS)})()
    self.orientationRate = type('O', (), {'z': [lat_accel / v] * len(ModelConstants.T_IDXS)})()


def test_curve_ahead_too_fast_brakes_and_stops_accelerating():
  cs = CurveSpeed(DT_MDL)
  for _ in range(40):
    a = cs.update(Plan(27.0, 3.5), 0.0, 0.3, 0.0)
  assert a < -0.5  # 97 km/h into a 3.5 m/s^2 curve: brake, not just coast
  cs = CurveSpeed(DT_MDL)
  for _ in range(40):
    a = cs.update(Plan(27.0, 2.0), 0.0, 0.3, 0.0)
  assert a == 0.3  # curve within the limit: the model's accel


def test_saturation_lowers_the_models_accel():
  cs = CurveSpeed(DT_MDL)
  for _ in range(60):
    a = cs.update(Plan(10.0, 0.5), 0.0, 0.5, 1.0)
  assert a < 0.0
