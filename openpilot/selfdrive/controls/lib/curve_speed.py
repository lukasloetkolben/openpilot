import numpy as np

from opendbc.car.interfaces import ACCEL_MAX
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.modeld.constants import ModelConstants

# Brakes for curves the model plans to take faster than MAX_LAT_ACCEL. controlsd caps the lateral accel at about 3.4 m/s^2
# (3.0 plus road roll), when the model plans more the car can't follow and slides wide over the outer line.

MAX_LAT_ACCEL = 2.8  # m/s^2
MAX_DECEL = -2.0  # m/s^2
MAX_JERK = 2.0  # m/s^3, how fast the braking builds up
LOOKAHEAD_T = 3.0  # s, how far along the planned path curves are braked for
MIN_BRAKE_T = 1.0  # s, the speed of a curve already reached is lost over at least this long
MIN_SPEED = 5.0  # m/s
MIN_CURVATURE = 1e-4  # 1/m


def get_curve_accel(v_ego, plan_x, plan_v, plan_yaw_rate):
  # accel that slows to the speed each point of the planned path can be driven at with MAX_LAT_ACCEL, by the time it is reached
  idx = np.array(ModelConstants.T_IDXS) <= LOOKAHEAD_T
  curvature = np.abs(np.array(plan_yaw_rate)[idx]) / np.maximum(np.array(plan_v)[idx], 1.0)
  v_max = np.sqrt(MAX_LAT_ACCEL / np.maximum(curvature, MIN_CURVATURE))
  dist = np.maximum(np.array(plan_x)[idx], v_ego * MIN_BRAKE_T)
  accel = (v_max ** 2 - v_ego ** 2) / (2 * dist)
  return float(max(np.min(accel), MAX_DECEL))


class CurveSpeed:
  def __init__(self, dt=DT_MDL):
    self.dt = dt

  def update(self, v_ego, model_v2, active, a_prev):
    # ACCEL_MAX never wins the planner's lowest accel, so it's off
    if not active or v_ego < MIN_SPEED or len(model_v2.position.x) != ModelConstants.IDX_N or \
       len(model_v2.velocity.x) != ModelConstants.IDX_N or len(model_v2.orientationRate.z) != ModelConstants.IDX_N:
      return ACCEL_MAX
    accel = get_curve_accel(v_ego, model_v2.position.x, model_v2.velocity.x, model_v2.orientationRate.z)
    return max(accel, a_prev - MAX_JERK * self.dt)
