import numpy as np

from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.modeld.constants import ModelConstants

# In tight junction and roundabout turns the model cuts the inner corner by 20-30 cm, where it no longer sees the lane lines.
# Moves the planned path outward there, the more the tighter the turn. It stays off where the inner line is seen, the lane guard
# keeps the car inside it there, and in turns the model plans near the lateral accel limit, where the car slides wide instead.

MAX_SHIFT = 0.3  # m, how far the path is moved outward in the tightest turns
TURN_CURVATURE_BP = [1 / 40., 1 / 25.]  # 1/m, the shift starts at the first and is full at the second
TURN_T = 2.0  # s, how far along the planned path the turn is measured
INNER_LINE_PROB_BP = [0.5, 0.7]  # full shift with the inner line below the first, none above the second
PLAN_LAT_ACCEL_BP = [2.4, 2.8]  # m/s^2, full shift with the planned lateral accel below the first, none above the second
LOOKAHEAD_T = 1.0  # s
MIN_LOOKAHEAD_DIST = 10.0  # m
MIN_SPEED = 3.0  # m/s
MAX_LAT_ACCEL = 1.0  # m/s^2
MAX_LAT_JERK = 2.75  # m/s^3


def get_turn(plan_v, plan_yaw_rate):
  # the tightest curvature of the planned path within TURN_T, positive to the right like desiredCurvature, and its highest lateral accel
  idx = np.array(ModelConstants.T_IDXS) <= TURN_T
  yaw_rate, v = np.array(plan_yaw_rate)[idx], np.array(plan_v)[idx]
  curvature = yaw_rate / np.maximum(v, 1.0)
  return float(curvature[np.argmax(np.abs(curvature))]), float(np.max(np.abs(yaw_rate * v)))


def get_corner_curvature(v_ego, turn_curvature, plan_lat_accel, inner_line_prob):
  shift = np.interp(abs(turn_curvature), TURN_CURVATURE_BP, [0.0, MAX_SHIFT]) * \
          np.interp(plan_lat_accel, PLAN_LAT_ACCEL_BP, [1.0, 0.0]) * np.interp(inner_line_prob, INNER_LINE_PROB_BP, [1.0, 0.0])
  # pure pursuit: curvature that moves the path by shift at the lookahead distance, away from the inside of the turn
  curvature = -np.sign(turn_curvature) * 2 * shift / max(v_ego * LOOKAHEAD_T, MIN_LOOKAHEAD_DIST) ** 2
  max_curvature = MAX_LAT_ACCEL / max(v_ego, MIN_SPEED) ** 2
  return float(np.clip(curvature, -max_curvature, max_curvature))


class CornerCorrection:
  def __init__(self, dt=DT_CTRL):
    self.dt = dt
    self.curvature = 0.0

  def update(self, v_ego, model_v2, active):
    target = 0.0
    if active and v_ego > MIN_SPEED and len(model_v2.velocity.x) == ModelConstants.IDX_N and \
       len(model_v2.orientationRate.z) == ModelConstants.IDX_N and len(model_v2.laneLineProbs) == 4:
      turn_curvature, plan_lat_accel = get_turn(model_v2.velocity.x, model_v2.orientationRate.z)
      inner_line_prob = model_v2.laneLineProbs[2 if turn_curvature > 0 else 1]
      target = get_corner_curvature(v_ego, turn_curvature, plan_lat_accel, inner_line_prob)

    max_change = MAX_LAT_JERK / max(v_ego, MIN_SPEED) ** 2 * self.dt
    self.curvature = float(np.clip(target, self.curvature - max_change, self.curvature + max_change))
    return self.curvature
