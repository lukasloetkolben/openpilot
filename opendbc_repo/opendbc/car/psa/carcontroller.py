import math
import numpy as np
from opendbc.can.packer import CANPacker
from opendbc.car import Bus
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.psa.psacan import create_lane_messages
from opendbc.car.psa.values import CarControllerParams


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.packer = CANPacker(dbc_names[Bus.main])
    self.apply_curvature_last = 0.

  def update(self, CC, CS, now_nanos):
    can_sends = []
    actuators = CC.actuators

    # lateral control: the car-side LKA ECU computes the steering angle request (LANE_KEEP_ASSIST)
    # from the lane line messages, so we steer by feeding it a virtual lane (see psacan.py)
    if self.frame % CarControllerParams.STEER_STEP == 0:
      apply_curvature = actuators.curvature

      # limit deviation from measured curvature (from steering angle, no yaw rate on CAN) at ALL
      # speeds: the old >9 m/s condition left the error unbounded below 32 km/h, where it reached
      # 0.011 and slammed the synthesized heading into its clamp (jerky steering, route 00000040)
      current_curvature = math.radians(CS.out.steeringAngleDeg) / (self.CP.steerRatio * self.CP.wheelbase)
      apply_curvature = float(np.clip(apply_curvature, current_curvature - CarControllerParams.CURVATURE_ERROR,
                                      current_curvature + CarControllerParams.CURVATURE_ERROR))

      apply_curvature = CarControllerParams.CURVATURE_LIMITS.apply_limits(apply_curvature, self.apply_curvature_last, CS.out.vEgoRaw,
                                                                          0., CC.latActive, CarControllerParams.STEER_STEP)
      self.apply_curvature_last = apply_curvature

      if CC.enabled:
        if CC.latActive and CS.lka_status == 4:
          # ECU is ACTIVE: virtual lane centered on openpilot's desired path. LINE_HEADING is the
          # ECU's dominant input at speed, so the remaining curvature error is synthesized into a
          # heading preview; it decays to zero as the car reaches the commanded curvature.
          curvature = apply_curvature
          heading_err = float(np.clip(apply_curvature - current_curvature, -CarControllerParams.HEADING_ERROR, CarControllerParams.HEADING_ERROR))
          heading = heading_err * CS.out.vEgoRaw * CarControllerParams.HEADING_LOOKAHEAD
          heading = float(np.clip(heading, -CarControllerParams.HEADING_MAX, CarControllerParams.HEADING_MAX))
        else:
          # activation phase (STATUS 3, or driver override): virtual lane centered on the car's
          # current motion, so the ECU always sees the ideal picture to advance STATUS 3 -> 4
          curvature = current_curvature
          heading = 0.
        # stay inside the safety TX bounds (absolute cap + speed-scaled lateral accel cap)
        max_curvature = min(CarControllerParams.CURVATURE_LIMITS.CURVATURE_MAX,
                            CarControllerParams.CURVATURE_LIMITS.MAX_LATERAL_ACCEL / max(CS.out.vEgoRaw, 1.) ** 2)
        curvature = float(np.clip(curvature, -max_curvature, max_curvature))
        can_sends.extend(create_lane_messages(self.packer, True, curvature, heading, CS.cam_lane_left, CS.cam_lane_right))
      else:
        # disengaged: pass the real camera lane lines through so the stock system keeps working
        can_sends.extend(create_lane_messages(self.packer, False, 0., 0., CS.cam_lane_left, CS.cam_lane_right))

    new_actuators = actuators.as_builder()
    new_actuators.curvature = float(self.apply_curvature_last)
    self.frame += 1
    return new_actuators, can_sends
