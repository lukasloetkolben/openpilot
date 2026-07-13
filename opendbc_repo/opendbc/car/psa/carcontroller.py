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
    self.lane_center_last = CarControllerParams.LANE_CENTER_EQ

  def update(self, CC, CS, now_nanos):
    can_sends = []
    actuators = CC.actuators

    # lateral control: the car-side LKA ECU computes the steering angle request (LANE_KEEP_ASSIST)
    # from the lane line messages, so we steer by feeding it a virtual lane (see psacan.py)
    if self.frame % CarControllerParams.STEER_STEP == 0:
      apply_curvature = actuators.curvature

      # limit deviation from measured curvature (from steering angle, no yaw rate on CAN).
      # Speed-dependent: near-open at city speed where a tight bound starves the planner
      # (lateral accel ~ err * v^2), tight at speed where wide bounds destabilize the loop.
      current_curvature = math.radians(CS.out.steeringAngleDeg) / (self.CP.steerRatio * self.CP.wheelbase)
      curvature_error = float(np.interp(CS.out.vEgoRaw, CarControllerParams.CURVATURE_ERROR_BP, CarControllerParams.CURVATURE_ERROR_V))
      apply_curvature = float(np.clip(apply_curvature, current_curvature - curvature_error,
                                      current_curvature + curvature_error))

      apply_curvature = CarControllerParams.CURVATURE_LIMITS.apply_limits(apply_curvature, self.apply_curvature_last, CS.out.vEgoRaw,
                                                                          0., CC.latActive, CarControllerParams.STEER_STEP)
      self.apply_curvature_last = apply_curvature

      if CC.enabled:
        if CC.latActive and CS.lka_status == 4:
          # ECU is ACTIVE: virtual lane centered on openpilot's desired path. The heading term is
          # the corrective (P) channel; the flipped curvature echo (see psacan.py) provides yaw
          # damping (D). This P+D structure is the empirically stable configuration - camera-
          # consistent geometry (unflipped curvature, preview/baseline heading) destabilized the
          # loop on-car every time it was tried (routes 4d/4f/51: wander, limit cycle, pull).
          curvature = apply_curvature
          heading = (apply_curvature - current_curvature) * CS.out.vEgoRaw * CarControllerParams.HEADING_LOOKAHEAD
          heading = float(np.clip(heading, -CarControllerParams.HEADING_MAX, CarControllerParams.HEADING_MAX))
          # lateral offset is the ECU's strongest channel (stock sysid ~8.5 deg/m): shift the lane
          # center toward the remaining curvature error to request the missing correction
          offset = (apply_curvature - current_curvature) * CarControllerParams.OFFSET_GAIN
          offset = float(np.clip(offset, -CarControllerParams.OFFSET_MAX, CarControllerParams.OFFSET_MAX))
        else:
          # activation phase (STATUS 3, or driver override): virtual lane centered on the car's
          # current motion, so the ECU always sees the ideal picture to advance STATUS 3 -> 4
          curvature = current_curvature
          heading = 0.
          offset = 0.
        # stay inside the safety TX bounds (absolute cap + speed-scaled lateral accel cap)
        max_curvature = min(CarControllerParams.CURVATURE_LIMITS.CURVATURE_MAX,
                            CarControllerParams.CURVATURE_LIMITS.MAX_LATERAL_ACCEL / max(CS.out.vEgoRaw, 1.) ** 2)
        curvature = float(np.clip(curvature, -max_curvature, max_curvature))
        # lane center: stock equilibrium plus the offset request, slew-limited (in the ECU's world
        # the car drifts off center slowly; jumps would look like a lost line)
        lane_center = CarControllerParams.LANE_CENTER_EQ + offset
        lane_center = float(np.clip(lane_center, self.lane_center_last - CarControllerParams.OFFSET_RATE,
                                    self.lane_center_last + CarControllerParams.OFFSET_RATE))
        self.lane_center_last = lane_center
        can_sends.extend(create_lane_messages(self.packer, True, curvature, heading, lane_center, CS.cam_lane_left, CS.cam_lane_right))
      else:
        self.lane_center_last = CarControllerParams.LANE_CENTER_EQ
        # disengaged: pass the real camera lane lines through so the stock system keeps working
        can_sends.extend(create_lane_messages(self.packer, False, 0., 0., 0., CS.cam_lane_left, CS.cam_lane_right))

    new_actuators = actuators.as_builder()
    new_actuators.curvature = float(self.apply_curvature_last)
    self.frame += 1
    return new_actuators, can_sends
