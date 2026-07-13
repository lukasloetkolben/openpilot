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
    self.cam_heading_offset = 0.  # rad, learned camera heading baseline (mounting yaw / road crown)
    self.heading_last = 0.  # rad, for the slew limit on the sent heading

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
          # heading correction; it decays to zero as the car reaches the commanded curvature.
          curvature = apply_curvature
          heading_err = float(np.clip(apply_curvature - current_curvature, -CarControllerParams.HEADING_ERROR, CarControllerParams.HEADING_ERROR))
          correction = heading_err * CS.out.vEgoRaw * CarControllerParams.HEADING_LOOKAHEAD
        else:
          # activation phase (STATUS 3, or driver override): virtual lane centered on the car's
          # current motion, so the ECU always sees the ideal picture to advance STATUS 3 -> 4
          curvature = current_curvature
          correction = 0.
        # stay inside the safety TX bounds (absolute cap + speed-scaled lateral accel cap)
        max_curvature = min(CarControllerParams.CURVATURE_LIMITS.CURVATURE_MAX,
                            CarControllerParams.CURVATURE_LIMITS.MAX_LATERAL_ACCEL / max(CS.out.vEgoRaw, 1.) ** 2)
        curvature = float(np.clip(curvature, -max_curvature, max_curvature))
        # the camera's LINE_HEADING is the lane angle ~18 m ahead, so the ECU expects
        # heading ~ baseline + curvature * preview; send the same structure or it reads our
        # geometry as contradictory (heading dominates and the curvature request is ignored)
        heading = self.cam_heading_offset + curvature * CarControllerParams.HEADING_PREVIEW_DIST + correction
        heading = float(np.clip(heading, -CarControllerParams.HEADING_MAX, CarControllerParams.HEADING_MAX))
        # slew limit: the ECU's heading gain is high (~119 deg/rad) with 0.3-0.8 s lag, so fast
        # clamp-to-clamp heading swings drive a limit cycle; move camera-like smoothly instead
        heading = float(np.clip(heading, self.heading_last - CarControllerParams.HEADING_RATE,
                                self.heading_last + CarControllerParams.HEADING_RATE))
        self.heading_last = heading
        can_sends.extend(create_lane_messages(self.packer, True, curvature, heading, CS.cam_lane_left, CS.cam_lane_right))
      else:
        # learn the camera's heading baseline (mounting yaw / road crown) from well-tracked real
        # lines: the heading minus its own curvature-preview component. Frozen while engaged.
        for cam in (CS.cam_lane_left, CS.cam_lane_right):
          if cam and cam['LINE_VALID'] and cam['LINE_TRACKED'] and cam['LINE_QUALITY'] >= 2 and \
             abs(cam['LINE_HEADING']) < 0.3 and abs(cam['LINE_CURVATURE']) < 0.05:
            target = cam['LINE_HEADING'] - cam['LINE_CURVATURE'] * CarControllerParams.HEADING_PREVIEW_DIST
            self.cam_heading_offset += CarControllerParams.HEADING_OFFSET_ALPHA * (target - self.cam_heading_offset)
        self.cam_heading_offset = float(np.clip(self.cam_heading_offset, -CarControllerParams.HEADING_OFFSET_MAX,
                                                CarControllerParams.HEADING_OFFSET_MAX))
        # track the would-be heading so engagement starts without a slew-limited jump
        self.heading_last = float(np.clip(self.cam_heading_offset + current_curvature * CarControllerParams.HEADING_PREVIEW_DIST,
                                          -CarControllerParams.HEADING_MAX, CarControllerParams.HEADING_MAX))
        # disengaged: pass the real camera lane lines through so the stock system keeps working
        can_sends.extend(create_lane_messages(self.packer, False, 0., 0., CS.cam_lane_left, CS.cam_lane_right))

    new_actuators = actuators.as_builder()
    new_actuators.curvature = float(self.apply_curvature_last)
    self.frame += 1
    return new_actuators, can_sends
