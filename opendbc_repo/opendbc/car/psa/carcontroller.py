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

    # lateral control: inject lane lines into the LKAS camera messages,
    # the car-side LKA ECU computes the steering angle request from them
    if self.frame % CarControllerParams.STEER_STEP == 0:
      apply_curvature = actuators.curvature

      # limit deviation from measured curvature (from steering angle, no yaw rate on CAN).
      # Must use the exact same linear formula as the safety curvature-error check in psa.h,
      # otherwise commands at the clamp edge get blocked at speed.
      current_curvature = math.radians(CS.out.steeringAngleDeg) / (self.CP.steerRatio * self.CP.wheelbase)
      if CS.out.vEgoRaw > 9:
        apply_curvature = float(np.clip(apply_curvature, current_curvature - CarControllerParams.CURVATURE_ERROR,
                                        current_curvature + CarControllerParams.CURVATURE_ERROR))

      apply_curvature = CarControllerParams.CURVATURE_LIMITS.apply_limits(apply_curvature, self.apply_curvature_last, CS.out.vEgoRaw,
                                                                          0., CC.latActive, CarControllerParams.STEER_STEP)
      self.apply_curvature_last = apply_curvature

      # synthesize a lane heading from the remaining curvature error so the ECU has authority at speed;
      # decays to zero as the car reaches the commanded curvature (closes the loop the camera normally would).
      # Bound the error at all speeds: below the 9 m/s clamp gate the raw error is 10-40x larger and would
      # saturate the heading, over-steering the low-speed regime that already works on curvature alone.
      heading_err = float(np.clip(apply_curvature - current_curvature,
                                  -CarControllerParams.CURVATURE_ERROR, CarControllerParams.CURVATURE_ERROR))
      heading = heading_err * CS.out.vEgoRaw * CarControllerParams.HEADING_LOOKAHEAD
      heading = float(np.clip(heading, -CarControllerParams.HEADING_MAX, CarControllerParams.HEADING_MAX))

      can_sends.extend(create_lane_messages(self.packer, CC.latActive, apply_curvature, heading))

    new_actuators = actuators.as_builder()
    new_actuators.curvature = float(self.apply_curvature_last)
    self.frame += 1
    return new_actuators, can_sends
