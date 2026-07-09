import math
import numpy as np
from opendbc.can.packer import CANPacker
from opendbc.car import Bus
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.psa.psacan import create_lane_messages
from opendbc.car.psa.values import CarControllerParams
from opendbc.car.vehicle_model import VehicleModel


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.packer = CANPacker(dbc_names[Bus.main])
    self.VM = VehicleModel(CP)
    self.apply_curvature_last = 0.

  def update(self, CC, CS, now_nanos):
    can_sends = []
    actuators = CC.actuators

    # lateral control: inject lane lines into the LKAS camera messages,
    # the car-side LKA ECU computes the steering angle request from them
    if self.frame % CarControllerParams.STEER_STEP == 0:
      apply_curvature = actuators.curvature

      # limit deviation from measured curvature (from steering angle, no yaw rate on CAN)
      if CS.out.vEgoRaw > 9:
        current_curvature = self.VM.calc_curvature(math.radians(CS.out.steeringAngleDeg), CS.out.vEgoRaw, 0.)
        apply_curvature = float(np.clip(apply_curvature, current_curvature - CarControllerParams.CURVATURE_ERROR,
                                        current_curvature + CarControllerParams.CURVATURE_ERROR))

      apply_curvature = CarControllerParams.CURVATURE_LIMITS.apply_limits(apply_curvature, self.apply_curvature_last, CS.out.vEgoRaw,
                                                                          0., CC.latActive, CarControllerParams.STEER_STEP)
      self.apply_curvature_last = apply_curvature

      can_sends.extend(create_lane_messages(self.packer, CC.latActive, apply_curvature))

    new_actuators = actuators.as_builder()
    new_actuators.curvature = float(self.apply_curvature_last)
    self.frame += 1
    return new_actuators, can_sends
