#!/usr/bin/env python3
from cereal import car
from panda import Panda
from openpilot.selfdrive.car.tesla.values import CANBUS, CAR
from openpilot.selfdrive.car import get_safety_config
from openpilot.selfdrive.car.interfaces import CarInterfaceBase


class CarInterface(CarInterfaceBase):
  @staticmethod
  def _get_params(ret, candidate, fingerprint, car_fw, disable_openpilot_long, experimental_long, docs):
    ret.carName = "tesla"

    # There is no safe way to do steer blending with user torque,
    # so the steering behaves like autopilot. This is not
    # how openpilot should be, hence dashcamOnly
    ret.dashcamOnly = False

    ret.steerControlType = car.CarParams.SteerControlType.angle

    ret.longitudinalActuatorDelay = 0.5 # s

    # Check if we have messages on an auxiliary panda, and that 0x2bf (DAS_control) is present on the AP powertrain bus
    # If so, we assume that it is connected to the longitudinal harness.
    ret.radarUnavailable = True

    ret.openpilotLongitudinalControl = False
    ret.safetyConfigs = [get_safety_config(car.CarParams.SafetyModel.tesla, 0)]

    ret.steerLimitTimer = 1.0
    ret.steerActuatorDelay = 0.25
    return ret

  def _update(self, c, frogpilot_toggles):
    ret, fp_ret = self.CS.update(self.cp, self.cp_cam, frogpilot_toggles)

    ret.events = self.create_common_events(ret).to_msg()

    return ret, fp_ret
