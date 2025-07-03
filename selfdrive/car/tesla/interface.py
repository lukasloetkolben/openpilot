#!/usr/bin/env python3
from cereal import car
from openpilot.selfdrive.car import get_safety_config
from openpilot.selfdrive.car.interfaces import CarInterfaceBase


class CarInterface(CarInterfaceBase):
  @staticmethod
  def _get_params(ret, candidate, fingerprint, car_fw, experimental_long, docs, frogpilot_toggles):
    ret.carName = "tesla"

    ret.dashcamOnly = False

    ret.steerControlType = car.CarParams.SteerControlType.angle

    ret.longitudinalActuatorDelay = 0.5 # s
    ret.radarTimeStep = (1.0 / 8) # 8Hz

    ret.openpilotLongitudinalControl = not frogpilot_toggles.disable_openpilot_long
    ret.safetyConfigs = [
      get_safety_config(car.CarParams.SafetyModel.tesla, 0)
    ]

    ret.steerLimitTimer = 1.0
    ret.steerActuatorDelay = 0.25
    return ret

  def _update(self, c, frogpilot_toggles):
    ret, fp_ret = self.CS.update(self.cp, self.cp_cam, frogpilot_toggles)

    ret.events = self.create_common_events(ret).to_msg()

    return ret, fp_ret
