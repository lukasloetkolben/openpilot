#!/usr/bin/env python3
from cereal import car
from panda import Panda
from openpilot.selfdrive.car.tesla.values import CAR
from openpilot.selfdrive.car import get_safety_config
from openpilot.selfdrive.car.interfaces import CarInterfaceBase


class CarInterface(CarInterfaceBase):
  @staticmethod
  def _get_params(ret, candidate, fingerprint, car_fw, disable_openpilot_long, experimental_long, docs, params):
    ret.carName = "tesla"

    # Steer blending with user torque is done virtually, and is limited to 2Nm of torque
    # before it temporarily disables OP Lat control for higher user torque. This is not
    # how openpilot typically works, hence dashcamOnly
    ret.dashcamOnly = False

    ret.steerControlType = car.CarParams.SteerControlType.angle

    ret.longitudinalActuatorDelay = 0.15 # s
    ret.radarUnavailable = True

    if candidate in [CAR.TESLA_AP3_MODEL3, CAR.TESLA_AP3_MODELY]:
      flags = Panda.FLAG_TESLA_MODEL3_Y

      if not disable_openpilot_long:
        flags |= Panda.FLAG_TESLA_LONG_CONTROL
      ret.openpilotLongitudinalControl = not disable_openpilot_long
      ret.safetyConfigs = [get_safety_config(car.CarParams.SafetyModel.tesla, flags)]

    ret.steerLimitTimer = 1.0
    ret.steerActuatorDelay = 0.1
    return ret

  def _update(self, c, frogpilot_toggles):
    ret, fp_ret = self.CS.update(self.cp, self.cp_cam, frogpilot_toggles)

    ret.events = self.create_common_events(ret).to_msg()

    return ret, fp_ret
