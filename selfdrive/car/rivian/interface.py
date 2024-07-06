#!/usr/bin/env python3
from cereal import car
from panda import Panda
from openpilot.selfdrive.car import get_safety_config
from openpilot.selfdrive.car.interfaces import CarInterfaceBase


class CarInterface(CarInterfaceBase):
  @staticmethod
  def _get_params(ret, candidate, fingerprint, car_fw, experimental_long, docs):
    ret.carName = "rivian"

    # There is no safe way to do steer blending with user torque,
    # so the steering behaves like autopilot. This is not
    # how openpilot should be, hence dashcamOnly
    ret.dashcamOnly = False

    ret.steerControlType = car.CarParams.SteerControlType.angle
    ret.radarUnavailable = True
    ret.longitudinalActuatorDelay = 0.5

    ret.openpilotLongitudinalControl = True
    flag = 0
    flag |= Panda.FLAG_RIVIAN_LONG_CONTROL
    ret.safetyConfigs = [get_safety_config(car.CarParams.SafetyModel.rivian, flag)]

    ret.steerLimitTimer = 1.0
    ret.steerActuatorDelay = 0.25
    return ret

  def _update(self, c):
    ret = self.CS.update(self.cp, self.cp_cam)

    ret.events = self.create_common_events(ret).to_msg()

    return ret
