from opendbc.can.packer import CANPacker
from openpilot.common.numpy_fast import clip
from openpilot.selfdrive.car import apply_std_steer_angle_limits
from openpilot.selfdrive.car.interfaces import CarControllerBase
from openpilot.selfdrive.car.rivian import riviancan
from openpilot.selfdrive.car.rivian.values import CarControllerParams


class CarController(CarControllerBase):
  def __init__(self, dbc_name, CP, VM):
    self.CP = CP
    self.frame = 0
    self.apply_angle_last = 0
    self.packer = CANPacker(dbc_name)

  def update(self, CC, CS, now_nanos):
    actuators = CC.actuators

    can_sends = []

    # Lateral control
    if CC.latActive:
      apply_angle = apply_std_steer_angle_limits(actuators.steeringAngleDeg, self.apply_angle_last, CS.out.vEgo, CarControllerParams)
      apply_angle = clip(apply_angle, -90, 90)
    else:
      apply_angle = CS.out.steeringAngleDeg

    self.apply_angle_last = apply_angle

    while len(CS.steer_counters) > 0:
      can_sends.append(
        riviancan.create_steering_control(self.packer, CS.steer_counters.popleft() + 1, apply_angle, CC.latActive))

    # Longitudinal control
    if self.CP.openpilotLongitudinalControl:
      while len(CS.long_counters) > 0:
        can_sends.append(riviancan.create_longitudinal_commands(self.packer, CS.long_counters.popleft() + 1, actuators.accel, CC.longActive))

    new_actuators = actuators.as_builder()
    new_actuators.steeringAngleDeg = self.apply_angle_last

    self.frame += 1
    return new_actuators, can_sends
