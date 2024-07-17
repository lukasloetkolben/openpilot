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
    else:
      apply_angle = CS.out.steeringAngleDeg

    apply_angle = clip(apply_angle, -488, 488)
    self.apply_angle_last = apply_angle

    while len(CS.steer_counters) > 0:
      can_sends.append(riviancan.create_steering_control(self.packer, CS.steer_counters.popleft() + 1, apply_angle, CC.latActive))

    can_sends.append(riviancan.create_acm_status(self.packer,0, CS.acm_fault_status, CS.acm_feature_status, CS.acm_status_counter + 1, CC.latActive))
    can_sends.append(riviancan.create_acm_status(self.packer,1, CS.acm_fault_status, CS.acm_feature_status, CS.acm_status_counter + 2, CC.latActive))
    can_sends.append(riviancan.create_acm_lka_hba_cmd(self.packer, CS.acm_lka_hba_cmd, CS.out.cruiseState.available))

    # Longitudinal control
    if self.CP.openpilotLongitudinalControl:
      while len(CS.long_counters) > 0:
        can_sends.append(riviancan.create_longitudinal_commands(self.packer, CS.long_counters.popleft() + 1, actuators.accel, CC.longActive))

    new_actuators = actuators.as_builder()
    new_actuators.steeringAngleDeg = self.apply_angle_last

    self.frame += 1
    return new_actuators, can_sends
