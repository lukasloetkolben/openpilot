import numpy as np
from opendbc.can import CANPacker
from opendbc.car import Bus
from opendbc.car.lateral import apply_steer_angle_limits_vm
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.rivian.riviancan import create_lka_steering, create_longitudinal, create_wheel_touch, create_adas_status, create_angle_steering, create_acm_status
from opendbc.car.rivian.values import CarControllerParams
from opendbc.car.vehicle_model import VehicleModel

def get_safety_CP():
  from opendbc.car.tesla.interface import CarInterface
  return CarInterface.get_non_essential_params("RIVIAN_R1_GEN1")

class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.apply_angle_last = 0
    self.packer = CANPacker(dbc_names[Bus.pt])
    self.active_frames = 0
    self.cancel_frames = 0
    self.VM = VehicleModel(get_safety_CP())

  def update(self, CC, CS, now_nanos):
    actuators = CC.actuators
    can_sends = []

    self.active_frames = self.active_frames + 1 if CC.enabled else 0

    can_sends.append(create_lka_steering(self.packer, self.frame, CS.acm_lka_hba_cmd))

    desired_angle = np.clip(actuators.steeringAngleDeg, -90, 90)
    self.apply_angle_last = apply_steer_angle_limits_vm(desired_angle, self.apply_angle_last, CS.out.vEgoRaw, CS.out.steeringAngleDeg, CC.latActive, CarControllerParams, self.VM)

    # send steering command
    lat_active = 0 if self.active_frames < 25 else 1
    can_sends.append(create_angle_steering(self.packer, self.frame, self.apply_angle_last, lat_active))

    if not CC.enabled:
      acm_status = 0
    else:
      acm_status = 1 if self.active_frames < 25 else 2
    can_sends.append(create_acm_status(self.packer, self.frame, acm_status))

    if self.frame % 5 == 0:
      can_sends.append(create_wheel_touch(self.packer, CS.sccm_wheel_touch, CC.enabled))

    # Longitudinal control
    if self.CP.openpilotLongitudinalControl:
      accel = float(CS.acm_longitudinal_request["ACM_AccelerationRequest"]) if CC.enabled else 0
      accel = accel if not CS.out.gasPressed else 0
      accel = float(np.clip(accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))
      can_sends.append(create_longitudinal(self.packer, self.frame, accel, CC.enabled))
    else:
      interface_status = None
      if CC.cruiseControl.cancel:
        # if there is a noEntry, we need to send a status of "available" before the ACM will accept "unavailable"
        # send "available" right away as the VDM itself takes a few frames to acknowledge
        interface_status = 1 if self.cancel_frames < 5 else 0
        self.cancel_frames += 1
      else:
        self.cancel_frames = 0

      can_sends.append(create_adas_status(self.packer, CS.vdm_adas_status, interface_status))

    new_actuators = actuators.as_builder()
    new_actuators.steeringAngleDeg = self.apply_angle_last

    self.frame += 1
    return new_actuators, can_sends
