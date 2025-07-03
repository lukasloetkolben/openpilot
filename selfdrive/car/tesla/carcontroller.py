from openpilot.common.numpy_fast import clip
from opendbc.can.packer import CANPacker
from openpilot.selfdrive.car import apply_std_steer_angle_limits
from openpilot.selfdrive.car.interfaces import CarControllerBase
from openpilot.selfdrive.car.tesla.teslacan import TeslaCAN
from openpilot.selfdrive.car.tesla.values import DBC, CarControllerParams


def torque_blended_angle(apply_angle, torsion_bar_torque):
  deadzone = CarControllerParams.TORQUE_TO_ANGLE_DEADZONE
  if abs(torsion_bar_torque) < deadzone:
    return apply_angle

  limit = CarControllerParams.TORQUE_TO_ANGLE_CLIP
  if apply_angle * torsion_bar_torque >= 0:
    # Manually steering in the same direction as OP
    strength = CarControllerParams.TORQUE_TO_ANGLE_MULTIPLIER_OUTER
  else:
    # User is opposing OP direction
    strength = CarControllerParams.TORQUE_TO_ANGLE_MULTIPLIER_INNER

  torque = torsion_bar_torque - deadzone if torsion_bar_torque > 0 else torsion_bar_torque + deadzone
  return apply_angle + clip(torque, -limit, limit) * strength

class CarController(CarControllerBase):
  def __init__(self, dbc_name, CP, FPCP, VM):
    self.CP = CP
    self.frame = 0
    self.apply_angle_last = 0
    self.last_hands_nanos = 0
    self.packer = CANPacker(dbc_name)
    self.tesla_can = TeslaCAN(self.packer)

  def update(self, CC, CS, now_nanos, frogpilot_toggles):
    actuators = CC.actuators
    pcm_cancel_cmd = CC.cruiseControl.cancel

    can_sends = []

    if self.frame % 2 == 0:
      # Detect a user override of the steering wheel when...
      CS.steering_override = (CS.hands_on_level >= 3 or  # user is applying lots of force or...
        (CS.steering_override and  # already overriding and...
         abs(CS.out.steeringAngleDeg - actuators.steeringAngleDeg) > CarControllerParams.CONTINUED_OVERRIDE_ANGLE) and
         not CS.out.standstill)  # continued angular disagreement while moving.

      # If fully hands off for 1 second then reset override (in case of continued disagreement above)
      if CS.hands_on_level > 0:
        self.last_hands_nanos = now_nanos
      elif now_nanos - self.last_hands_nanos > 1e9:
        CS.steering_override = False

      # Reset override when disengaged to ensure a fresh activation always engages steering.
      if not CC.latActive:
        CS.steering_override = False

      # Temporarily disable LKAS if user is overriding or if OP lat isn't active
      lkas_enabled = CC.latActive and not CS.steering_override

      if lkas_enabled:
        if frogpilot_toggles.virtual_torque_blending:
          # Update steering angle request with user input torque
          apply_angle = torque_blended_angle(actuators.steeringAngleDeg, CS.out.steeringTorque)
        else:
          apply_angle = actuators.steeringAngleDeg

        # Angular rate limit based on speed
        apply_angle = apply_std_steer_angle_limits(apply_angle, self.apply_angle_last, CS.out.vEgo, CarControllerParams)

        # To not fault the EPS
        apply_angle = clip(apply_angle, CS.out.steeringAngleDeg - 20, CS.out.steeringAngleDeg + 20)
      else:
        apply_angle = CS.out.steeringAngleDeg

      self.apply_angle_last = apply_angle
      can_sends.append(self.tesla_can.create_steering_control(apply_angle, lkas_enabled, (self.frame // 2) % 16))

    if not frogpilot_toggles.virtual_torque_blending:
      # Cancel on user steering override when blending is disabled
      if CS.steering_override:
        pcm_cancel_cmd = True

    # Longitudinal control
    if self.CP.openpilotLongitudinalControl:
      state = 4 if not pcm_cancel_cmd else 13  # 4=ACC_ON, 13=ACC_CANCEL_GENERIC_SILENT
      accel = clip(actuators.accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX)
      cntr = CS.das_control["DAS_controlCounter"]
      if frogpilot_toggles.hybrid_tacc and CC.longActive and (CC.hudControl.leadVisible or CS.out.gasPressed):
        can_sends.append(self.tesla_can.hybrid_longitudinal(state, accel, CS.das_control, cntr, CS.out.vEgo, CS.out.gasPressed))
      else:
        can_sends.append(self.tesla_can.create_longitudinal_command(state, accel, cntr, CC.longActive))

    # Increment counter so cancel is prioritized even without openpilot longitudinal
    if pcm_cancel_cmd and not self.CP.openpilotLongitudinalControl:
      cntr = (CS.das_control["DAS_controlCounter"] + 1) % 8
      can_sends.append(self.tesla_can.create_longitudinal_command(13, 0,  cntr, False))

    # TODO: HUD control

    new_actuators = actuators.as_builder()
    new_actuators.steeringAngleDeg = self.apply_angle_last

    self.frame += 1
    return new_actuators, can_sends
