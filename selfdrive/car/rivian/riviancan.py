def checksum(data, prefix, poly):
  crc = 0
  data = bytes(prefix) + data
  for byte in data:
    crc ^= byte
    for _ in range(8):
      if crc & 0x80:
        crc = (crc << 1) ^ poly
      else:
        crc <<= 1
      crc &= 0xFF
  return crc ^ 0xFF


def create_steering_control(packer, frame, apply_steer, lkas):
  values = {
    "ACM_SteeringControl_Counter": frame % 15,
    "ACM_EacEnabled": 1 if lkas else 0,
    "ACM_HapticRequired": 0,
    "ACM_SteeringAngleRequest": apply_steer,
  }

  data = packer.make_can_msg("ACM_SteeringControl", 0, values)[2]
  values["ACM_SteeringControl_Checksum"] = checksum(data[1:], 0x02, 0x1D)
  return packer.make_can_msg("ACM_SteeringControl", 0, values)


def create_longitudinal_commands(packer, frame, accel, acc_enabled):
  values = {
    "ACM_longitudinalRequest_Counter": frame % 15,
    "ACM_AccelerationRequest": accel,
    "ACM_VehicleHoldRequired": 0,  # todo
    "ACM_PrndRequired": 0,  # todo
    "ACM_longInterfaceEnable": acc_enabled,
    "ACM_AccelerationRequestType": 0,
  }
  data = packer.make_can_msg("ACM_longitudinalRequest", 0, values)[2]
  values["ACM_longitudinalRequest_Checksum"] = checksum(data[1:], 0x3C, 0x1D)
  return packer.make_can_msg("ACM_longitudinalRequest", 0, values)


def create_button_cmd(packer, frame, button):
  values = {}

  return packer.make_can_msg("", 0, values)
