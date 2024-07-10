def crc8(data, poly, xor_output):
  crc = 0
  for byte in data:
    crc ^= byte
    for _ in range(8):
      if crc & 0x80:
        crc = (crc << 1) ^ poly
      else:
        crc <<= 1
      crc &= 0xFF
  return crc ^ xor_output


def create_steering_control(packer, frame, apply_steer, lkas):
  values = {
    "ACM_SteeringControl_Counter": frame % 15,
    "ACM_EacEnabled": 1 if lkas else 0,
    "ACM_HapticRequired": 0,
    "ACM_SteeringAngleRequest": apply_steer,
  }

  data = packer.make_can_msg("ACM_SteeringControl", 0, values)[2]
  values["ACM_SteeringControl_Checksum"] = crc8(data[1:], 0x1D, 0x41)
  return packer.make_can_msg("ACM_SteeringControl", 0, values)

def create_acm_status(packer, acm_fault_status, acm_feature_status, frame, lkas):
  values = {
    "ACM_Status_Counter": frame % 15,
    "ACM_FeatureStatus": 2 if lkas else acm_feature_status,
    "ACM_FaultStatus": acm_fault_status,
  }

  data = packer.make_can_msg("ACM_Status", 0, values)[2]
  values["ACM_Status_Checksum"] = crc8(data[1:], 0x1D, 0x5F)
  return packer.make_can_msg("ACM_Status", 0, values)

def create_longitudinal_commands(packer, frame, accel, enabled):
  values = {
    "ACM_longitudinalRequest_Counter": frame % 15,
    "ACM_AccelerationRequest": accel if enabled else 0,
    "ACM_VehicleHoldRequired": 0,
    "ACM_PrndRequired": 0,
    "ACM_longInterfaceEnable": 1 if enabled else 0,
    "ACM_AccelerationRequestType": 0,
  }
  data = packer.make_can_msg("ACM_longitudinalRequest", 0, values)[2]
  values["ACM_longitudinalRequest_Checksum"] = crc8(data[1:], 0x1D, 0x12)
  return packer.make_can_msg("ACM_longitudinalRequest", 0, values)


def create_button_cmd(packer, frame, button):
  values = {}

  return packer.make_can_msg("", 0, values)
