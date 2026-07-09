def psa_checksum(address: int, sig, d: bytearray) -> int:
  chk_ini = {0x452: 0x4, 0x38D: 0x7, 0x42D: 0xC}.get(address, 0xB)
  byte = sig.start_bit // 8
  d[byte] &= 0x0F if sig.start_bit % 8 >= 4 else 0xF0
  checksum = sum((b >> 4) + (b & 0xF) for b in d)
  return (chk_ini - checksum) & 0xF


def create_lka_steering(packer, lat_active: bool, apply_angle: float, status: int):
  values = {
    'DRIVE': 1,
    'STATUS': status,
    'LXA_ACTIVATION': 1,
    'TORQUE_FACTOR': lat_active * 100,
    'SET_ANGLE': apply_angle,
  }

  return packer.make_can_msg('LANE_KEEP_ASSIST', 0, values)


def create_lane_messages(packer, lat_active: bool, apply_curvature: float):
  # Lane line messages normally sent by the LKAS camera. The car-side LKA ECU derives its
  # steering angle request (LANE_KEEP_ASSIST) from them. Both lines are sent centered with
  # zero heading so only the curvature term steers.
  # TODO: verify LINE_CURVATURE sign on car, camera convention is opposite of openpilot curvature
  msgs = []
  for name, lat_pos in (('LKAS_CAM_LANE_LEFT', 1.75), ('LKAS_CAM_LANE_RIGHT', -1.75)):
    values = {
      'LINE_HEADING': 0,
      'LINE_CURVATURE_RATE': 0,
      'LINE_CURVATURE': -apply_curvature if lat_active else 0.,
      'LINE_QUALITY': 2,  # matches camera value seen during stock engagement
      'LINE_VALID': 1,
      'LINE_TRACKED': int(lat_active),
      'LINE_LATERAL_POSITION': lat_pos,
    }
    msgs.append(packer.make_can_msg(name, 0, values))
  return msgs
