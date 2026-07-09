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


def create_lane_messages(packer, engaged: bool, curvature: float, heading: float, cam_left: dict, cam_right: dict):
  # The car-side LKA ECU compares the car against these lane lines to compute its steering
  # command (LANE_KEEP_ASSIST). While engaged we replace the camera's lines with a virtual lane
  # centered on the car: the ECU always sees the ideal activation picture (valid, tracked,
  # centered), which lets it advance STATUS 3 (AUTHORIZED) -> 4 (ACTIVE) and then track our
  # geometry instead of the real lane. When disengaged the real camera lines pass through
  # unchanged so the stock system keeps working.
  # Sign conventions verified on route 0000003c--f13451c457: LINE_CURVATURE is opposite to the
  # steering angle / openpilot curvature (flip), LINE_HEADING shares the angle sign (no flip).
  if not engaged:
    return [packer.make_can_msg(name, 0, dict(cam)) for name, cam in
            (('LKAS_CAM_LANE_LEFT', cam_left), ('LKAS_CAM_LANE_RIGHT', cam_right)) if cam]

  msgs = []
  for name, lat_pos in (('LKAS_CAM_LANE_LEFT', 1.75), ('LKAS_CAM_LANE_RIGHT', -1.75)):
    values = {
      'LINE_HEADING': heading,
      'LINE_CURVATURE_RATE': 0,
      'LINE_CURVATURE': -curvature,
      'LINE_QUALITY': 2,  # matches camera value seen during stock engagement
      'LINE_VALID': 1,
      'LINE_LATERAL_POSITION': lat_pos,
      'LINE_TRACKED': 1,
    }
    msgs.append(packer.make_can_msg(name, 0, values))
  return msgs
