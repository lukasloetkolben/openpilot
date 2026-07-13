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
  # Sign conventions: the camera's own signals are ISO (+ = left) for BOTH heading and curvature
  # (log-verified: cam curvature vs steering slope +0.92; ECU response fit +56 deg/rad heading,
  # +621 deg/(1/m) received curvature). The curvature below is nevertheless sent FLIPPED, on
  # purpose: since the virtual lane is centered on the car, the sent curvature is inherently an
  # echo of the car's own state (clamped to current +/- CURVATURE_ERROR). Flipped, that echo
  # opposes the car's rotation - yaw damping (D-term) - while the same-signed heading correction
  # steers (P-term). Sending it unflipped (ISO-"correct") makes the echo positive feedback and
  # the loop wanders or oscillates; verified on-car both ways (routes 40/4d/4f/51).
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
