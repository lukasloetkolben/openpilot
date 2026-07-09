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


def create_lane_messages(packer, lat_active: bool, apply_curvature: float, cam_left: dict, cam_right: dict):
  # Re-emit the camera's real lane lines to the car, overriding only LINE_CURVATURE while steering.
  # The stock LKA ECU keeps doing its own lane-centering off the real heading/position; openpilot
  # just biases the curvature. All other fields pass through unchanged.
  # TODO: verify LINE_CURVATURE sign on car, camera convention is opposite of openpilot curvature
  msgs = []
  for name, cam in (('LKAS_CAM_LANE_LEFT', cam_left), ('LKAS_CAM_LANE_RIGHT', cam_right)):
    if not cam:
      continue
    values = dict(cam)
    if lat_active:
      values['LINE_CURVATURE'] = -apply_curvature
    msgs.append(packer.make_can_msg(name, 0, values))
  return msgs
