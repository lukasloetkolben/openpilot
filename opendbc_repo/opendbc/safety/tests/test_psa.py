#!/usr/bin/env python3
import unittest

from opendbc.car.structs import CarParams
from opendbc.safety.tests.libsafety import libsafety_py
import opendbc.safety.tests.common as common
from opendbc.safety.tests.common import CANPackerSafety

LANE_KEEP_ASSIST = 0x3F2
LKAS_CAM_LANE_LEFT = 0x42B
LKAS_CAM_LANE_RIGHT = 0x44B

MAX_CURVATURE = 0.02       # 1/m, PSA_ABS_CURVATURE
MAX_HEADING = 0.10         # rad, PSA_MAX_HEADING
MAX_LATERAL_ACCEL = 3.6    # m/s^2, speed-scaled curvature cap
LANE_POS_MIN = 1.31        # m, PSA_LANE_POS_MIN (84 * 1/64)
LANE_POS_MAX = 2.19        # m, PSA_LANE_POS_MAX (140 * 1/64)


class TestPsaSafety(common.CarSafetyTest):
  # openpilot injects the lane lines; the camera's own copies are blocked from forwarding
  RELAY_MALFUNCTION_ADDRS = {0: (LKAS_CAM_LANE_LEFT, LKAS_CAM_LANE_RIGHT)}
  FWD_BLACKLISTED_ADDRS = {2: [LKAS_CAM_LANE_LEFT, LKAS_CAM_LANE_RIGHT]}
  TX_MSGS = [[LKAS_CAM_LANE_LEFT, 0], [LKAS_CAM_LANE_RIGHT, 0]]

  MAIN_BUS = 0
  ADAS_BUS = 1
  CAM_BUS = 2

  def setUp(self):
    self.packer = CANPackerSafety("psa_aee2010_r3")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.psa, 0)
    self.safety.init_tests()

  def _lane_msg(self, name: str, curvature: float, heading: float = 0.0, lat_pos: float | None = None):
    if lat_pos is None:
      lat_pos = 1.75 if name == "LKAS_CAM_LANE_LEFT" else -1.75
    values = {
      "LINE_CURVATURE": curvature,
      "LINE_HEADING": heading,
      "LINE_LATERAL_POSITION": lat_pos,
      "LINE_VALID": 1,
      "LINE_TRACKED": 1,
      "LINE_QUALITY": 2,
    }
    return self.packer.make_can_msg_safety(name, self.MAIN_BUS, values)

  def _pcm_status_msg(self, enable):
    # cruise state is the stock LKA STATUS: 3 (AUTHORIZED) and 4 (ACTIVE) count as engaged
    values = {"STATUS": 3 if enable else 2}
    return self.packer.make_can_msg_safety("LANE_KEEP_ASSIST", self.MAIN_BUS, values)

  def _speed_msg(self, speed):
    values = {"VITESSE_VEHICULE_ROUES": speed * 3.6}
    return self.packer.make_can_msg_safety("HS2_DYN_ABR_38D", self.MAIN_BUS, values)

  def _user_brake_msg(self, brake):
    values = {"P013_MainBrake": brake}
    return self.packer.make_can_msg_safety("Dat_BSI", self.MAIN_BUS, values)

  def _user_gas_msg(self, gas):
    values = {"P002_Com_rAPP": int(gas * 100)}
    return self.packer.make_can_msg_safety("Dyn_CMM", self.MAIN_BUS, values)

  def _set_speed(self, speed):
    for _ in range(6):
      self.assertTrue(self._rx(self._speed_msg(speed)))

  def test_rx_hook(self):
    # speed message has a real checksum
    for _ in range(10):
      self.assertTrue(self._rx(self._speed_msg(0)))
    msg = self._speed_msg(0)
    # invalidate checksum
    msg[0].data[5] = 0x00
    self.assertFalse(self._rx(msg))

  def test_lane_lines_passthrough_when_not_engaged(self):
    # without controls_allowed the frames are a camera passthrough and must not be bounded
    self.safety.set_controls_allowed(False)
    for name in ("LKAS_CAM_LANE_LEFT", "LKAS_CAM_LANE_RIGHT"):
      self.assertTrue(self._tx(self._lane_msg(name, 0.05, heading=0.5)))

  def test_lane_line_curvature_bounds(self):
    self.safety.set_controls_allowed(True)
    self._set_speed(5)  # low speed: absolute cap binds (ISO cap is ~0.144)
    for name in ("LKAS_CAM_LANE_LEFT", "LKAS_CAM_LANE_RIGHT"):
      for sign in (1, -1):
        self.assertTrue(self._tx(self._lane_msg(name, sign * (MAX_CURVATURE - 0.001))))
        self.assertFalse(self._tx(self._lane_msg(name, sign * (MAX_CURVATURE + 0.001))))

  def test_lane_line_curvature_speed_cap(self):
    self.safety.set_controls_allowed(True)
    self._set_speed(30)  # ISO lateral accel cap: 3.6 / 30^2 = 0.004
    iso_cap = MAX_LATERAL_ACCEL / 30 ** 2
    for name in ("LKAS_CAM_LANE_LEFT", "LKAS_CAM_LANE_RIGHT"):
      for sign in (1, -1):
        self.assertTrue(self._tx(self._lane_msg(name, sign * (iso_cap - 0.001))))
        self.assertFalse(self._tx(self._lane_msg(name, sign * (iso_cap + 0.001))))

  def test_lane_line_heading_bounds(self):
    self.safety.set_controls_allowed(True)
    self._set_speed(5)
    for name in ("LKAS_CAM_LANE_LEFT", "LKAS_CAM_LANE_RIGHT"):
      for sign in (1, -1):
        self.assertTrue(self._tx(self._lane_msg(name, 0., heading=sign * (MAX_HEADING - 0.01))))
        self.assertFalse(self._tx(self._lane_msg(name, 0., heading=sign * (MAX_HEADING + 0.01))))

  def test_lane_line_position_bounds(self):
    # the virtual line must stay a plausible lane line on its own side of the car
    self.safety.set_controls_allowed(True)
    self._set_speed(5)
    for name, side in (("LKAS_CAM_LANE_LEFT", 1), ("LKAS_CAM_LANE_RIGHT", -1)):
      self.assertTrue(self._tx(self._lane_msg(name, 0., lat_pos=side * (LANE_POS_MIN + 0.05))))
      self.assertTrue(self._tx(self._lane_msg(name, 0., lat_pos=side * (LANE_POS_MAX - 0.05))))
      self.assertFalse(self._tx(self._lane_msg(name, 0., lat_pos=side * (LANE_POS_MIN - 0.05))))
      self.assertFalse(self._tx(self._lane_msg(name, 0., lat_pos=side * (LANE_POS_MAX + 0.05))))
      self.assertFalse(self._tx(self._lane_msg(name, 0., lat_pos=-side * 1.75)))  # wrong side
      self.assertFalse(self._tx(self._lane_msg(name, 0., lat_pos=0.)))  # line under the car


if __name__ == "__main__":
    unittest.main()
