#!/usr/bin/env python3
import unittest

from opendbc.car.structs import CarParams
from opendbc.safety.tests.libsafety import libsafety_py
import opendbc.safety.tests.common as common
from opendbc.safety.tests.common import CANPackerSafety
from opendbc.car.volkswagen.values import VolkswagenSafetyFlags

# MQB Evo V1: MQB RX message IDs + MEB TX steering (HCA_03 curvature)
MSG_ESP_19     = 0xB2   # RX from ABS, for wheel speeds
MSG_LH_EPS_03 = 0x9F   # RX from EPS, for driver steering torque
MSG_ESP_05     = 0x106  # RX from ABS, for brake pressure
MSG_TSK_06     = 0x120  # RX from ECU, for ACC status
MSG_MOTOR_20   = 0x121  # RX from ECU, for driver throttle input
MSG_GRA_ACC_01 = 0x12B  # TX by OP, ACC control buttons
MSG_KLR_01     = 0x25D  # TX by OP, capacitive wheel touch
MSG_HCA_03     = 0x303  # TX by OP, curvature steering
MSG_LDW_02     = 0x397  # TX by OP, lane departure warning HUD


class TestVolkswagenMqbEvoV1SafetyBase(common.CarSafetyTest, common.CurvatureSteeringSafetyTest):
  RELAY_MALFUNCTION_ADDRS = {0: (MSG_HCA_03, MSG_LDW_02),
                             2: (MSG_LH_EPS_03, MSG_KLR_01)}

  # === curvature limits (same as MEB) ===
  MAX_CURVATURE = 29105
  MAX_CURVATURE_TEST = 0.195
  CURVATURE_TO_CAN = 149253.7313
  INACTIVE_CURVATURE_IS_ZERO = True
  MAX_POWER = 125
  MAX_POWER_TEST = 50
  SEND_RATE = 0.02

  # Wheel speeds from ESP_19 (MQB style)
  def _speed_msg(self, speed):
    values = {"ESP_%s_Radgeschw_02" % s: speed for s in ["HL", "HR", "VL", "VR"]}
    return self.packer.make_can_msg_safety("ESP_19", 0, values)

  # Brake pressure from ESP_05 only (no Motor_14 on this platform)
  def _user_brake_msg(self, brake):
    values = {"ESP_Fahrer_bremst": brake}
    return self.packer.make_can_msg_safety("ESP_05", 0, values)

  # Driver throttle input from Motor_20 (MQB style)
  def _user_gas_msg(self, gas):
    values = {"MO_Fahrpedalrohwert_01": gas}
    return self.packer.make_can_msg_safety("Motor_20", 0, values)

  def _vehicle_moving_msg(self, speed):
    return self._speed_msg(speed)

  # ACC engagement status from TSK_06 (MQB style)
  def _tsk_status_msg(self, enable, main_switch=True):
    if main_switch:
      tsk_status = 3 if enable else 2
    else:
      tsk_status = 0
    values = {"TSK_Status": tsk_status}
    return self.packer.make_can_msg_safety("TSK_06", 0, values)

  def _pcm_status_msg(self, enable):
    return self._tsk_status_msg(enable)

  # Driver steering input torque from LH_EPS_03
  def _torque_driver_msg(self, torque):
    values = {"EPS_Lenkmoment": abs(torque), "EPS_VZ_Lenkmoment": torque < 0}
    return self.packer.make_can_msg_safety("LH_EPS_03", 0, values)

  # Curvature steering command via HCA_03 (MEB style)
  def _curvature_cmd_msg(self, curvature, steer_req=1, power=50):
    values = {
      "Curvature": abs(curvature),
      "Curvature_VZ": curvature > 0,
      "RequestStatus": 4 if steer_req else 0,
      "Power": power,
    }
    return self.packer.make_can_msg_safety("HCA_03", 0, values)

  # Cruise control buttons
  def _button_msg(self, cancel=0, resume=0, _set=0, bus=2):
    values = {"GRA_Abbrechen": cancel, "GRA_Tip_Setzen": _set, "GRA_Tip_Wiederaufnahme": resume}
    return self.packer.make_can_msg_safety("GRA_ACC_01", bus, values)

  def test_brake_signal(self):
    self._rx(self._user_brake_msg(False))
    self.assertFalse(self.safety.get_brake_pressed_prev())
    self._rx(self._user_brake_msg(True))
    self.assertTrue(self.safety.get_brake_pressed_prev())


class TestVolkswagenMqbEvoV1StockSafety(TestVolkswagenMqbEvoV1SafetyBase):
  TX_MSGS = [[MSG_HCA_03, 0], [MSG_GRA_ACC_01, 0], [MSG_GRA_ACC_01, 2],
             [MSG_LDW_02, 0], [MSG_LH_EPS_03, 2], [MSG_KLR_01, 0], [MSG_KLR_01, 2]]
  FWD_BLACKLISTED_ADDRS = {0: [MSG_LH_EPS_03, MSG_KLR_01],
                           2: [MSG_HCA_03, MSG_LDW_02]}

  def setUp(self):
    self.packer = CANPackerSafety("vw_mqbevo")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.volkswagenMqbEvoV1, 0)
    self.safety.init_tests()

  def test_spam_cancel_safety_check(self):
    self.safety.set_controls_allowed(0)
    self.assertTrue(self._tx(self._button_msg(cancel=1)))
    self.assertFalse(self._tx(self._button_msg(resume=1)))
    self.assertFalse(self._tx(self._button_msg(_set=1)))
    # do not block resume if we are engaged already
    self.safety.set_controls_allowed(1)
    self.assertTrue(self._tx(self._button_msg(resume=1)))


if __name__ == "__main__":
  unittest.main()
