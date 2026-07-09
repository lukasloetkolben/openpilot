from opendbc.car import structs, Bus
from opendbc.can.parser import CANParser
# from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.psa.values import DBC, CarControllerParams
from opendbc.car.interfaces import CarStateBase

GearShifter = structs.CarState.GearShifter
TransmissionType = structs.CarParams.TransmissionType


class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    # latest real lane lines from the camera (bus 2), passed through to the car while disengaged
    self.cam_lane_left: dict = {}
    self.cam_lane_right: dict = {}
    self.lka_status = 0

  def update(self, can_parsers) -> structs.CarState:
    cp = can_parsers[Bus.main]
    # cp_adas = can_parsers[Bus.adas]  # no known messages on this car yet
    cp_cam = can_parsers[Bus.cam]
    ret = structs.CarState()

    # capture the camera's real lane lines for passthrough (see carcontroller)
    self.cam_lane_left = dict(cp_cam.vl['LKAS_CAM_LANE_LEFT'])
    self.cam_lane_right = dict(cp_cam.vl['LKAS_CAM_LANE_RIGHT'])

    # car speed
    self.parse_wheel_speeds(ret,
      cp.vl['Dyn4_FRE']['P263_VehV_VPsvValWhlFrtL'],
      cp.vl['Dyn4_FRE']['P264_VehV_VPsvValWhlFrtR'],
      cp.vl['Dyn4_FRE']['P265_VehV_VPsvValWhlBckL'],
      cp.vl['Dyn4_FRE']['P266_VehV_VPsvValWhlBckR'],
    )
    # ret.yawRate = cp_adas.vl['HS2_DYN_UCF_MDD_32D']['VITESSE_LACET_BRUTE'] * CV.DEG_TO_RAD # Not present on camera harness
    ret.standstill = not ret.vEgoRaw > 0.001

    # gas
    ret.gasPressed = cp.vl['Dyn_CMM']['P002_Com_rAPP'] > 0

    # brake
    ret.brakePressed = bool(cp.vl['Dat_BSI']['P013_MainBrake'])
    ret.parkingBrake = cp.vl['Dyn_EasyMove']['P337_Com_stPrkBrk'] == 1 # 0: disengaged, 1: engaged, 3: brake actuator moving

    # steering wheel
    ret.steeringAngleDeg = cp.vl['STEERING_ALT']['ANGLE'] # EPS
    ret.steeringRateDeg = cp.vl['STEERING_ALT']['RATE'] * (2 * cp.vl['STEERING_ALT']['RATE_SIGN'] - 1) # convert [0,1] to [-1,1] EPS: rot. speed * rot. sign
    ret.steeringTorque = cp.vl['STEERING']['DRIVER_TORQUE']
    ret.steeringTorqueEps = cp.vl['IS_DAT_DIRA']['EPS_TORQUE']
    ret.steeringPressed = self.update_steering_pressed(abs(ret.steeringTorque) > CarControllerParams.STEER_DRIVER_ALLOWANCE, 5)
    self.eps_active = cp.vl['IS_DAT_DIRA']['EPS_STATE_LKA'] == 3 # 0: Unauthorized, 1: Authorized, 2: Available, 3: Active, 4: Defect

    # cruise: no ACC on this car, use the stock LKA STATUS as engaged state.
    # 3: AUTHORIZED (driver pressed LKA, waiting for valid lane), 4: ACTIVE (ECU steering).
    # Engage on 3 so we feed the virtual lane, which lets the ECU advance to 4.
    self.lka_status = cp.vl['LANE_KEEP_ASSIST']['STATUS']
    ret.cruiseState.available = True
    ret.cruiseState.enabled = self.lka_status in (3, 4)

    # gear
    if bool(cp.vl['Dat_BSI']['P103_Com_bRevGear']):
      ret.gearShifter = GearShifter.reverse
    else:
      ret.gearShifter = GearShifter.drive

    # blinkers
    blinker = cp.vl['HS2_DAT7_BSI_612']['CDE_CLG_ET_HDC']
    ret.leftBlinker = blinker == 1
    ret.rightBlinker = blinker == 2

    # lock info
    ret.doorOpen = any((cp.vl['Dat_BSI']['DRIVER_DOOR'], cp.vl['Dat_BSI']['PASSENGER_DOOR']))
    ret.seatbeltUnlatched = cp.vl['RESTRAINTS']['DRIVER_SEATBELT'] != 2
    return ret

  @staticmethod
  def get_can_parsers(CP):
    return {
      Bus.main: CANParser(DBC[CP.carFingerprint][Bus.pt], [], 0),
      Bus.adas: CANParser(DBC[CP.carFingerprint][Bus.pt], [], 1),
      Bus.cam: CANParser(DBC[CP.carFingerprint][Bus.pt], [], 2),
    }
