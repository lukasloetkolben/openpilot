from cereal import car
from collections import deque
from openpilot.selfdrive.car.interfaces import CarStateBase
from opendbc.can.parser import CANParser
from openpilot.selfdrive.car.rivian.values import DBC, GEAR_MAP, BUTTONS


class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    self.button_states = {button.event_type: False for button in BUTTONS}
    self.steer_counters = deque(maxlen=32)
    self.long_counters = deque(maxlen=32)
    self.acm_status_counter = 0
    self.acm_fault_status = 0
    self.acm_feature_status = 0

  def update(self, cp, cp_cam):
    ret = car.CarState.new_message()

    ret.vEgoRaw = cp.vl["ESPiB1"]["ESPiB1_VehicleSpeed"]
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.standstill = (ret.vEgo < 0.1)

    # Gas pedal
    pedal_status = cp.vl["VDM_PropStatus"]["VDM_AcceleratorPedalPosition"]
    ret.gas = pedal_status / 100.0
    ret.gasPressed = (pedal_status > 0)

    # Brake pedal
    ret.brake = 0
    ret.brakePressed = cp.vl["iBESP2"]["iBESP2_BrakePedalApplied"] == 1

    # Steering wheel
    ret.steeringAngleDeg = cp.vl["EPAS_AdasStatus"]["EPAS_InternalSas"]
    ret.steeringRateDeg = cp.vl["EPAS_AdasStatus"]["EPAS_SteeringAngleSpeed"]
    ret.steeringTorque = cp.vl["EPAS_SystemStatus"]["EPAS_TorsionBarTorque"]
    ret.steeringPressed = abs(ret.steeringTorque) > 1.0

    # 5 = EPAS_Feature_Status_Invalid_Err
    ret.steerFaultPermanent = cp.vl["EPAS_AdasStatus"]["EPAS_EacErrorCode"] == 5
    ret.steerFaultTemporary = False # "EPAS_Angle_Control_Cntr_Err", EPAS_Angle_Control_Crc_Err

    # Cruise state
    ret.cruiseState.enabled = cp.vl["VDM_AdasSts"]["VDM_AdasDriverModeStatus"] == 1
    # ret.cruiseState.enabled = cp.vl["ACM_Status"]["ACM_FeatureStatus"] == 2
    ret.cruiseState.speed = cp.vl["ESPiB1"]["ESPiB1_VehicleSpeed"] # todo
    ret.cruiseState.available = cp.vl["VDM_AdasSts"]["VDM_AdasInterfaceStatus"] == 1
    ret.cruiseState.standstill = False  # This needs to be false, since we can resume from stop without sending anything special

    # Gear
    ret.gearShifter = GEAR_MAP[int(cp.vl["VDM_PropStatus"]["VDM_Prndl_Status"])]

    # Buttons
    button_events = []
    # for button in BUTTONS:
    #   state = (cp.vl[button.can_addr][button.can_msg] in button.values)
    #   if self.button_states[button.event_type] != state:
    #     event = car.CarState.ButtonEvent.new_message()
    #     event.type = button.event_type
    #     event.pressed = state
    #     button_events.append(event)
    #   self.button_states[button.event_type] = state
    ret.buttonEvents = button_events

    # Doors
    ret.doorOpen = False

    # Blinkers
    ret.leftBlinker = False
    ret.rightBlinker = False

    # Seatbelt
    ret.seatbeltUnlatched = False # cp.vl["RCM_Status"]["RCM_Status_IND_WARN_BELT_DRIVER"] != 0

    # Blindspot
    ret.leftBlindspot = False
    ret.rightBlindspot = False

    # AEB
    ret.stockAeb = cp_cam.vl["ACM_AebRequest"]["ACM_EnableRequest"] != 0

    # Messages needed by carcontroller
    self.steer_counters.extend(cp_cam.vl_all["ACM_SteeringControl"]["ACM_SteeringControl_Counter"])
    self.long_counters.extend(cp_cam.vl_all["ACM_longitudinalRequest"]["ACM_longitudinalRequest_Counter"])
    self.acm_fault_status = cp_cam.vl["ACM_Status"]["ACM_FaultStatus"]
    self.acm_feature_status = cp_cam.vl["ACM_Status"]["ACM_FeatureStatus"]
    self.acm_status_counter = cp_cam.vl["ACM_Status"]["ACM_Status_Counter"]
    return ret

  @staticmethod
  def get_can_parser(CP):
    messages = [
      # sig_address, frequency
      ("ESPiB1", 50),
      ("VDM_PropStatus", 50),
      ("iBESP2", 50),
      ("EPAS_AdasStatus", 100),
      ("EPAS_SystemStatus", 100),
      ("RCM_Status", 8),
      ("VDM_AdasSts", 100),
    ]

    return CANParser(DBC[CP.carFingerprint]["pt"], messages, 0)

  @staticmethod
  def get_cam_can_parser(CP):
    messages = [
      ("ACM_longitudinalRequest", 100),
      ("ACM_AebRequest", 100),
      ("ACM_SteeringControl", 100),
      ("ACM_Status", 100),
    ]

    return CANParser(DBC[CP.carFingerprint]["pt"], messages, 2)
