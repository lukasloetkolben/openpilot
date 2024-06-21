from collections import namedtuple
from dataclasses import dataclass, field

from cereal import car
from openpilot.selfdrive.car import AngleRateLimit
from openpilot.selfdrive.car import CarSpecs, DbcDict, PlatformConfig, Platforms, dbc_dict
from openpilot.selfdrive.car.docs_definitions import CarHarness, CarDocs, CarParts
from openpilot.selfdrive.car.fw_query_definitions import FwQueryConfig, Request, StdQueries

Ecu = car.CarParams.Ecu

Button = namedtuple('Button', ['event_type', 'can_addr', 'can_msg', 'values'])

class CAR(Platforms):
  RIVIAN_R1S = PlatformConfig(
    [CarDocs("Rivian R1S", "All")],
    CarSpecs(mass=3206., wheelbase=3.08, steerRatio=15.0),
    dbc_dict('rivian_can', None)
  )

FW_QUERY_CONFIG = FwQueryConfig(
  requests=[
    Request(
      [StdQueries.TESTER_PRESENT_REQUEST, StdQueries.SUPPLIER_SOFTWARE_VERSION_REQUEST],
      [StdQueries.TESTER_PRESENT_RESPONSE, StdQueries.SUPPLIER_SOFTWARE_VERSION_RESPONSE],
      bus=0,
    ),
  ]
)

GEAR_MAP = [
  car.CarState.GearShifter.unknown,
  car.CarState.GearShifter.park,
  car.CarState.GearShifter.reverse,
  car.CarState.GearShifter.neutral,
  car.CarState.GearShifter.drive,
]

BUTTONS = [
  Button(car.CarState.ButtonEvent.Type.leftBlinker, "SCCM_leftStalk", "SCCM_turnIndicatorStalkStatus", [3, 4]),
  Button(car.CarState.ButtonEvent.Type.rightBlinker, "SCCM_leftStalk", "SCCM_turnIndicatorStalkStatus", [1, 2]),
  Button(car.CarState.ButtonEvent.Type.accelCruise, "VCLEFT_switchStatus", "VCLEFT_swcRightScrollTicks", list(range(1, 10))),
  Button(car.CarState.ButtonEvent.Type.decelCruise, "VCLEFT_switchStatus", "VCLEFT_swcRightScrollTicks", list(range(-9, 0))),
  Button(car.CarState.ButtonEvent.Type.cancel, "SCCM_rightStalk", "SCCM_rightStalkStatus", [1, 2]),
  Button(car.CarState.ButtonEvent.Type.resumeCruise, "SCCM_rightStalk", "SCCM_rightStalkStatus", [3, 4]),
]


class CarControllerParams:
  ANGLE_RATE_LIMIT_UP = AngleRateLimit(speed_bp=[0., 5., 15.], angle_v=[10., 1.6, .3])
  ANGLE_RATE_LIMIT_DOWN = AngleRateLimit(speed_bp=[0., 5., 15.], angle_v=[10., 7.0, 0.8])
  JERK_LIMIT_MAX = 8
  JERK_LIMIT_MIN = -8

  def __init__(self, CP):
    pass


DBC = CAR.create_dbc_map()
