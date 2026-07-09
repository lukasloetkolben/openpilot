from dataclasses import dataclass, field

from opendbc.car.structs import CarParams
from opendbc.car import Bus, CarSpecs, DbcDict, PlatformConfig, Platforms
from opendbc.car.lateral import AngleSteeringLimits, CurvatureSteeringLimits
from opendbc.car.docs_definitions import CarDocs, CarHarness, CarParts
from opendbc.car.fw_query_definitions import FwQueryConfig, Request, StdQueries

Ecu = CarParams.Ecu


class CarControllerParams:
  STEER_STEP = 5  # LKAS camera lane messages at 20Hz

  ANGLE_LIMITS: AngleSteeringLimits = AngleSteeringLimits(
    390, # deg
    ([0., 5., 25.], [2.5, 1.5, .2]),
    ([0., 5., 25.], [5., 2., .3]),
  )
  CURVATURE_LIMITS: CurvatureSteeringLimits = CurvatureSteeringLimits(0.02)  # max curvature for lane injection, 1/m
  CURVATURE_ERROR = 0.002  # max deviation from measured curvature, 1/m
  # The LKA ECU steers mostly on lane heading, not curvature, above ~40 km/h, so we synthesize a
  # heading from the curvature error to give it authority at speed. heading[rad] = err * v * lookahead.
  # Both are first-cut values that need on-car tuning.
  HEADING_LOOKAHEAD = 1.5  # s, lookahead distance = v * this
  HEADING_MAX = 0.10  # rad
  STEER_DRIVER_ALLOWANCE = 5  # Driver intervention threshold, 0.5 Nm


@dataclass
class PSACarDocs(CarDocs):
  package: str = "Adaptive Cruise Control (ACC) & Lane Assist"
  car_parts: CarParts = field(default_factory=CarParts.common([CarHarness.psa_a]))


@dataclass
class PSAPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: {
    Bus.pt: 'psa_aee2010_r3',
  })


class CAR(Platforms):
  PSA_PEUGEOT_208 = PSAPlatformConfig(
    [PSACarDocs("Peugeot 208 2019-25")],
    CarSpecs(mass=1530, wheelbase=2.54, steerRatio=17.6),
  )


# Placeholder, FW Query will be added in separate PR
FW_QUERY_CONFIG = FwQueryConfig(
  requests=[
    Request(
      [StdQueries.TESTER_PRESENT_REQUEST, StdQueries.UDS_VERSION_REQUEST],
      [StdQueries.TESTER_PRESENT_RESPONSE, StdQueries.UDS_VERSION_RESPONSE],
      bus=0,
    ),
  ],
)

DBC = CAR.create_dbc_map()
