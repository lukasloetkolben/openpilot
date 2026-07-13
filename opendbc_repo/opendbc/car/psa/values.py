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
  # max deviation from measured curvature, 1/m, speed-dependent. Lateral accel ~ err * v^2, so a
  # fixed bound that is tight at highway speed starves the planner at city speed: at 28 km/h a
  # 0.002 bound is 0.13 m/s^2 - route 52 was clamp-limited 67% of ACTIVE time and drifted toward
  # the road edge. Wide-open at city speed (matching 0bd67c4, the best-driving commit, which
  # skipped the clamp below 9 m/s entirely), tight above. Wider bounds AT SPEED destabilize the
  # loop (routes 4f/51) - keep the high-speed end at 0.002.
  CURVATURE_ERROR_BP = [7., 11.]   # m/s
  CURVATURE_ERROR_V = [0.02, 0.002]  # 1/m
  # synthesized lane heading: the LKA ECU steers mostly on LINE_HEADING at speed, so the remaining
  # curvature error is converted into a heading preview; it decays as the error closes.
  HEADING_LOOKAHEAD = 1.5  # s, heading = curvature error * v * lookahead
  HEADING_MAX = 0.10  # rad, bound on synthesized heading, must match PSA_MAX_HEADING in safety
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
