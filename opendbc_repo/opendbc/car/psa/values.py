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
  # max deviation from measured curvature, 1/m. Must be wide enough that the planner's desired
  # curvature dominates what is sent (a tight clamp turns the sent curvature into an echo of the
  # car's own state - positive feedback, no lane-restoring force), but 0.010 let city-speed desired
  # swings peg the heading preview at its clamp and drove a ~0.3 Hz limit cycle (route 4f).
  # The panda still enforces the 0.02 abs cap and the 3.6 m/s^2 speed-scaled accel cap.
  CURVATURE_ERROR = 0.005
  # synthesized lane heading. The camera's LINE_HEADING is the lane angle ~16-21 m ahead, not at the
  # bumper: on well-tracked frames cam_heading ~ offset + curvature * 18 (route 3c corr 0.97). The
  # ECU expects heading and curvature to be consistent that way, so the virtual lane mimics it:
  # heading = learned camera baseline + curvature * preview + correction term.
  HEADING_PREVIEW_DIST = 18.  # m, heading/curvature consistency distance, from camera fits (16-21 m)
  HEADING_OFFSET_ALPHA = 0.01  # per 20Hz step (~5 s time constant) low-pass for the camera baseline
  HEADING_OFFSET_MAX = 0.05  # rad, sanity bound on the learned baseline
  HEADING_LOOKAHEAD = 1.5  # s, correction term = curvature error * v * lookahead
  HEADING_ERROR = 0.004  # 1/m, separate bound on the error feeding the heading, keeps it off the clamp
  HEADING_MAX = 0.10  # rad, bound on synthesized heading, must match PSA_MAX_HEADING in safety
  # slew limit on the sent heading, rad per 20Hz step. The real camera's heading moves smoothly
  # (median 0.0003 rad/step); unlimited, the synthesized heading swung clamp-to-clamp within ~1 s,
  # which with the ECU's ~119 deg/rad heading gain and 0.3-0.8 s lag drove a limit cycle (route 4f)
  HEADING_RATE = 0.004
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
