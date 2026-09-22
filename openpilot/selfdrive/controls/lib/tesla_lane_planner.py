"""
Lateral planning from the stock Tesla Autopilot lane model.

The Autopilot computer publishes its fused lane model on the party bus as DAS_lanes
(0x239) at 10Hz. It is a cubic describing the *lane center* ahead of the car:

  y(x) = c0 + c1*x + c2*x^2 + c3*x^3      for 0 <= x <= viewRange

x is forward and y is positive to the right, the same frame modelV2 uses, so no sign
flip is needed. c3 is always transmitted as exactly zero on the cars seen so far.

We turn that into a desired curvature with pure pursuit rather than a MPC: the
polynomial only reaches ~20-45m and its coefficients are 8 bit quantized, so there is
not enough information in it to justify anything more elaborate. Pure pursuit also
degrades gracefully, since at long lookahead the curvature it produces tends to the
2*c2 path curvature, which is the best validated part of the message.

Whenever Tesla's lane model is unusable (no markings, short range, lane change, low
speed) we hand back to the comma model. The handoff is ramped, never stepped.
"""
import numpy as np

from openpilot.common.realtime import DT_MDL

# Pure pursuit lookahead. The polynomial's far field (the 2*c2 curvature term) is much
# better conditioned than its near field: replaying logged routes, agreement with what
# the car actually did rises from r=0.85 at a 1.2s lookahead to r=0.92 once we use the
# whole reported view range, and the commanded curvature also gets smoother. So reach as
# far as Tesla says it can see, and never past it.
LOOKAHEAD_TIME = 2.0       # s
MIN_LOOKAHEAD = 20.0       # m
MAX_LOOKAHEAD = 45.0       # m

MIN_VIEW_RANGE = 20.0      # m, below this the lane model is too short to steer on

# Lane based control is a highway feature. Tesla stops publishing usable lines well
# before these speeds anyway, this is just a backstop.
MIN_SPEED = 8.0            # m/s
MIN_SPEED_HYST = 2.0       # m/s, re-engage above MIN_SPEED + this

# Sanity limits on the decoded polynomial. The DBC rails are +-3.5m / +-0.2rad /
# +-0.0025m^-1; a signal sitting on its rail means the APS is not tracking anything.
MAX_C0 = 2.0               # m
MAX_C1 = 0.15              # rad
MAX_C2 = 0.0024            # 1/m
MAX_CURVATURE = 0.05       # 1/m, ~20m radius

STALE_TIME = 0.3           # s without a new DAS_lanes counter before we call it dead

# Ramp in over 1s, ramp out over 0.5s: always quicker to give control back than to take it.
BLEND_UP_RATE = DT_MDL / 1.0
BLEND_DOWN_RATE = DT_MDL / 0.5

# Low pass on the 10Hz curvature so the 8 bit quantization steps do not reach the
# steering actuator as steps.
CURVATURE_TAU = 0.15       # s


class InvalidReason:
  NONE = 'none'
  NO_DATA = 'noData'
  LINES_NOT_FUSED = 'linesNotFused'
  SHORT_RANGE = 'shortRange'
  LOW_SPEED = 'lowSpeed'
  LANE_CHANGE = 'laneChange'
  IMPLAUSIBLE = 'implausible'


LINE_USAGE = ('rejectedUnavailable', 'available', 'fused', 'blacklisted')
FUSED = 2


class TeslaLanePlan:
  def __init__(self):
    self.valid = False
    self.invalid_reason = InvalidReason.NO_DATA
    self.desired_curvature = 0.0
    self.lookahead = 0.0
    self.lateral_offset = 0.0
    self.c0 = self.c1 = self.c2 = self.c3 = 0.0
    self.view_range = 0.0
    self.lane_width = 0.0
    self.left_line_usage = 0
    self.right_line_usage = 0


class TeslaLanePlanner:
  def __init__(self):
    self.blend = 0.0
    self.curvature = 0.0
    self.curvature_initialized = False
    self.last_counter: float | None = None
    self.time_since_counter = 0.0
    self.speed_ok = False
    self.plan = TeslaLanePlan()

  def reset(self):
    self.blend = 0.0
    self.curvature = 0.0
    self.curvature_initialized = False

  @staticmethod
  def eval_poly(c0: float, c1: float, c2: float, c3: float, x: float) -> float:
    return c0 + c1 * x + c2 * x ** 2 + c3 * x ** 3

  def _lane_data_fresh(self, counter: float, dt: float) -> bool:
    """DAS_lanes carries a 4 bit counter that advances every frame; a frozen counter
    means the APS stopped publishing even though the CAN parser still holds a value."""
    if self.last_counter is None or counter != self.last_counter:
      self.last_counter = counter
      self.time_since_counter = 0.0
    else:
      self.time_since_counter += dt
    return self.time_since_counter < STALE_TIME

  def _check(self, lanes, v_ego: float, lane_change: bool, dt: float) -> str:
    if lanes is None:
      self.time_since_counter += dt
      return InvalidReason.NO_DATA

    if not self._lane_data_fresh(lanes['DAS_lanesCounter'], dt):
      return InvalidReason.NO_DATA

    # speed gate with hysteresis so we do not chatter around the threshold
    if self.speed_ok:
      self.speed_ok = v_ego > MIN_SPEED
    else:
      self.speed_ok = v_ego > MIN_SPEED + MIN_SPEED_HYST
    if not self.speed_ok:
      return InvalidReason.LOW_SPEED

    if lane_change:
      return InvalidReason.LANE_CHANGE

    # Tesla tells us directly whether it is actually using both lines. Anything other
    # than FUSED on either side means the polynomial is extrapolated or stale.
    if int(lanes['DAS_leftLineUsage']) != FUSED or int(lanes['DAS_rightLineUsage']) != FUSED:
      return InvalidReason.LINES_NOT_FUSED

    if lanes['DAS_virtualLaneViewRange'] < MIN_VIEW_RANGE:
      return InvalidReason.SHORT_RANGE

    if (abs(lanes['DAS_virtualLaneC0']) > MAX_C0 or abs(lanes['DAS_virtualLaneC1']) > MAX_C1 or
        abs(lanes['DAS_virtualLaneC2']) > MAX_C2):
      return InvalidReason.IMPLAUSIBLE

    return InvalidReason.NONE

  def update(self, lanes, v_ego: float, model_curvature: float,
             lane_change: bool = False, dt: float = DT_MDL) -> TeslaLanePlan:
    """
    lanes:            decoded DAS_lanes signal dict, or None if not being received
    v_ego:            m/s
    model_curvature:  comma model desired curvature, blended with ours during handoff
    lane_change:      True while the comma model is running a lane change
    """
    plan = TeslaLanePlan()
    reason = self._check(lanes, v_ego, lane_change, dt)
    lanes_ok = reason == InvalidReason.NONE

    if lanes is not None:
      plan.c0 = float(lanes['DAS_virtualLaneC0'])
      plan.c1 = float(lanes['DAS_virtualLaneC1'])
      plan.c2 = float(lanes['DAS_virtualLaneC2'])
      plan.c3 = float(lanes['DAS_virtualLaneC3'])
      plan.view_range = float(lanes['DAS_virtualLaneViewRange'])
      plan.lane_width = float(lanes['DAS_virtualLaneWidth'])
      plan.left_line_usage = int(lanes['DAS_leftLineUsage'])
      plan.right_line_usage = int(lanes['DAS_rightLineUsage'])

    tesla_curvature = model_curvature
    if lanes_ok:
      # pure pursuit, never reaching past the range Tesla claims the model is good to.
      # MIN_VIEW_RANGE == MIN_LOOKAHEAD, so this stays at or above MIN_LOOKAHEAD.
      lookahead = float(np.clip(v_ego * LOOKAHEAD_TIME, MIN_LOOKAHEAD, MAX_LOOKAHEAD))
      lookahead = min(lookahead, plan.view_range)
      y = self.eval_poly(plan.c0, plan.c1, plan.c2, plan.c3, lookahead)
      curvature = 2.0 * y / lookahead ** 2

      if not np.isfinite(curvature) or abs(curvature) > MAX_CURVATURE:
        reason = InvalidReason.IMPLAUSIBLE
        lanes_ok = False
      else:
        plan.lookahead = lookahead
        plan.lateral_offset = y
        tesla_curvature = curvature

    if lanes_ok:
      # seed the filter from the model on the first valid frame so the blend starts
      # from wherever the car is already steering
      if not self.curvature_initialized:
        self.curvature = model_curvature
        self.curvature_initialized = True
      alpha = dt / (CURVATURE_TAU + dt)
      self.curvature += alpha * (tesla_curvature - self.curvature)
      self.blend = min(self.blend + BLEND_UP_RATE, 1.0)
    else:
      # Hold the last valid curvature and let the blend alone walk us back to the
      # model. Filtering it toward the model as well would compound with the ramp and
      # put a step in the output on the first frame of the handoff.
      self.blend = max(self.blend - BLEND_DOWN_RATE, 0.0)
      if self.blend == 0.0:
        self.curvature_initialized = False

    plan.desired_curvature = float(self.blend * self.curvature + (1.0 - self.blend) * model_curvature)
    # `valid` means "controlsd should use desiredCurvature"; at blend 0 it already
    # equals the model curvature exactly, so the handoff in either direction is seamless.
    plan.valid = self.blend > 0.0
    plan.invalid_reason = reason
    self.plan = plan
    return plan
