from dataclasses import dataclass

import numpy as np

import openpilot.cereal.messaging as messaging
from openpilot.common.realtime import DT_CTRL

# Tesla Autopilot's own lane: DAS_lanes (0x239) from the Autopilot computer on the camera side bus. It sends the center of
# its virtual lane as a cubic, y = c0 + c1 x + c2 x^2 + c3 x^3 in the same frame as modelV2 (x forward, y right), and the
# lane width. It reports a lane also where openpilot sees no line, e.g. the middle of a wide road.

DAS_LANES = 0x239
DAS_BUS = 2
MAX_AGE = 0.5  # s, sent at 10 Hz
MIN_VIEW_RANGE = 10.0  # m


@dataclass(frozen=True)
class TeslaLane:
  width: float  # m, 2.0 + 0.3125 m steps
  view_range: float  # m
  coeffs: tuple[float, float, float, float]  # c0 m, c1 rad, c2 1/m, c3 1/m^2
  left_usage: int  # 0 rejected, 1 available, 2 fused, 3 blacklisted
  right_usage: int

  def center(self, x):
    c0, c1, c2, c3 = self.coeffs
    return c0 + c1 * x + c2 * x ** 2 + c3 * x ** 3

  def edges(self, n=20):
    # (x, y) of the left and right edge of the lane up to its view range
    xs = np.linspace(0.0, self.view_range, n)
    center = self.center(xs)
    return (xs, center - self.width / 2), (xs, center + self.width / 2)


def parse_das_lanes(dat):
  if len(dat) != 8:
    return None
  return TeslaLane(width=2.0 + 0.3125 * (dat[0] >> 4), view_range=float(dat[1]),
                   coeffs=(dat[2] * 0.035 - 3.5, dat[3] * 0.0016 - 0.2, dat[4] * 2e-5 - 0.0025, dat[5] * 2.4e-7 - 3e-5),
                   left_usage=dat[6] & 3, right_usage=(dat[6] >> 2) & 3)


class TeslaLaneReader:
  def __init__(self, sock=None):
    self.sock = sock
    self.lane = None
    self.age = np.inf  # s

  def feed(self, can_events):
    # can events newest first (parsed lazily), keeps the newest DAS_lanes
    self.age += DT_CTRL
    for evt in can_events:
      for frame in evt.can:
        if frame.src == DAS_BUS and frame.address == DAS_LANES:
          lane = parse_das_lanes(frame.dat)
          if lane is not None:
            self.lane, self.age = lane, 0.0
            return

  def update(self):
    if self.sock is None:
      self.sock = messaging.sub_sock('can')
    self.feed(messaging.log_from_bytes(raw) for raw in reversed(messaging.drain_sock_raw(self.sock)))

  def get(self):
    if self.lane is None or self.age > MAX_AGE or self.lane.view_range < MIN_VIEW_RANGE:
      return None
    return self.lane
