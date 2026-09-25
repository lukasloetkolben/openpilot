"""Shared colouring for the onroad path while Tesla's own lane model is steering.

There are two independent path renderers - the big UI (tici/tizi) and mici - each with
its own _draw_path. Both must tint identically, so the colours and the blend lookup live
here rather than being duplicated and drifting apart.
"""
import numpy as np
import pyray as rl

# Deliberately more opaque than the stock path colours: at stock alpha the tint is too
# washed out over asphalt to read as "blue" at a glance, which is the whole point of it.
TESLA_LANE_COLOR = rl.Color(0, 122, 255, 255)

# 3-stop variant, for the non-experimental gradient
TESLA_LANE_COLORS = [
  rl.Color(0, 122, 255, 200),
  rl.Color(40, 150, 255, 170),
  rl.Color(80, 180, 255, 50),
]


def lane_blend(sm) -> float:
  """How much of the commanded curvature is coming from Tesla's lane model, 0..1.

  0 means the comma model is steering and the path keeps its stock colour.
  """
  if 'teslaLanePlan' not in sm.data or not sm.valid['teslaLanePlan']:
    return 0.0
  plan = sm['teslaLanePlan']
  if not plan.valid:
    return 0.0
  return float(np.clip(plan.blend, 0.0, 1.0))


def _lift_alpha(a: int) -> int:
  """Lift a stock path alpha so the tint stays visible, keeping the fade toward the
  horizon rather than flattening it."""
  return min(255, int(a * 1.8) + 45)


def blend_colors(begin, end, t: float):
  if t >= 1.0:
    return end
  if t <= 0.0:
    return begin
  inv = 1.0 - t
  return [rl.Color(
    int(inv * s.r + t * e.r),
    int(inv * s.g + t * e.g),
    int(inv * s.b + t * e.b),
    int(inv * s.a + t * e.a),
  ) for s, e in zip(begin, end, strict=True)]


def tint_gradient(colors, blend: float):
  """Tint an N-stop gradient (the experimental acceleration colouring) toward the Tesla
  colour, keeping each stop's own alpha shape but lifting it so it stays readable."""
  if blend <= 0.0:
    return colors
  tinted = [rl.Color(TESLA_LANE_COLOR.r, TESLA_LANE_COLOR.g, TESLA_LANE_COLOR.b,
                     _lift_alpha(c.a)) for c in colors]
  return blend_colors(colors, tinted, blend)


def tint_stops(colors, blend: float):
  """Tint the 3-stop throttle/no-throttle gradient toward the Tesla colours."""
  if blend <= 0.0:
    return colors
  return blend_colors(colors, TESLA_LANE_COLORS, blend)
