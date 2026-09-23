#!/usr/bin/env python3
"""Live view of who is steering: the comma model, or Tesla's lane model.

Run it on the device while driving (or parked, to watch the gating):

  PYTHONPATH=/data/openpilot /usr/local/venv/bin/python openpilot/tools/tesla/watch_lane_plan.py

TESLA means controlsd is using teslaLanePlan.desiredCurvature; COMMA means it fell back
to modelV2.action.desiredCurvature. HANDOFF is the ramp between them, where the commanded
curvature is a mix of the two.
"""
import time

import openpilot.cereal.messaging as messaging


def source(plan, stale: bool) -> str:
  if stale or not plan.valid:
    return "COMMA "
  if plan.blend >= 0.999:
    return "TESLA "
  return "HANDOFF"


def main():
  sm = messaging.SubMaster(['teslaLanePlan', 'modelV2', 'carState', 'carControl', 'selfdriveState'])

  hdr = f"{'source':>7} {'blend':>6} {'why not':>14} {'curv cmd':>9} {'comma':>9} "
  hdr += f"{'v':>5} {'range':>6} {'look':>5} {'width':>6} {'c0':>7} {'lat':>6}"
  print(hdr)
  last = 0.0
  while True:
    sm.update(200)
    if not sm.updated['teslaLanePlan'] and not sm.updated['modelV2']:
      # daemon not running at all -> the comma model is definitely driving
      if time.monotonic() - last > 1.0:
        last = time.monotonic()
        running = sm.recv_frame['teslaLanePlan'] > 0
        why = 'no teslaLanePlan' if not running else 'stale'
        print(f"{'COMMA ':>7} {'-':>6} {why:>14}   (teslalatplannerd not publishing)")
      continue

    if time.monotonic() - last < 0.25:
      continue
    last = time.monotonic()

    p = sm['teslaLanePlan']
    stale = not sm.alive['teslaLanePlan']
    why = '-' if p.valid else str(p.invalidReason)
    engaged = 'lat' if sm['carControl'].latActive else '---'
    line = f"{source(p, stale):>7} {p.blend:>6.2f} {why:>14} {p.desiredCurvature:>+9.5f} "
    line += f"{sm['modelV2'].action.desiredCurvature:>+9.5f} {sm['carState'].vEgo:>5.1f} "
    line += f"{p.viewRange:>6.0f} {p.lookahead:>5.0f} {p.laneWidth:>6.2f} {p.c0:>+7.2f} "
    line += f"{p.lateralOffset:>+6.2f}  {engaged}"
    print(line)


if __name__ == "__main__":
  main()
