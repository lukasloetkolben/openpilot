#!/usr/bin/env python3
"""Replay a route through TeslaLanePlanner and score it offline.

Runs the real CAN parser and the real planner over a logged route, then compares the
curvature it would have commanded against what the car actually did and against what
the comma model asked for over the same frames.

On a route driven on stock Autopilot, controlsState.curvature is Autopilot's own
steering output, so it is a reasonable reference for "what a good driver did here".

  ./replay_lane_plan.py "077b771458fd542f/000000aa--95dd232092/4:8/r"
"""
import argparse
import collections

import numpy as np

from opendbc.can import CANParser
from opendbc.car.tesla.values import CANBUS

from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.controls.lib.tesla_lane_planner import InvalidReason, TeslaLanePlanner
from openpilot.tools.lib.logreader import LogReader

DBC_NAME = "tesla_model3_party"


def replay(route: str, bus: int):
  planner = TeslaLanePlanner()
  cp = CANParser(DBC_NAME, [("DAS_lanes", 10)], bus)

  rows = []
  reasons: collections.Counter = collections.Counter()
  v_ego = curvature = model_curvature = 0.0
  lane_change = False
  last_plan_t = None

  for msg in LogReader(route):
    w = msg.which()
    t = msg.logMonoTime / 1e9

    if w == 'can':
      cp.update([(msg.logMonoTime, [(c.address, bytes(c.dat), c.src) for c in msg.can])])
    elif w == 'carState':
      v_ego = msg.carState.vEgo
    elif w == 'controlsState':
      curvature = msg.controlsState.curvature
    elif w == 'modelV2':
      model_curvature = float(msg.modelV2.action.desiredCurvature)
      lane_change = msg.modelV2.meta.laneChangeState != 'off'

      # step the planner once per model frame (20Hz), same cadence as the daemon
      dt = DT_MDL if last_plan_t is None else max(t - last_plan_t, 1e-3)
      last_plan_t = t
      lanes = cp.vl["DAS_lanes"] if cp.can_valid else None
      plan = planner.update(lanes, v_ego, model_curvature, lane_change, dt)
      reasons[plan.invalid_reason] += 1
      rows.append((t, plan.valid, planner.blend, plan.desired_curvature,
                   model_curvature, curvature, v_ego, plan.view_range, plan.lookahead))

  return np.array(rows, dtype=float), reasons


def score(name: str, cmd: np.ndarray, ref: np.ndarray, t: np.ndarray) -> None:
  """Report agreement with the reference, searching a small lag (actuation delay)."""
  best = None
  for lag in np.arange(0.0, 0.75, 0.05):
    shifted = np.interp(t + lag, t, ref)
    r = np.corrcoef(cmd, shifted)[0, 1]
    rmse = float(np.sqrt(np.mean((cmd - shifted) ** 2)))
    if best is None or r > best[1]:
      best = (lag, r, rmse)
  lag, r, rmse = best
  print(f"  {name:<28} r={r:+.3f}  rmse={rmse:.5f} 1/m  (best lag {lag:.2f}s)")


def main():
  p = argparse.ArgumentParser()
  p.add_argument("route")
  p.add_argument("--bus", type=int, default=CANBUS.autopilot_party)
  args = p.parse_args()

  rows, reasons = replay(args.route, args.bus)
  if not len(rows):
    raise SystemExit("no modelV2 frames in route")

  t, valid, blend, tesla, model, actual, v_ego, view_range, lookahead = rows.T
  n = len(t)

  print(f"\n{n} model frames ({n * DT_MDL:.0f}s)\n")
  print(f"Tesla lane plan in control : {100 * (blend > 0).mean():5.1f}% of frames ({100 * (blend == 1.0).mean():.1f}% fully)")
  print("Why not, per frame:")
  for reason, cnt in reasons.most_common():
    label = 'using tesla lanes' if reason == InvalidReason.NONE else reason
    print(f"  {label:<28} {100 * cnt / n:5.1f}%")

  eng = blend == 1.0
  if eng.sum() < 50:
    print("\nnot enough fully engaged frames to score")
    return

  print(f"\nWhile fully on Tesla lanes ({eng.sum()} frames):")
  print(f"  viewRange  {view_range[eng].min():.0f}-{view_range[eng].max():.0f} m, "
        + f"lookahead {lookahead[eng].min():.0f}-{lookahead[eng].max():.0f} m, "
        + f"speed {v_ego[eng].min():.0f}-{v_ego[eng].max():.0f} m/s")
  print("\n  agreement with what the car actually did (stock Autopilot steering):")
  score("tesla lane plan", tesla[eng], actual[eng], t[eng])
  score("comma model (baseline)", model[eng], actual[eng], t[eng])
  print("\n  tesla lane plan vs comma model:")
  score("", tesla[eng], model[eng], t[eng])

  # Smoothness: a step here reaches the steering actuator. Measure only over
  # consecutive engaged frames, otherwise this reports the comma model's own steps
  # from the frames where we are just passing it through.
  idx = np.where(eng)[0]
  contig = idx[1:][np.diff(idx) == 1]
  steps = np.abs(tesla[contig] - tesla[contig - 1])
  print(f"\nCommanded curvature step per frame while engaged: max {steps.max():.5f}, p99 {np.percentile(steps, 99):.5f} 1/m")
  handoffs = int(np.sum(np.abs(np.diff((blend > 0).astype(int))) > 0))
  print(f"Handoffs between comma model and Tesla lanes: {handoffs}")


if __name__ == "__main__":
  main()
