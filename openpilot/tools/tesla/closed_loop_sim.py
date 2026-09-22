#!/usr/bin/env python3
"""Closed loop stability check for TeslaLanePlanner, driven from a logged route.

Replaying a log open loop (see replay_lane_plan.py) only scores the curvature we would
have asked for while somebody else was steering. It cannot answer whether the car would
weave, because under our control the car goes somewhere else and the lane moves in its
frame.

We can still answer that offline, because DAS_lanes is explicit road geometry and the
road does not care what the car does. At every frame the log tells us where the lane is
relative to the real car. If we simulate a car that has drifted (dy, dpsi) away from the
real one, the same lane in the simulated car's frame is just

    y_sim(x) = y_real(x) - dy - dpsi*x          (small angle)

which for our polynomial means c0 -= dy, c1 -= dpsi, c2 unchanged. So we can feed the
planner what it *would* have seen, and integrate the error dynamics

    dk    = lag_tau(k_cmd - k_real)        deviation in actual curvature
    d(dpsi)/dt = v * dk
    d(dy)/dt   = v * dpsi

The lag is applied to the *deviation* in commanded curvature, not to the absolute
curvature: k_real is the curvature the real car actually achieved, so it already carries
its own actuator lag, and lagging it again would double count. The self check below
pins this down - a controller that commands exactly k_real must not drift at all.

This is a perturbation test: kick the car off line, see whether the controller brings it
back, and whether it rings on the way. Only valid while the deviation stays small enough
that the real APS would still have reported the same lane, so keep the kicks modest.

Note this is only possible because Tesla hands us explicit geometry. The comma model
cannot be tested this way from a log, since its output depends on camera images we
cannot re-render from a perturbed pose.

  ./closed_loop_sim.py "077b771458fd542f/000000aa--95dd232092/4:8/r"
"""
import argparse

import numpy as np

from opendbc.can import CANParser
from opendbc.car.tesla.values import CANBUS

from openpilot.common.realtime import DT_CTRL, DT_MDL
from openpilot.selfdrive.controls.lib.drive_helpers import clip_curvature
from openpilot.selfdrive.controls.lib.tesla_lane_planner import InvalidReason, TeslaLanePlanner
from openpilot.tools.lib.logreader import LogReader

# Steering actuator + tyre response, collapsed into one lag. Measured lead of the plan
# over what the car did was 0.15-0.20s on this route.
DEFAULT_TAU = 0.2

MIN_RUN = 8.0        # s, shortest usable stretch of valid lane data
DIVERGED = 2.0       # m, past this the perturbation is no longer small and we stop


def load(route: str, bus: int):
  cp = CANParser("tesla_model3_party", [("DAS_lanes", 10)], bus)
  rows = []
  v_ego = curv = model_curv = 0.0
  for msg in LogReader(route):
    w = msg.which()
    if w == 'can':
      cp.update([(msg.logMonoTime, [(c.address, bytes(c.dat), c.src) for c in msg.can])])
    elif w == 'carState':
      v_ego = msg.carState.vEgo
    elif w == 'controlsState':
      curv = msg.controlsState.curvature
    elif w == 'modelV2':
      model_curv = float(msg.modelV2.action.desiredCurvature)
      if not cp.can_valid:
        rows.append((msg.logMonoTime / 1e9, 0, 0, 0, 0, 0, 0, v_ego, curv, model_curv))
        continue
      v = cp.vl["DAS_lanes"]
      ok = (int(v['DAS_leftLineUsage']) == 2 and int(v['DAS_rightLineUsage']) == 2
            and v['DAS_virtualLaneViewRange'] >= 20)
      rows.append((msg.logMonoTime / 1e9, ok, v['DAS_virtualLaneC0'], v['DAS_virtualLaneC1'],
                   v['DAS_virtualLaneC2'], v['DAS_virtualLaneViewRange'], v['DAS_virtualLaneWidth'],
                   v_ego, curv, model_curv))
  return np.array(rows, dtype=float)


def runs_of_valid(a, min_len):
  ok = a[:, 1] > 0
  out, start = [], None
  for i in range(len(ok)):
    if ok[i] and start is None:
      start = i
    elif not ok[i] and start is not None:
      if a[i - 1, 0] - a[start, 0] >= min_len:
        out.append((start, i))
      start = None
  if start is not None and a[-1, 0] - a[start, 0] >= min_len:
    out.append((start, len(ok)))
  return out


CTRL_SUBSTEPS = int(DT_MDL / DT_CTRL)   # controlsd limits curvature at 100Hz


def simulate(a, lo, hi, dy0: float, tau: float, perfect: bool = False):
  """Kick the car dy0 metres off the real line and let the planner steer it back.

  perfect=True replaces the planner with one that commands exactly what the real car
  did; the resulting deviation must stay at zero, which is what validates the rest.
  """
  planner = TeslaLanePlanner()
  dy, dpsi, dk = dy0, 0.0, 0.0
  prev_cmd = a[lo, 8]
  lane_err, drift, handed_off = [], [], False

  for i in range(lo, hi):
    _, _, c0, c1, c2, vr, width, v_ego, k_real, model_curv = a[i]

    if perfect:
      target = k_real
    else:
      lanes = {
        'DAS_virtualLaneC0': c0 - dy,      # the lane as seen from the drifted pose
        'DAS_virtualLaneC1': c1 - dpsi,
        'DAS_virtualLaneC2': c2,
        'DAS_virtualLaneC3': 0.0,
        'DAS_virtualLaneViewRange': vr,
        'DAS_virtualLaneWidth': width,
        'DAS_leftLineUsage': 2,
        'DAS_rightLineUsage': 2,
        'DAS_lanesCounter': i % 16,
      }
      plan = planner.update(lanes, v_ego, model_curv, False, DT_MDL)
      if plan.invalid_reason != InvalidReason.NONE:
        handed_off = True
      target = plan.desired_curvature

    k_cmd = prev_cmd
    for _ in range(CTRL_SUBSTEPS):
      k_cmd, _ = clip_curvature(v_ego, k_cmd, target, 0.0)
    prev_cmd = k_cmd

    # deviation dynamics relative to the path the real car took
    dk += (DT_MDL / (tau + DT_MDL)) * ((k_cmd - k_real) - dk)
    dpsi += v_ego * dk * DT_MDL
    dy += v_ego * dpsi * DT_MDL
    # what we care about is distance from the lane centre, not from the path the real
    # car happened to take. c0 is where the lane centre was for the real car.
    lane_err.append(dy - c0)
    drift.append(dy)

    if abs(dy) > DIVERGED:
      break

  return np.array(lane_err), np.array(drift), handed_off


def crossing_rate(h):
  """Zero crossings of the lateral error per 10s: how much the controller hunts."""
  if len(h) < 2:
    return 0.0
  return float(np.sum(np.diff(np.sign(h)) != 0) / (len(h) * DT_MDL / 10.0))


def kick_rejection(drift, dy0):
  """Seconds for the injected offset to decay to 15% of itself, relative to the path the
  real car took. Isolates the perturbation response from the road's own geometry."""
  if not dy0 or len(drift) == 0:
    return None
  for i, d in enumerate(drift):
    if abs(d) < 0.15 * abs(dy0):
      return i * DT_MDL
  return None


def main():
  p = argparse.ArgumentParser()
  p.add_argument("route")
  p.add_argument("--bus", type=int, default=CANBUS.autopilot_party)
  p.add_argument("--tau", type=float, default=DEFAULT_TAU, help="actuator lag, s")
  args = p.parse_args()

  a = load(args.route, args.bus)
  runs = runs_of_valid(a, MIN_RUN)
  total = sum(a[hi - 1, 0] - a[lo, 0] for lo, hi in runs)
  print(f"\n{len(runs)} stretches of continuously valid lane data, {total:.0f}s total")
  print(f"actuator lag tau = {args.tau:.2f}s\n")

  worst = 0.0
  for lo, hi in runs:
    _, drift, _ = simulate(a, lo, hi, 0.0, args.tau, perfect=True)
    worst = max(worst, float(np.max(np.abs(drift))) if len(drift) else 0.0)
  refs = [a[lo:hi, 2] for lo, hi in runs]
  ref = np.abs(np.concatenate(refs))
  print(f"reference: stock Autopilot over these same stretches, rms {np.sqrt(np.mean(ref ** 2)):.2f}m, "
        + f"p99.5 {np.percentile(ref, 99.5):.2f}m, {np.mean([crossing_rate(r) for r in refs]):.1f} crossings/10s")
  verdict = 'ok' if worst < 0.05 else 'BROKEN, results below are meaningless'
  print(f"self check: a controller commanding exactly what the car did drifts {worst:.3f}m ({verdict})\n")

  print(f"{'kick':>6} {'rms err':>9} {'p99.5 err':>10} {'crossings/10s':>14} {'kick rejected':>14} {'diverged':>9}")
  for dy0 in (0.0, 0.5, -0.5, 1.0, -1.0):
    rms, p995, cross, rej, div = [], [], [], [], 0
    for lo, hi in runs:
      h, drift, _ = simulate(a, lo, hi, dy0, args.tau)
      if not len(h):
        continue
      rms.append(float(np.sqrt(np.mean(h ** 2))))
      p995.append(float(np.percentile(np.abs(h), 99.5)))
      cross.append(crossing_rate(h))
      t = kick_rejection(drift, dy0)
      if t is not None:
        rej.append(t)
      div += abs(drift[-1]) > DIVERGED * 0.99
    if not rms:
      continue
    r = f"{np.mean(rej):.1f}s" if rej else ("-" if not dy0 else "never")
    print(f"{dy0:>+6.2f} {np.mean(rms):>9.2f} {np.max(p995):>10.2f} {np.mean(cross):>14.1f} "
          + f"{r:>14} {div:>5}/{len(runs)}")

  print("\nerr = distance from lane centre; compare against the stock Autopilot reference above.")
  print("The steering chain is collapsed into one lag, so treat this as a stability screen,")
  print("not a substitute for a careful first drive.")


if __name__ == "__main__":
  main()
