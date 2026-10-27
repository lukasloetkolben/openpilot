#!/usr/bin/env python3
"""Replay a route through LaneGuard and estimate how far the car would stay from the lane lines.

The simulated car starts on the logged car and is moved only by the extra curvature the guard
(and optionally the model pulling back) adds, after a first-order steering lag. Lane lines and
the planned path are re-expressed relative to the simulated car. How the real model reacts to
being pushed is unknown, so it is bracketed: --pullback 0 means the model keeps its plan
relative to the car, --pullback T means it steers back to its own line with a T s lookahead.

  lane_guard_sim.py 077b771458fd542f/0000002f--69e7e502cb --cache /tmp/route.npy
"""
import argparse
import os
from multiprocessing import Pool
from types import SimpleNamespace

import numpy as np

from openpilot.selfdrive.controls.lib.lane_guard import CAR_HALF_WIDTH, LaneGuard, MIN_LINE_PROB
from openpilot.selfdrive.controls.lib.ldw import CAMERA_OFFSET
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.tools.lib.logreader import LogReader
from openpilot.tools.lib.route import Route

N = ModelConstants.IDX_N
STEER_TAU = 0.25  # s
MIN_SPEED = 8.0  # m/s, only score frames the guard can act in


def read_segment(path):
  rows = []
  v, pressed, lat = 0.0, False, False
  for m in LogReader(path, sort_by_time=True):
    w = m.which()
    if w == 'carState':
      v, pressed = m.carState.vEgo, m.carState.steeringPressed
    elif w == 'carControl':
      lat = m.carControl.latActive
    elif w == 'modelV2':
      md = m.modelV2
      if len(md.position.x) != N or len(md.laneLines) != 4 or len(md.laneLineStds) != 4:
        continue
      ll = md.laneLines
      rows.append(np.concatenate([[m.logMonoTime * 1e-9, v, pressed, lat, md.meta.laneChangeState.raw],
                                  md.laneLineProbs, md.laneLineStds, md.position.x, md.position.y,
                                  ll[1].x, ll[1].y, ll[2].x, ll[2].y]))
  return np.array(rows).reshape(-1, 13 + 6 * N)


def load(route, cache):
  if cache and os.path.exists(cache):
    return np.load(cache)
  with Pool(8) as p:
    data = np.concatenate(p.map(read_segment, Route(route).log_paths()))
  if cache:
    np.save(cache, data)
  return data


def simulate(data, pullback):
  t, v, pressed, lat, lane_change = (data[:, i] for i in range(5))
  probs, stds = data[:, 5:9], data[:, 9:13]
  path_x, path_y, left_x, left_y, right_x, right_y = (data[:, 13 + i * N:13 + (i + 1) * N] for i in range(6))

  guard = LaneGuard(dt=0.05)
  dy = dpsi = k_lag = 0.0
  gaps, guard_k = [], []
  for i in range(len(t)):
    if not lat[i] or pressed[i]:
      guard = LaneGuard(dt=0.05)
      dy = dpsi = k_lag = 0.0
      continue
    dt = min(t[i] - t[i - 1], 0.1) if i else 0.05

    shift_l = dy + dpsi * left_x[i]
    shift_r = dy + dpsi * right_x[i]
    plan_y = path_y[i].copy()
    k_model = 0.0
    if pullback > 0:
      lookahead = max(v[i] * pullback, 10.0)
      plan_y -= (dy + dpsi * path_x[i]) * np.minimum(path_x[i] / lookahead, 1.0)
      k_model = -2 * (dy + dpsi * lookahead) / lookahead ** 2
    model = SimpleNamespace(
      meta=SimpleNamespace(laneChangeState=int(lane_change[i])),
      position=SimpleNamespace(x=path_x[i], y=plan_y),
      laneLines=[None, SimpleNamespace(x=left_x[i], y=left_y[i] - shift_l), SimpleNamespace(x=right_x[i], y=right_y[i] - shift_r), None],
      laneLineProbs=probs[i], laneLineStds=stds[i])
    k_guard = guard.update(v[i], model, True)

    if v[i] > MIN_SPEED and probs[i, 1] > MIN_LINE_PROB and probs[i, 2] > MIN_LINE_PROB and lane_change[i] == 0:
      left, right = left_y[i, 0] - dy, right_y[i, 0] - dy
      car_y = -CAMERA_OFFSET
      gaps.append(((car_y - CAR_HALF_WIDTH) - left, right - (car_y + CAR_HALF_WIDTH), right - left))
      guard_k.append((k_guard, v[i]))

    k_lag += (k_guard + k_model - k_lag) * dt / STEER_TAU
    dpsi += v[i] * k_lag * dt
    dy += v[i] * dpsi * dt
  return np.array(gaps), np.array(guard_k)


def report(name, gaps, guard_k=None):
  ok = (gaps[:, 2] > 2.0) & (gaps[:, 2] < 4.5)
  g = np.minimum(gaps[ok, 0], gaps[ok, 1])
  over = g < 0
  events = np.count_nonzero(np.diff(over.astype(int)) == 1)
  wide = gaps[ok, 2] > 2.4
  p1, p5, p50 = np.percentile(g, [1, 5, 50])
  line = f"{name:22s} body over line {over.mean() * 100:5.1f}% (lanes >2.4 m {over[wide].mean() * 100:4.1f}%, {events:3d} events)"
  line += f"  gap<0.10 m {(g < 0.10).mean() * 100:5.1f}%  gap p1 {p1:+.2f} p5 {p5:+.2f} p50 {p50:+.2f} m"
  if guard_k is not None:
    k, v = guard_k[ok, 0], guard_k[ok, 1]
    lat_accel = np.abs(k) * v ** 2
    line += f"  guard active {(np.abs(k) > 1e-5).mean() * 100:4.1f}%  lat accel p99 {np.percentile(lat_accel, 99):.2f} m/s^2"
  print(line)


if __name__ == "__main__":
  parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  parser.add_argument("route")
  parser.add_argument("--cache", help="npy file to store the extracted model data in")
  parser.add_argument("--pullback", type=float, nargs="*", default=[0.0, 2.5, 1.5], help="model pullback lookahead(s) in s, 0 = none")
  args = parser.parse_args()

  data = load(args.route, args.cache)
  # baseline: the logged car, with the guard never touching it
  lat = (data[:, 3] > 0.5) & (data[:, 2] < 0.5)
  probs = data[:, 5:9]
  keep = lat & (data[:, 1] > MIN_SPEED) & (probs[:, 1] > MIN_LINE_PROB) & (probs[:, 2] > MIN_LINE_PROB) & (data[:, 4] == 0)
  left0, right0 = data[keep, 13 + 3 * N], data[keep, 13 + 5 * N]
  car_y = -CAMERA_OFFSET
  report("logged (no guard)", np.stack([(car_y - CAR_HALF_WIDTH) - left0, right0 - (car_y + CAR_HALF_WIDTH), right0 - left0], axis=1))
  for pullback in args.pullback:
    gaps, guard_k = simulate(data, pullback)
    report(f"guard, pullback {pullback:.1f}s" if pullback else "guard, model ignores", gaps, guard_k)
