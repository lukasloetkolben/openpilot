#!/usr/bin/env python3
"""
Georeference the model's plan and the ensemble's fused plan, and fetch a satellite backdrop.

  ./map_paths.py <route> --out <dir>

Writes paths.json (both paths per tick, in lat/lon), satellite.png (stitched Esri World Imagery)
and meta.json (the pixel to lat/lon transform) so the two can be overlaid.

How the frames are joined: the 20 Hz geometry comes from integrating the model pose, which is
smooth and internally consistent, and the absolute placement comes from the 1 Hz GPS track via a
rigid least squares fit over the whole window. Relative differences between the two paths are
therefore exact; their absolute position against the lane markings is only as good as the GPS and
the imagery registration, a few metres either way.
"""

import argparse
import json
import math
import os
import urllib.request
import numpy as np

from openpilot.selfdrive.modeld.constants import ModelConstants, Plan
from openpilot.selfdrive.modeld.temporal_ensemble import EnsembleConfig, TemporalEnsemble, path_from_yaw
from openpilot.tools.lib.logreader import LogReader

TILE_URL = "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}"
TILE_PX = 256


def collect(route: str) -> tuple[list[dict], np.ndarray]:
  models: dict[int, dict] = {}
  cam_odo: dict[int, np.ndarray] = {}
  gps: list[tuple[float, float, float]] = []

  for msg in LogReader(route):
    w = msg.which()
    if w == 'modelV2':
      m = msg.modelV2
      if len(m.position.x) != ModelConstants.IDX_N:
        continue
      models[m.frameId] = {'t': m.timestampEof / 1e9, 'log_t': msg.logMonoTime / 1e9,
                           'px': np.array(m.position.x), 'py': np.array(m.position.y),
                           'yaw': np.array(m.orientation.z), 'yaw_rate': np.array(m.orientationRate.z),
                           'py_std': np.array(m.position.yStd),
                           'intent': (str(m.meta.laneChangeState), str(m.meta.laneChangeDirection))}
    elif w == 'cameraOdometry':
      cam_odo[msg.cameraOdometry.frameId] = np.array(list(msg.cameraOdometry.trans) + list(msg.cameraOdometry.rot))
    elif w == 'gpsLocation':
      g = msg.gpsLocation
      if g.hasFix:
        gps.append((msg.logMonoTime / 1e9, g.latitude, g.longitude))
    elif w == 'carState':
      models.setdefault('_cs', []).append((msg.logMonoTime / 1e9, msg.carState.vEgo))  # type: ignore[union-attr]

  cs = np.array(models.pop('_cs', [(0., 0.)]))
  snaps = []
  for fid in sorted(k for k in models if isinstance(k, int)):
    if fid not in cam_odo:
      continue
    s = models[fid]
    s['pose'] = cam_odo[fid]
    s['v_ego'] = float(np.interp(s['log_t'], cs[:, 0], cs[:, 1]))
    snaps.append(s)
  return snaps, np.array(gps)


def run_ensemble(snaps: list[dict], cfg: EnsembleConfig) -> list[dict]:
  te = TemporalEnsemble(cfg)
  plan = np.zeros((ModelConstants.IDX_N, ModelConstants.PLAN_WIDTH))
  out, prev_intent = [], None
  for s in snaps:
    if prev_intent is not None and s['intent'] != prev_intent:
      te.reset()
    prev_intent = s['intent']
    plan[:, Plan.POSITION.start + 0] = s['px']
    plan[:, Plan.POSITION.start + 1] = s['py']
    plan[:, Plan.T_FROM_CURRENT_EULER.start + 2] = s['yaw']
    plan[:, Plan.ORIENTATION_RATE.start + 2] = s['yaw_rate']
    arc = np.concatenate(([0.], np.cumsum(np.hypot(np.diff(s['px']), np.diff(s['py'])))))
    sigma = s['py_std'] / np.maximum(arc, 1.0)
    te.update(s['t'], plan, None, s['pose'], s['v_ego'], 0.275, yaw_std=sigma)
    # Both paths must be reconstructed the same way or the comparison is meaningless: the model's
    # position head and its orientation head are only approximately consistent with each other,
    # and that disagreement is far larger than anything the fusion does.
    ox, oy = path_from_yaw(te.s_grid, s['yaw'])
    fx, fy = path_from_yaw(te.s_grid, te.fused_yaw)
    ex, ey, epsi = te.ego_pose
    out.append({'t': s['t'], 'log_t': s['log_t'], 'v': s['v_ego'], 'gated': te.gated,
                'n': te.n_members, 'ego': (ex, ey, epsi), 'curv': float(abs(s['yaw'][8] / max(arc[8], 1e-3))),
                'orig': (ox, oy), 'fused': (fx, fy), 'model': (s['px'].copy(), s['py'].copy())})
  return out


def to_local(lat: np.ndarray, lon: np.ndarray, lat0: float, lon0: float) -> np.ndarray:
  """Equirectangular metres east/north about a reference. Fine over a few km."""
  k = math.cos(math.radians(lat0))
  return np.stack([(lon - lon0) * k * 111320.0, (lat - lat0) * 110540.0], axis=1)


def rigid_fit(src: np.ndarray, dst: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
  """Least squares rotation + translation taking src onto dst. No scale: odometry sets the scale."""
  cs, cd = src.mean(0), dst.mean(0)
  H = (src - cs).T @ (dst - cd)
  U, _, Vt = np.linalg.svd(H)
  R = Vt.T @ U.T
  if np.linalg.det(R) < 0:
    Vt[-1] *= -1
    R = Vt.T @ U.T
  return R, cd - R @ cs


def deg2tile(lat: float, lon: float, z: int) -> tuple[float, float]:
  n = 2.0 ** z
  x = (lon + 180.0) / 360.0 * n
  y = (1.0 - math.asinh(math.tan(math.radians(lat))) / math.pi) / 2.0 * n
  return x, y


def tile2deg(x: float, y: float, z: int) -> tuple[float, float]:
  n = 2.0 ** z
  lon = x / n * 360.0 - 180.0
  lat = math.degrees(math.atan(math.sinh(math.pi * (1 - 2 * y / n))))
  return lat, lon


def fetch_satellite(lat_min, lat_max, lon_min, lon_max, zoom, out_png) -> dict:
  from PIL import Image
  x0f, y0f = deg2tile(lat_max, lon_min, zoom)
  x1f, y1f = deg2tile(lat_min, lon_max, zoom)
  x0, y0, x1, y1 = int(x0f), int(y0f), int(x1f), int(y1f)
  nx, ny = x1 - x0 + 1, y1 - y0 + 1
  if nx * ny > 400:
    raise SystemExit(f"{nx}x{ny} tiles is too many, lower the zoom")
  img = Image.new('RGB', (nx * TILE_PX, ny * TILE_PX))
  for i, tx in enumerate(range(x0, x1 + 1)):
    for j, ty in enumerate(range(y0, y1 + 1)):
      url = TILE_URL.format(z=zoom, x=tx, y=ty)
      req = urllib.request.Request(url, headers={'User-Agent': 'openpilot-temporal-ensemble/1.0'})
      with urllib.request.urlopen(req, timeout=30) as r:
        from io import BytesIO
        img.paste(Image.open(BytesIO(r.read())), (i * TILE_PX, j * TILE_PX))
  img.save(out_png, quality=88)
  nw_lat, nw_lon = tile2deg(x0, y0, zoom)
  se_lat, se_lon = tile2deg(x1 + 1, y1 + 1, zoom)
  return {'zoom': zoom, 'width': nx * TILE_PX, 'height': ny * TILE_PX,
          'nw': [nw_lat, nw_lon], 'se': [se_lat, se_lon],
          'tile_x0': x0, 'tile_y0': y0, 'n_tiles': [nx, ny]}


def main():
  p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  p.add_argument('route')
  p.add_argument('--out', required=True)
  p.add_argument('--zoom', type=int, default=18)
  p.add_argument('--stride', type=int, default=5, help='keep every Nth model tick')
  p.add_argument('--horizon', type=float, default=90.0, help='metres of plan to draw')
  p.add_argument('--seconds', type=float, default=30.0, help='length of the window to draw')
  p.add_argument('--fit-window', type=float, default=6.0, help='seconds of gps used per local fit')
  args = p.parse_args()
  os.makedirs(args.out, exist_ok=True)

  snaps, gps = collect(args.route)
  print(f"{len(snaps)} model ticks, {len(gps)} gps fixes")
  if len(gps) < 10:
    raise SystemExit("not enough gps")
  ticks = run_ensemble(snaps, EnsembleConfig())

  lat0, lon0 = float(np.mean(gps[:, 1])), float(np.mean(gps[:, 2]))
  gps_xy = to_local(gps[:, 1], gps[:, 2], lat0, lon0)
  ego = np.array([t['ego'][:2] for t in ticks])
  log_t = np.array([t['log_t'] for t in ticks])

  # pick the curviest window of the requested length, a whole route is too much to draw
  curv = np.array([t['curv'] for t in ticks])
  n_win = int(args.seconds * ModelConstants.MODEL_RUN_FREQ)
  if n_win < len(ticks):
    k = np.convolve(curv, np.ones(n_win) / n_win, mode='valid')
    i0 = int(np.argmax(k))
  else:
    i0, n_win = 0, len(ticks)
  # NB: ego/log_t stay full length. Slicing them would make np.interp clamp every gps fix
  # outside the drawing window onto the window edge, which silently ruins the local fits.
  ticks = ticks[i0:i0 + n_win]
  print(f"window: ticks {i0}..{i0 + n_win}, {n_win / ModelConstants.MODEL_RUN_FREQ:.0f} s, "
        + f"mean |curvature| {curv[i0:i0 + n_win].mean():.5f} 1/m")

  # Dead reckoning drifts, so one global rigid transform does not fit a multi km track. Fit a
  # local one per tick over a short window of gps fixes instead: the plan is under 100 m long,
  # local consistency is all that is needed and it keeps the two paths in the same frame.
  def local_fit(t_ref: float):
    sel = np.abs(gps[:, 0] - t_ref) <= args.fit_window
    if sel.sum() < 4:
      sel = np.argsort(np.abs(gps[:, 0] - t_ref))[:6]
    src = np.stack([np.interp(gps[sel, 0], log_t, ego[:, 0]),
                    np.interp(gps[sel, 0], log_t, ego[:, 1])], axis=1)
    R, tr = rigid_fit(src, gps_xy[sel])
    return R, tr, float(np.sqrt(np.mean(np.sum((src @ R.T + tr - gps_xy[sel]) ** 2, axis=1))))

  def world_to_ll(pts: np.ndarray, R, tr) -> np.ndarray:
    m = pts @ R.T + tr
    k = math.cos(math.radians(lat0))
    return np.stack([lat0 + m[:, 1] / 110540.0, lon0 + m[:, 0] / (k * 111320.0)], axis=1)

  out_ticks, maxdiff, fit_rms = [], [], []
  for i, tk in enumerate(ticks):
    if i % args.stride:
      continue
    R, tr, rms_i = local_fit(tk['log_t'])
    fit_rms.append(rms_i)
    ex, ey, epsi = tk['ego']
    c, s_ = math.cos(epsi), math.sin(epsi)
    res = {'t': round(tk['t'], 3), 'v': round(tk['v'], 2), 'gated': tk['gated'], 'n': tk['n']}
    for key in ('orig', 'fused', 'model'):
      px, py = tk[key]
      arc = np.concatenate(([0.], np.cumsum(np.hypot(np.diff(px), np.diff(py)))))
      sel = arc <= args.horizon
      wx = ex + c * px[sel] - s_ * py[sel]
      wy = ey + s_ * px[sel] + c * py[sel]
      res[key] = [[round(a, 7), round(b, 7)] for a, b in world_to_ll(np.stack([wx, wy], axis=1), R, tr)]
    ox, oy = tk['orig']
    fx, fy = tk['fused']
    maxdiff.append(float(np.max(np.hypot(ox - fx, oy - fy))))
    out_ticks.append(res)
  rms = float(np.mean(fit_rms))
  print(f"local odometry to gps fit: mean rms {rms:.2f} m")

  lats = [p[0] for t in out_ticks for p in t['orig']]
  lons = [p[1] for t in out_ticks for p in t['orig']]
  pad = 0.0004
  meta = fetch_satellite(min(lats) - pad, max(lats) + pad, min(lons) - pad, max(lons) + pad,
                         args.zoom, os.path.join(args.out, 'satellite.jpg'))
  meta['fit_rms_m'] = round(rms, 2)
  meta['route'] = args.route
  meta['path_diff_m'] = {'p50': round(float(np.percentile(maxdiff, 50)), 3),
                         'p90': round(float(np.percentile(maxdiff, 90)), 3),
                         'p99': round(float(np.percentile(maxdiff, 99)), 3),
                         'max': round(float(np.max(maxdiff)), 3)}
  with open(os.path.join(args.out, 'paths.json'), 'w') as f:
    json.dump({'meta': meta, 'ticks': out_ticks}, f)
  print(f"wrote {len(out_ticks)} ticks, satellite {meta['width']}x{meta['height']} px")
  print(f"max lateral difference between the two paths over the drawn horizon: {meta['path_diff_m']}")


if __name__ == '__main__':
  main()
