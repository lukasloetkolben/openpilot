#!/usr/bin/env python3
"""
Offline evaluation of temporal ensembling on a recorded route. Changes nothing in the system.

  ./replay_ensemble.py <route>                      # metrics only
  ./replay_ensemble.py <route> --plot               # + plots
  ./replay_ensemble.py <route> --pose device        # control run against the IMU pose

The logged modelV2 does not contain orientation stds, only position stds, so --sigma selects
which uncertainty proxy drives the inverse variance weighting. Use --sigma age to check how
much of the effect survives with pure age weighting.
"""

import argparse
import numpy as np

from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.controls.lib.drive_helpers import get_curvature_from_plan
from openpilot.selfdrive.modeld.constants import ModelConstants, Plan
from openpilot.selfdrive.modeld.temporal_ensemble import EnsembleConfig, TemporalEnsemble, stds_monotonic
from openpilot.tools.lib.logreader import LogReader

T_IDXS = np.array(ModelConstants.T_IDXS)
LAT_SMOOTH_SECONDS = 0.0  # mirrors modeld, importing it would pull in the compiled messaging stack


def collect(route: str) -> list[dict]:
  """Join modelV2 with the pose sources and the car state, keyed on frameId."""
  lr = LogReader(route)

  models: dict[int, dict] = {}
  cam_odo: dict[int, np.ndarray] = {}
  device_motion: list[tuple[float, np.ndarray]] = []
  car_state: list[tuple[float, float]] = []
  lat_active: list[tuple[float, bool]] = []
  lat_delay: list[tuple[float, float]] = []

  for msg in lr:
    w = msg.which()
    if w == 'modelV2':
      m = msg.modelV2
      if len(m.position.x) != ModelConstants.IDX_N:
        continue
      models[m.frameId] = {
        'frame_id': m.frameId,
        't': m.timestampEof / 1e9,
        'log_t': msg.logMonoTime / 1e9,
        'px': np.array(m.position.x), 'py': np.array(m.position.y),
        'py_std': np.array(m.position.yStd),
        'yaw': np.array(m.orientation.z),
        'yaw_rate': np.array(m.orientationRate.z),
        'curvature': m.action.desiredCurvature,
        'intent': (str(m.meta.laneChangeState), str(m.meta.laneChangeDirection)),
        'big': bool(m.big),
      }
    elif w == 'cameraOdometry':
      c = msg.cameraOdometry
      cam_odo[c.frameId] = np.array(list(c.trans) + list(c.rot))
    elif w == 'deviceMotion':
      d = msg.deviceMotion
      device_motion.append((msg.logMonoTime / 1e9,
                            np.array([d.velocityDevice.x, d.velocityDevice.y, d.velocityDevice.z,
                                      d.angularVelocityDevice.x, d.angularVelocityDevice.y,
                                      d.angularVelocityDevice.z])))
    elif w == 'carState':
      car_state.append((msg.logMonoTime / 1e9, msg.carState.vEgo))
    elif w == 'carControl':
      lat_active.append((msg.logMonoTime / 1e9, msg.carControl.latActive))
    elif w == 'lateralDelay':
      lat_delay.append((msg.logMonoTime / 1e9, msg.lateralDelay.lateralDelay))

  cs_t = np.array([t for t, _ in car_state])
  cs_v = np.array([v for _, v in car_state])
  la_t = np.array([t for t, _ in lat_active])
  la_v = np.array([a for _, a in lat_active])
  ld_t = np.array([t for t, _ in lat_delay])
  ld_v = np.array([d for _, d in lat_delay])
  dm_t = np.array([t for t, _ in device_motion])
  dm_p = np.array([p for _, p in device_motion]) if device_motion else np.zeros((0, 6))

  out = []
  for fid in sorted(models):
    if fid not in cam_odo:
      continue
    s = models[fid]
    s['pose_camera'] = cam_odo[fid]
    # the IMU pose is on its own clock, interpolate it to the frame timestamp
    s['pose_device'] = (np.array([np.interp(s['log_t'], dm_t, dm_p[:, i]) for i in range(6)])
                        if len(dm_t) else None)
    s['v_ego'] = float(np.interp(s['log_t'], cs_t, cs_v)) if len(cs_t) else float(s['px'][1] / T_IDXS[1])
    s['lat_active'] = bool(np.interp(s['log_t'], la_t, la_v.astype(float)) > 0.5) if len(la_t) else True
    # reconstruct exactly what modeld used: lateralDelay + LAT_SMOOTH + frame_delay + action_delay
    delay = float(np.interp(s['log_t'], ld_t, ld_v)) if len(ld_t) else 0.2
    s['lat_action_t'] = delay + LAT_SMOOTH_SECONDS + DT_MDL + DT_MDL / 2
    out.append(s)
  return out


def sigma_for(snap: dict, mode: str) -> np.ndarray | None:
  if mode == 'age':
    return np.ones(ModelConstants.IDX_N)  # constant -> inverse variance term drops out
  if mode == 'posy':
    # lateral position std at arclength s implies a heading std of roughly yStd / s
    s = np.concatenate(([0.], np.cumsum(np.hypot(np.diff(snap['px']), np.diff(snap['py'])))))
    return snap['py_std'] / np.maximum(s, 1.0)
  raise ValueError(mode)


def simulate(snaps: list[dict], cfg: EnsembleConfig, pose_key: str, sigma_mode: str,
             lat_action_t: float | None) -> dict:
  te = TemporalEnsemble(cfg)
  plan = np.zeros((ModelConstants.IDX_N, ModelConstants.PLAN_WIDTH))
  deltas, gates, members, plan_curv = [], [], [], []
  prev_intent = None

  for s in snaps:
    pose = s[pose_key]
    if pose is None:
      deltas.append(0.0)
      gates.append(False)
      members.append(1)
      plan_curv.append(0.0)
      continue
    # reset on the edge of an intent change, not for as long as it lasts
    intent = (s['lat_active'], *s['intent'])
    if prev_intent is not None and intent != prev_intent:
      te.reset()
    prev_intent = intent
    plan[:, Plan.POSITION.start + 0] = s['px']
    plan[:, Plan.POSITION.start + 1] = s['py']
    plan[:, Plan.T_FROM_CURRENT_EULER.start + 2] = s['yaw']
    plan[:, Plan.ORIENTATION_RATE.start + 2] = s['yaw_rate']
    action_t = s['lat_action_t'] if lat_action_t is None else lat_action_t
    d = te.update(s['t'], plan, None, pose, s['v_ego'], action_t,
                  yaw_std=sigma_for(s, sigma_mode))
    deltas.append(d)
    gates.append(te.gated)
    members.append(te.n_members)
    # what the curvature would be if it came from the waypoints, which is the quantity the
    # ensemble actually operates on. On a model with an action head this is not what the
    # recorded desiredCurvature is, and comparing against the wrong baseline is misleading.
    plan_curv.append(get_curvature_from_plan(s['yaw'], s['yaw_rate'], ModelConstants.T_IDXS,
                                             s['v_ego'], action_t))

  base = np.array([s['curvature'] for s in snaps])
  pc = np.array(plan_curv)
  v = np.array([s['v_ego'] for s in snaps])
  delta = np.array(deltas)
  return {'base': base, 'fused': base + delta, 'plan_curv': pc, 'plan_fused': pc + delta,
          'delta': delta, 'gate': np.array(gates), 'members': np.array(members), 'v_ego': v,
          't': np.array([s['t'] for s in snaps])}


def jerk(curv: np.ndarray, v: np.ndarray, dt: float) -> np.ndarray:
  """Lateral jerk proxy: d/dt of the commanded lateral acceleration."""
  return np.diff(curv * v ** 2) / dt


def report(name: str, r: dict) -> None:
  dt = float(np.median(np.diff(r['t'])))
  curvy = np.abs(r['base']) > 0.002
  print(f"\n--- {name} ---")
  print(f"  ticks                {len(r['base'])}   dt {dt*1e3:.1f} ms")
  print(f"  gate rate            {r['gate'].mean()*100:.2f} %")
  print(f"  mean members         {r['members'].mean():.2f}")
  p50, p99 = np.percentile(np.abs(r['delta']), [50, 99])
  print(f"  |delta| p50 / p99    {p50:.2e} / {p99:.2e} 1/m")
  res = r['plan_curv'] - r['base']
  print(f"  plan curv vs recorded  rms {np.std(res):.2e}, corr {np.corrcoef(r['plan_curv'], r['base'])[0,1]:.4f}")
  for bl, fu, what in (('base', 'fused', 'vs recorded  '), ('plan_curv', 'plan_fused', 'vs plan curv ')):
    for label, mask in (("straight", ~curvy), ("curvy", curvy)):
      if mask.sum() < 10:
        continue
      jb = jerk(r[bl][mask], r['v_ego'][mask], dt)
      jf = jerk(r[fu][mask], r['v_ego'][mask], dt)
      rel = (np.std(jf) / np.std(jb) - 1) * 100
      print(f"  jerk rms {what}{label:8s} {np.std(jb):.4f} -> {np.std(jf):.4f} m/s^3  ({rel:+.1f} %)")


def std_monotonicity(snaps: list[dict]) -> None:
  rhos, monos = [], []
  for s in snaps:
    if np.ptp(s['py_std']) == 0.0:
      continue
    mono, rho = stds_monotonic(s['py_std'])
    rhos.append(rho)
    monos.append(mono)
  if not rhos:
    print("\nposition yStd: constant, unusable for weighting")
    return
  p05, p50, p95 = np.percentile(rhos, [5, 50, 95])
  print(f"\nposition yStd over distance: rho p05/p50/p95 = {p05:.3f} / {p50:.3f} / {p95:.3f}")
  print(f"  strictly non decreasing on {100 * np.mean(monos):.1f} % of ticks")


def plot(results: dict[str, dict]) -> None:
  import matplotlib.pyplot as plt
  fig, ax = plt.subplots(3, 1, sharex=True, figsize=(14, 9))
  first = next(iter(results.values()))
  t = first['t'] - first['t'][0]
  ax[0].plot(t, first['base'], lw=0.8, label='model', color='k')
  for name, r in results.items():
    ax[0].plot(t, r['fused'], lw=0.8, label=f'ensemble {name}')
    ax[1].plot(t, r['delta'], lw=0.8, label=name)
  ax[0].set_ylabel('desired curvature [1/m]')
  ax[1].set_ylabel('correction [1/m]')
  ax[2].plot(t, first['v_ego'], lw=0.8, label='v_ego')
  ax[2].plot(t, first['gate'] * first['v_ego'].max(), lw=0.8, label='gate')
  ax[2].set_ylabel('m/s')
  ax[2].set_xlabel('t [s]')
  for a in ax:
    a.legend()
    a.grid(alpha=.3)
  fig.tight_layout()
  plt.show()


def main():
  p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  p.add_argument('route')
  p.add_argument('--pose', choices=['camera', 'device', 'both'], default='camera',
                 help='pose source. "both" runs the correlated pose error control')
  p.add_argument('--sigma', choices=['posy', 'age'], default='posy')
  p.add_argument('--lat-action-t', type=float, default=None,
                 help='override the action time. Default: reconstructed from the logged lateralDelay')
  p.add_argument('--buffer-len', type=int, default=EnsembleConfig().buffer_len)
  p.add_argument('--age-tau', type=float, default=EnsembleConfig().age_tau)
  p.add_argument('--gate-psi', type=float, default=EnsembleConfig().gate_psi)
  p.add_argument('--plot', action='store_true')
  args = p.parse_args()

  snaps = collect(args.route)
  n_big = sum(s['big'] for s in snaps)
  print(f"loaded {len(snaps)} model ticks from {args.route}")
  print(f"  big model on {100 * n_big / max(len(snaps), 1):.0f} % of ticks"
        + ("  -> desiredCurvature comes from the action head, not the waypoints" if n_big else ""))
  if len(snaps) < 20:
    raise SystemExit("not enough data")
  std_monotonicity(snaps)

  cfg = EnsembleConfig(buffer_len=args.buffer_len, age_tau=args.age_tau, gate_psi=args.gate_psi)
  sources = ['camera', 'device'] if args.pose == 'both' else [args.pose]
  results = {}
  for src in sources:
    if snaps[0][f'pose_{src}'] is None:
      print(f"no {src} pose in this route, skipping")
      continue
    results[src] = simulate(snaps, cfg, f'pose_{src}', args.sigma, args.lat_action_t)
    report(f"pose={src} sigma={args.sigma}", results[src])

  # the fallback must be exactly the current behaviour
  fb = simulate(snaps, EnsembleConfig(buffer_len=0), f'pose_{sources[0]}', args.sigma, args.lat_action_t)
  assert np.array_equal(fb['fused'], fb['base']), "fallback is not exact"
  print("\nfallback check: identical to the recorded output")

  if len(results) == 2:
    d = results['camera']['delta'] - results['device']['delta']
    print(f"\ncorrelated pose error: camera vs device correction differs by rms {np.std(d):.2e}, "
          + f"max {np.max(np.abs(d)):.2e} 1/m")

  if args.plot and results:
    plot(results)


if __name__ == '__main__':
  main()
