#!/usr/bin/env python3
"""Run driving_supercombo.onnx over a logged route and check it against the recorded modelV2.

Rebuilds modeld's per-tick inputs from the log (calibration, intrinsics, traffic
convention, action_t, and desire via DesireHelper driven by our own lane-change
probability) so the only thing that differs from onroad is the runtime.

  ./run_route.py <route>                       # validate against logged modelV2
  ./run_route.py <route> --segments 0 1        # more segments
  ./run_route.py <route> --limit 300           # first N ticks only
"""
import argparse
import sys
import time

import numpy as np

from openpilot.cereal import log
from openpilot.common.transformations.camera import DEVICE_CAMERAS
from openpilot.common.transformations.model import get_warp_matrix
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.controls.lib.desire_helper import DesireHelper
from openpilot.selfdrive.modeld.constants import ModelConstants, Plan
from openpilot.tools.lib.framereader import FrameReader
from openpilot.tools.lib.logreader import LogReader
from openpilot.tools.lib.route import Route
from openpilot.tools.supercombo_onnx.runner import SupercomboRunner, DEFAULT_MODEL

LAT_SMOOTH_SECONDS = 0.0   # keep in sync with modeld.py
LONG_SMOOTH_SECONDS = 0.3


def collect_ticks(route_name: str, seg: int):
  """One entry per logged modelV2, carrying the state modeld saw when it ran."""
  state = {'rpy': None, 'device_type': None, 'sensor': None, 'is_rhd': False,
           'lat_delay': 0.0, 'car_state': None, 'lat_active': False, 'long_delay': 0.0}
  ticks = []
  for msg in LogReader(f"{route_name}/{seg}"):
    w = msg.which()
    if w == 'extrinsicsCalibration':
      state['rpy'] = np.array(msg.extrinsicsCalibration.rpyCalib, dtype=np.float32)
    elif w == 'deviceState':
      state['device_type'] = str(msg.deviceState.deviceType)
    elif w == 'narrowRoadCameraState':
      state['sensor'] = str(msg.narrowRoadCameraState.sensor)
    elif w == 'driverMonitoringState':
      state['is_rhd'] = bool(msg.driverMonitoringState.isRHD)
    elif w == 'lateralDelay':
      state['lat_delay'] = float(msg.lateralDelay.lateralDelay)
    elif w == 'carState':
      state['car_state'] = msg.carState
    elif w == 'carControl':
      state['lat_active'] = bool(msg.carControl.latActive)
    elif w == 'carParams':
      state['long_delay'] = float(msg.carParams.longitudinalActuatorDelay)
    elif w == 'modelV2':
      if state['rpy'] is None or state['car_state'] is None:
        continue
      ticks.append({
        'frame_id': msg.modelV2.frameId,
        'frame_id_extra': msg.modelV2.frameIdExtra,
        'rpy': state['rpy'].copy(),
        'device_type': state['device_type'],
        'sensor': state['sensor'],
        'is_rhd': state['is_rhd'],
        'lat_delay': state['lat_delay'],
        'long_delay': state['long_delay'],
        'car_state': state['car_state'].as_builder().as_reader(),
        'lat_active': state['lat_active'],
        'logged': msg.modelV2.as_builder().as_reader(),
      })
  return ticks


def main():
  p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  p.add_argument('route')
  p.add_argument('--segments', type=int, nargs='+', default=[0])
  p.add_argument('--limit', type=int, default=None, help='stop after N model ticks')
  p.add_argument('--model', default=str(DEFAULT_MODEL))
  p.add_argument('--providers', nargs='+', default=['CPUExecutionProvider'])
  args = p.parse_args()

  runner = SupercomboRunner(args.model, providers=args.providers)
  print(f"model  {args.model}")
  print(f"ort    {runner.sess.get_providers()}")

  route = Route(args.route)
  n_done = 0
  errs: dict[str, list[np.ndarray]] = {'position_x': [], 'position_y': [], 'lane_line_prob': []}
  t_infer = []

  for seg in args.segments:
    ticks = collect_ticks(args.route, seg)
    if not ticks:
      print(f"segment {seg}: no usable modelV2 ticks", file=sys.stderr)
      continue
    fr_main = FrameReader(route.camera_paths()[seg], pix_fmt='nv12')
    fr_extra = FrameReader(route.ecamera_paths()[seg], pix_fmt='nv12')
    cam_w, cam_h = fr_main.w, fr_main.h

    dc = DEVICE_CAMERAS[(ticks[0]['device_type'], ticks[0]['sensor'])]
    base_fid, base_fid_extra = ticks[0]['frame_id'], ticks[0]['frame_id_extra']

    DH = DesireHelper()
    runner.reset()
    cam = f"{ticks[0]['device_type']}/{ticks[0]['sensor']}"
    print(f"\nsegment {seg}: {len(ticks)} ticks, {cam_w}x{cam_h}, {cam}")

    for tick in ticks:
      idx_main = tick['frame_id'] - base_fid
      idx_extra = tick['frame_id_extra'] - base_fid_extra
      if not (0 <= idx_main < fr_main.frame_count and 0 <= idx_extra < fr_extra.frame_count):
        continue

      tfm_main = get_warp_matrix(tick['rpy'], dc.narrow_road.intrinsics, False).astype(np.float32)
      tfm_extra = get_warp_matrix(tick['rpy'], dc.wide_road.intrinsics, True).astype(np.float32)

      traffic_convention = np.zeros(2, dtype=np.float32)
      traffic_convention[int(tick['is_rhd'])] = 1

      vec_desire = np.zeros(ModelConstants.DESIRE_LEN, dtype=np.float32)
      if 0 <= DH.desire < ModelConstants.DESIRE_LEN:
        vec_desire[DH.desire] = 1

      frame_delay = DT_MDL
      action_delay = DT_MDL / 2
      lat_action_t = tick['lat_delay'] + LAT_SMOOTH_SECONDS + frame_delay + action_delay
      long_action_t = tick['long_delay'] + LONG_SMOOTH_SECONDS + frame_delay + action_delay

      t0 = time.perf_counter()
      out = runner.run(fr_main.get(idx_main), fr_extra.get(idx_extra), tfm_main, tfm_extra,
                       cam_w, cam_h, desire=vec_desire, traffic_convention=traffic_convention,
                       action_t=np.array([lat_action_t, long_action_t], dtype=np.float32))
      t_infer.append(time.perf_counter() - t0)

      desire_state = out['desire_state'][0].reshape(-1)
      lane_change_prob = float(desire_state[log.Desire.laneChangeLeft] + desire_state[log.Desire.laneChangeRight])
      DH.update(tick['car_state'], tick['lat_active'], lane_change_prob)

      if runner.warm:
        logged = tick['logged']
        pos = out['plan'][0, :, Plan.POSITION]
        errs['position_x'].append(np.abs(np.array(logged.position.x) - pos[:, 0]))
        errs['position_y'].append(np.abs(np.array(logged.position.y) - pos[:, 1]))
        errs['lane_line_prob'].append(np.abs(np.array(logged.laneLineProbs) - out['lane_lines_prob'][0, 1::2]))

      n_done += 1
      if n_done % 50 == 0:
        print(f"  {n_done} ticks", end='\r', flush=True)
      if args.limit is not None and n_done >= args.limit:
        break
    if args.limit is not None and n_done >= args.limit:
      break

  print(f"\n\nran {n_done} ticks, {len(errs['position_x'])} compared after warmup")
  if t_infer:
    print(f"inference {np.mean(t_infer) * 1e3:.1f} ms mean, {np.percentile(t_infer, 95) * 1e3:.1f} ms p95")
  for k, v in errs.items():
    if not v:
      continue
    a = np.stack(v)
    print(f"{k:16s} mean abs diff vs logged {a.mean():.4f}  max {a.max():.4f}")
  if errs['position_x']:
    ax, ay = np.stack(errs['position_x']).mean(0), np.stack(errs['position_y']).mean(0)
    print("\nplan position error by index (mean abs, m):")
    for i in range(0, ModelConstants.IDX_N, 4):
      t, x = ModelConstants.T_IDXS[i], ModelConstants.X_IDXS[i]
      print(f"  t={t:5.2f}s  x={x:6.1f}m  dx={ax[i]:.3f}  dy={ay[i]:.3f}")


if __name__ == '__main__':
  main()
