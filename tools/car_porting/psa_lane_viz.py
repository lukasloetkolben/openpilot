#!/usr/bin/env python3
"""PSA camera lane line debug viewer.

Shows the road camera stream of a route segment next to a top-down
visualization of the lane lines the PSA LKAS camera reports on CAN
(LKAS_CAM_LANE_LEFT 0x42B / LKAS_CAM_LANE_RIGHT 0x44B), decoded with
opendbc/dbc/psa_aee2010_r3.dbc. Modeled after tools/replay/ui.py (the
radar point debug UI), but fully offline: it reads rlog + fcamera
directly, no replay daemon needed.

  - solid lines: lane lines as reported by the camera (default bus 2)
  - magenta lines: lane lines openpilot injected towards the car
    (panda TX echo, bus 128), when present
  - white dashed line: path implied by the current steering angle

Usage:
  tools/car_porting/psa_lane_viz.py [077b771458fd542f/0000003c--f13451c457]      # full route
  tools/car_porting/psa_lane_viz.py 077b771458fd542f/0000003c--f13451c457/2      # single segment

Controls:
  space      play/pause
  left/right seek -/+ 1 s
  , / .      step one frame (pauses)
  up/down    playback speed
  pgup/pgdn  previous/next segment
  home       back to start
  click/drag on the seek bar to scrub
"""
import argparse
import bisect
import os
import subprocess
import threading
import time
from collections import defaultdict

import numpy as np
import pyray as rl

from opendbc.can.dbc import DBC
from opendbc.can.parser import get_raw_value
from openpilot.common.basedir import BASEDIR
from openpilot.tools.lib.filereader import FileReader
from openpilot.tools.lib.framereader import FfmpegDecoder, ffprobe
from openpilot.tools.lib.logreader import LogReader
from openpilot.tools.lib.route import Route, SegmentRange

DBC_NAME = "psa_aee2010_r3"
LANE_ADDRS = {1067: "left", 1099: "right"}  # LKAS_CAM_LANE_LEFT / _RIGHT
LKA_ADDR = 1010  # LANE_KEEP_ASSIST (car-side ECU -> camera)

# Empirical sign conventions from route 077b771458fd542f/0000003c (ISO frame, +y = left):
#   LINE_LATERAL_POSITION: + = line left of camera (left line ~ +1.7 m)
#   LINE_HEADING:          same sign as ISO yaw (+ = pointing left)
#   LINE_CURVATURE:        + = curving left, SAME sign as steering curvature (verified against
#                          yaw-rate-calibrated steering angle: corr peak +0.5s preview, slope ~+1)
CURV_SIGN = 1.0

# per-field not-available raw value is max-1 (32766 / 2046 / 510)
NA_RAW = {"LINE_HEADING": 32766, "LINE_CURVATURE_RATE": 32766, "LINE_CURVATURE": 2046, "LINE_LATERAL_POSITION": 510}

WHEELBASE = 2.54
STEER_RATIO = 17.6

DRAW_DIST = 60.0  # m, how far ahead lane polynomials are evaluated
VIEW_DIST = 70.0  # m, top-down view range forward
VIEW_LAT = 8.0    # m, top-down view range each side


def decode_signals(msg_def, dat):
  out = {}
  for name, sig in msg_def.sigs.items():
    raw = get_raw_value(dat, sig)
    if sig.is_signed and raw & (1 << (sig.size - 1)):
      raw -= 1 << sig.size
    out[name] = raw * sig.factor + sig.offset
    if name in NA_RAW and raw == NA_RAW[name]:
      out[name] = None
  return out


class Series:
  """Time-indexed samples with latest-at-or-before lookup."""

  def __init__(self):
    self.ts: list[float] = []
    self.vals: list[dict] = []

  def append(self, t, val):
    self.ts.append(t)
    self.vals.append(val)

  def at(self, t):
    i = bisect.bisect_right(self.ts, t) - 1
    if i < 0:
      return None, None
    return self.vals[i], t - self.ts[i]


def load_route_data(route_str, cam_bus):
  sr = SegmentRange(route_str)
  seg_idxs = sr.seg_idxs

  print(f"loading camera paths for {sr.route_name} ({len(seg_idxs)} segments)...")
  route = Route(sr.route_name)
  fcams, qcams = route.camera_paths(), route.qcamera_paths()
  use_fcam = all(fcams[i] is not None for i in seg_idxs)
  if not use_fcam:
    print("fcamera.hevc not available for all segments, falling back to qcamera.ts")
  cam_paths = [(fcams if use_fcam else qcams)[i] for i in seg_idxs]
  assert all(p is not None for p in cam_paths), "camera files missing for some segments"
  idx_which = "roadEncodeIdx" if use_fcam else "qRoadEncodeIdx"

  dbc = DBC(DBC_NAME)
  lane_defs = {addr: dbc.msgs[addr] for addr in LANE_ADDRS}
  lka_def = dbc.msgs[LKA_ADDR]

  print(f"reading rlogs for {route_str}...")
  frame_ts_by_seg: dict[int, dict[int, int]] = defaultdict(dict)
  # raw (logMonoTime, signals) lists; converted to relative-time Series once t0 is known
  lanes_raw = {cam_bus: {"left": [], "right": []}, 128: {"left": [], "right": []}}
  lka_raw, car_raw = [], []

  for msg in LogReader(route_str):
    which = msg.which()
    if which == idx_which:
      ei = getattr(msg, idx_which)
      frame_ts_by_seg[ei.segmentNum][ei.segmentId] = ei.timestampSof
    elif which == "can":
      for c in msg.can:
        if c.address in LANE_ADDRS and c.src in lanes_raw:
          lanes_raw[c.src][LANE_ADDRS[c.address]].append((msg.logMonoTime, decode_signals(lane_defs[c.address], c.dat)))
        elif c.address == LKA_ADDR and c.src == 0:
          lka_raw.append((msg.logMonoTime, decode_signals(lka_def, c.dat)))
    elif which == "carState":
      cs = msg.carState
      car_raw.append((msg.logMonoTime, {"vEgo": cs.vEgo, "steeringAngleDeg": cs.steeringAngleDeg}))

  assert frame_ts_by_seg, f"no {idx_which} in rlogs, cannot sync video to CAN"
  t0 = min(min(d.values()) for d in frame_ts_by_seg.values())

  counts, ts_parts = [], []
  for seg in seg_idxs:
    by_id = frame_ts_by_seg.get(seg, {})
    n = max(by_id) + 1 if by_id else 0
    ts = np.full(n, np.nan)
    for i, t in by_id.items():
      ts[i] = (t - t0) * 1e-9
    # fill gaps by interpolation and extrapolate head/tail at 20 Hz so the global timeline stays monotonic
    nans = np.isnan(ts)
    if nans.any() and (~nans).any():
      known = np.flatnonzero(~nans)
      ts[nans] = np.interp(np.flatnonzero(nans), known, ts[known])
      if known[0] > 0:
        ts[:known[0]] = ts[known[0]] - 0.05 * np.arange(known[0], 0, -1)
      if known[-1] < n - 1:
        ts[known[-1] + 1:] = ts[known[-1]] + 0.05 * np.arange(1, n - known[-1])
    counts.append(n)
    ts_parts.append(ts)
  frame_ts = np.maximum.accumulate(np.concatenate(ts_parts))

  def to_series(raw):
    s = Series()
    for t_ns, val in raw:
      s.append((t_ns - t0) * 1e-9, val)
    return s

  lanes = {bus: {side: to_series(raw) for side, raw in sides.items()} for bus, sides in lanes_raw.items()}
  lka, car = to_series(lka_raw), to_series(car_raw)

  n_cam = len(lanes[cam_bus]["left"].ts) + len(lanes[cam_bus]["right"].ts)
  n_inj = len(lanes[128]["left"].ts) + len(lanes[128]["right"].ts)
  print(f"{len(frame_ts)} video frames, {n_cam} lane msgs on bus {cam_bus}, {n_inj} injected (bus 128), {len(lka_raw)} LKA msgs, {len(car_raw)} carState")
  return cam_paths, counts, seg_idxs, use_fcam, frame_ts, lanes, lka, car


class VideoSource(threading.Thread):
  """Background GOP decoder so playback doesn't stall on ffmpeg."""

  AHEAD = 40
  BEHIND = 10

  def __init__(self, cam_path, downscale=2):
    super().__init__(daemon=True)
    self.dec = FfmpegDecoder(cam_path)
    self.ds = downscale
    self.w = -(-self.dec.w // downscale)
    self.h = -(-self.dec.h // downscale)
    self.frame_count = self.dec.frame_count
    self.frames: dict[int, np.ndarray] = {}
    self.lock = threading.Lock()
    self.want = 0
    self.stop_flag = False
    self.start()

  def get(self, fidx):
    with self.lock:
      self.want = fidx
      return self.frames.get(fidx)

  def stop(self):
    self.stop_flag = True

  def _prune(self, want):
    for k in [k for k in self.frames if k < want - self.BEHIND or k > want + self.AHEAD + 30]:
      del self.frames[k]

  def run(self):
    while not self.stop_flag:
      with self.lock:
        want = self.want
        missing = next((i for i in range(want, min(want + self.AHEAD, self.frame_count)) if i not in self.frames), None)
      if missing is None:
        time.sleep(0.02)
        continue
      start = self.dec.get_gop_start(missing)
      try:
        for fidx, frm in self.dec.get_iterator(start):
          if self.ds > 1:
            frm = np.ascontiguousarray(frm[::self.ds, ::self.ds])
          with self.lock:
            self.frames[fidx] = frm
            want = self.want
            self._prune(want)
          if self.stop_flag or fidx >= want + self.AHEAD or fidx < want - self.BEHIND:
            break  # reached lookahead or user seeked away, restart from new position
      except Exception as e:
        print(f"video decode error at frame {missing}: {e}")
        time.sleep(0.5)


class QcamSource(threading.Thread):
  """Decodes the whole (small) qcamera.ts into memory in the background."""

  def __init__(self, cam_path, expected_count):
    super().__init__(daemon=True)
    self.fn = cam_path
    stream = next(s for s in ffprobe(cam_path, fmt="mpegts")["streams"] if s["codec_type"] == "video")
    self.w, self.h = stream["width"], stream["height"]
    self.frame_count = expected_count
    self.frames: list[np.ndarray] = []
    self.stop_flag = False
    self.start()

  def get(self, fidx):
    return self.frames[fidx] if fidx < len(self.frames) else None

  def stop(self):
    self.stop_flag = True

  def run(self):
    with FileReader(self.fn) as f:
      data = f.read()
    proc = subprocess.Popen(["ffmpeg", "-v", "quiet", "-f", "mpegts", "-i", "pipe:0",
                             "-f", "rawvideo", "-pix_fmt", "rgb24", "pipe:1"],
                            stdin=subprocess.PIPE, stdout=subprocess.PIPE)
    threading.Thread(target=lambda: (proc.stdin.write(data), proc.stdin.close()), daemon=True).start()
    frame_bytes = self.w * self.h * 3
    while not self.stop_flag:
      buf = proc.stdout.read(frame_bytes)
      if len(buf) < frame_bytes:
        break
      self.frames.append(np.frombuffer(buf, dtype=np.uint8).reshape(self.h, self.w, 3))
    proc.kill()
    self.frame_count = min(self.frame_count, len(self.frames))
    print(f"qcamera decoded: {len(self.frames)} frames")


class MultiVideo:
  """Concatenates per-segment video sources; creates them in the background and evicts distant ones."""

  def __init__(self, cam_paths, counts, use_fcam, downscale, first_seg=0):
    self.paths, self.counts, self.use_fcam, self.ds = cam_paths, counts, use_fcam, downscale
    self.offsets = np.concatenate(([0], np.cumsum(counts)))
    self.frame_count = int(self.offsets[-1])
    self.sources: dict[int, VideoSource | QcamSource] = {}
    self.pending: set[int] = set()
    self.lock = threading.Lock()
    first = first_seg if counts[first_seg] > 0 else next(i for i, c in enumerate(counts) if c > 0)
    self.sources[first] = self._make(first)
    self.w, self.h = self.sources[first].w, self.sources[first].h

  def _make(self, i):
    print(f"opening video for segment {i}...")
    if self.use_fcam:
      return VideoSource(self.paths[i], downscale=self.ds)
    return QcamSource(self.paths[i], self.counts[i])

  def _ensure(self, i):
    with self.lock:
      if not (0 <= i < len(self.paths)) or self.counts[i] == 0 or i in self.sources or i in self.pending:
        return
      self.pending.add(i)

    def create():
      src = self._make(i)
      with self.lock:
        self.sources[i] = src
        self.pending.discard(i)
    threading.Thread(target=create, daemon=True).start()

  def locate(self, fidx):
    i = int(np.searchsorted(self.offsets, fidx, side="right") - 1)
    return i, fidx - int(self.offsets[i])

  def get(self, fidx):
    i, local = self.locate(fidx)
    self._ensure(i)
    if local > self.counts[i] - 400:  # get a head start on the next segment
      self._ensure(i + 1)
    with self.lock:
      for k in [k for k in self.sources if abs(k - i) > 1]:
        self.sources.pop(k).stop()
      src = self.sources.get(i)
    return src.get(local) if src else None

  def stop(self):
    with self.lock:
      for s in self.sources.values():
        s.stop()


def lane_poly(sample, dist=DRAW_DIST, step=2.0):
  """Evaluate lane line polynomial -> (x forward, y left) arrays, or None."""
  pos, head, curv = sample.get("LINE_LATERAL_POSITION"), sample.get("LINE_HEADING"), sample.get("LINE_CURVATURE")
  if pos is None:
    return None
  head = head or 0.0
  curv = CURV_SIGN * (curv or 0.0)
  xs = np.arange(0.0, dist + 1e-6, step)
  ys = pos + head * xs + curv * xs ** 2 / 2.0
  return xs, ys


class TopDownView:
  def __init__(self, rect):
    self.rect = rect

  def to_px(self, fx, fy):
    r = self.rect
    sx = r.x + r.width / 2 - fy * (r.width / 2) / VIEW_LAT
    sy = r.y + r.height - 14 - fx * (r.height - 28) / VIEW_DIST
    return sx, sy

  def draw_polyline(self, xs, ys, color, thick=2.0, dashed=False):
    pts = [self.to_px(x, y) for x, y in zip(xs, ys, strict=True)]
    for i in range(len(pts) - 1):
      if dashed and i % 2:
        continue
      rl.draw_line_ex(rl.Vector2(*pts[i]), rl.Vector2(*pts[i + 1]), thick, color)

  def draw(self, font, lane_state, inj_state, car_sample):
    r = self.rect
    rl.draw_rectangle_rec(r, rl.Color(24, 24, 28, 255))
    rl.draw_rectangle_lines_ex(r, 1, rl.Color(70, 70, 78, 255))
    rl.begin_scissor_mode(int(r.x), int(r.y), int(r.width), int(r.height))

    grid = rl.Color(52, 52, 60, 255)
    for d in range(0, int(VIEW_DIST) + 1, 10):
      sx0, sy = self.to_px(d, VIEW_LAT)
      sx1, _ = self.to_px(d, -VIEW_LAT)
      rl.draw_line_ex(rl.Vector2(sx0, sy), rl.Vector2(sx1, sy), 1, grid)
      rl.draw_text_ex(font, f"{d}m", rl.Vector2(r.x + 4, sy - 14), 13, 0, rl.Color(110, 110, 120, 255))
    cx0, cy0 = self.to_px(0, 0)
    cx1, cy1 = self.to_px(VIEW_DIST, 0)
    rl.draw_line_ex(rl.Vector2(cx0, cy0), rl.Vector2(cx1, cy1), 1, grid)

    # ego
    ex, ey = self.to_px(0, 0)
    car_w = 1.8 * (r.width / 2) / VIEW_LAT
    car_l = 4.4 * (r.height - 28) / VIEW_DIST
    rl.draw_rectangle_rec(rl.Rectangle(ex - car_w / 2, ey, car_w, min(car_l, 12 + car_l)), rl.Color(120, 120, 130, 255))

    # steering-angle implied path
    if car_sample is not None:
      kappa = np.tan(np.radians(car_sample["steeringAngleDeg"]) / STEER_RATIO) / WHEELBASE
      xs = np.arange(0.0, DRAW_DIST + 1e-6, 2.0)
      self.draw_polyline(xs, kappa * xs ** 2 / 2.0, rl.Color(255, 255, 255, 180), 1.5, dashed=True)

    for side, base in (("left", rl.Color(255, 220, 60, 255)), ("right", rl.Color(90, 255, 120, 255))):
      sample, age = lane_state[side]
      if sample is None or age > 0.5:
        continue
      poly = lane_poly(sample)
      if poly is None:
        continue
      valid, tracked = sample.get("LINE_VALID"), sample.get("LINE_TRACKED")
      color = base if (valid and tracked) else (rl.Color(255, 140, 40, 255) if valid else rl.Color(150, 60, 60, 255))
      self.draw_polyline(*poly, color, 2.0 + (sample.get("LINE_QUALITY") or 0) * 0.7)

    # openpilot-injected lines (panda TX echo)
    for side in ("left", "right"):
      sample, age = inj_state[side]
      if sample is None or age > 0.5:
        continue
      poly = lane_poly(sample)
      if poly is not None:
        self.draw_polyline(*poly, rl.Color(255, 80, 255, 220), 1.8, dashed=True)
    rl.end_scissor_mode()


def fmt_line(side, sample, age):
  if sample is None:
    return f"{side:<5} --- no data ---"

  def f(key, scale, fmt, unit=""):
    v = sample.get(key)
    return "   NA  " if v is None else format(v * scale, fmt) + unit

  flags = f"q{sample.get('LINE_QUALITY', 0):.0f} {'V' if sample.get('LINE_VALID') else '.'}{'T' if sample.get('LINE_TRACKED') else '.'}"
  geo = f"pos {f('LINE_LATERAL_POSITION', 1, '+6.2f', 'm')}  head {f('LINE_HEADING', 1000, '+7.2f', 'mrad')}  curv {f('LINE_CURVATURE', 1000, '+7.3f', '/km')}"
  return f"{side:<5} {geo}  rate {f('LINE_CURVATURE_RATE', 1, '+6.0f')}  {flags}  age {age:4.2f}s"


def main():
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument("route", nargs="?", default="077b771458fd542f/0000003c--f13451c457",
                  help="route or route/segment, e.g. 077b771458fd542f/0000003c--f13451c457[/2]")
  ap.add_argument("--bus", type=int, default=2, help="CAN bus with the camera lane messages (default 2 = native camera; 128 = what openpilot sent)")
  ap.add_argument("--start-seg", type=int, default=2, help="segment to start playback at (default 2)")
  ap.add_argument("--seek", type=float, help="start playback at this route-relative time in seconds (overrides --start-seg)")
  ap.add_argument("--full-res", action="store_true", help="decode video at full resolution")
  ap.add_argument("--screenshot", help="save a screenshot of the UI to this path and exit (for testing)")
  args = ap.parse_args()

  cam_paths, counts, seg_idxs, use_fcam, frame_ts, lanes, lka, car = load_route_data(args.route, args.bus)
  offsets = np.concatenate(([0], np.cumsum(counts)))
  if args.seek is not None:
    seek_fidx = int(np.clip(np.searchsorted(frame_ts, args.seek), 0, len(frame_ts) - 1))
    start_seg = int(np.searchsorted(offsets, seek_fidx, side="right") - 1)
  else:
    start_seg = seg_idxs.index(args.start_seg) if args.start_seg in seg_idxs else 0
    seek_fidx = int(offsets[start_seg])
  video = MultiVideo(cam_paths, counts, use_fcam, downscale=1 if args.full_res else 2, first_seg=start_seg)
  n_frames = min(len(frame_ts), video.frame_count)

  # green strip on the seek bar where both camera lines are valid+tracked
  def both_tracked(t):
    ok = True
    for side in ("left", "right"):
      s, age = lanes[args.bus][side].at(t)
      ok &= s is not None and age < 0.5 and bool(s.get("LINE_VALID")) and bool(s.get("LINE_TRACKED"))
    return ok
  tracked_strip = np.array([both_tracked(t) for t in frame_ts[:n_frames]])

  pad = 8
  vid_w, vid_h = video.w, video.h
  disp = 2 if vid_w < 700 else 1  # upscale the small qcamera for display
  disp_w, disp_h = vid_w * disp, vid_h * disp
  td_w = 420
  info_h, bar_h = 118, 14
  win_w = pad + disp_w + pad + td_w + pad
  win_h = pad + disp_h + pad + info_h + pad + bar_h + pad

  rl.set_trace_log_level(rl.TraceLogLevel.LOG_ERROR)
  rl.init_window(win_w, win_h, f"PSA lane debug - {args.route}")
  rl.set_target_fps(60)

  font_path = os.path.join(BASEDIR, "openpilot/selfdrive/assets/fonts/JetBrainsMono-Medium.ttf")
  font = rl.load_font_ex(font_path, 16, None, 0) if os.path.exists(font_path) else rl.get_font_default()

  tex_img = rl.gen_image_color(vid_w, vid_h, rl.BLANK)
  tex = rl.load_texture_from_image(tex_img)
  rl.unload_image(tex_img)
  rgba = np.zeros((vid_h, vid_w, 4), dtype=np.uint8)
  rgba[:, :, 3] = 255

  vid_rect = rl.Rectangle(pad, pad, disp_w, disp_h)
  td = TopDownView(rl.Rectangle(pad + disp_w + pad, pad, td_w, disp_h))
  bar_rect = rl.Rectangle(pad, win_h - pad - bar_h, win_w - 2 * pad, bar_h)

  playing, speed, play_t = True, 1.0, float(frame_ts[seek_fidx])
  fidx, have_frame, drawn = seek_fidx, False, 0

  while not rl.window_should_close():
    n_frames = min(n_frames, video.frame_count)
    if rl.is_key_pressed(rl.KeyboardKey.KEY_SPACE):
      playing = not playing
    if rl.is_key_pressed(rl.KeyboardKey.KEY_UP):
      speed = min(speed * 2, 8.0)
    if rl.is_key_pressed(rl.KeyboardKey.KEY_DOWN):
      speed = max(speed / 2, 0.125)
    if rl.is_key_pressed(rl.KeyboardKey.KEY_HOME):
      play_t = float(frame_ts[0])
    for key, dt in ((rl.KeyboardKey.KEY_LEFT, -1.0), (rl.KeyboardKey.KEY_RIGHT, 1.0)):
      if rl.is_key_pressed(key):
        play_t = float(np.clip(play_t + dt, frame_ts[0], frame_ts[n_frames - 1]))
    for key, di in ((rl.KeyboardKey.KEY_COMMA, -1), (rl.KeyboardKey.KEY_PERIOD, 1)):
      if rl.is_key_pressed(key):
        playing = False
        play_t = float(frame_ts[int(np.clip(fidx + di, 0, n_frames - 1))])
    for key, di in ((rl.KeyboardKey.KEY_PAGE_UP, -1), (rl.KeyboardKey.KEY_PAGE_DOWN, 1)):
      if rl.is_key_pressed(key):
        cur_seg, _ = video.locate(fidx)
        play_t = float(frame_ts[int(video.offsets[int(np.clip(cur_seg + di, 0, len(counts) - 1))])])
    if rl.is_mouse_button_down(rl.MouseButton.MOUSE_BUTTON_LEFT):
      mp = rl.get_mouse_position()
      if rl.check_collision_point_rec(mp, bar_rect):
        frac = (mp.x - bar_rect.x) / bar_rect.width
        play_t = float(frame_ts[int(np.clip(frac * n_frames, 0, n_frames - 1))])

    if playing and have_frame:
      play_t += rl.get_frame_time() * speed
      if play_t >= frame_ts[n_frames - 1]:
        play_t, playing = float(frame_ts[n_frames - 1]), False

    fidx = int(np.clip(np.searchsorted(frame_ts[:n_frames], play_t, side="right") - 1, 0, n_frames - 1))
    frame = video.get(fidx)
    if frame is not None:
      rgba[:, :, :3] = frame[:vid_h, :vid_w]
      rl.update_texture(tex, rl.ffi.cast("void *", rgba.ctypes.data))
      have_frame = True
    elif playing:
      play_t -= rl.get_frame_time() * speed  # buffering: hold position

    t = float(frame_ts[fidx])
    cam_state = {side: lanes[args.bus][side].at(t) for side in ("left", "right")}
    inj_state = {side: lanes[128][side].at(t) for side in ("left", "right")}
    car_sample, _ = car.at(t)
    lka_sample, _ = lka.at(t)

    rl.begin_drawing()
    rl.clear_background(rl.Color(16, 16, 18, 255))

    rl.draw_texture_pro(tex, rl.Rectangle(0, 0, vid_w, vid_h), vid_rect, rl.Vector2(0, 0), 0, rl.WHITE)
    if frame is None:
      rl.draw_text_ex(font, "buffering...", rl.Vector2(vid_rect.x + 12, vid_rect.y + 12), 20, 0, rl.YELLOW)

    td.draw(font, cam_state, inj_state, car_sample)

    ix, iy = pad + 4, pad + disp_h + pad
    white, gray = rl.Color(235, 235, 235, 255), rl.Color(150, 150, 160, 255)
    state = f"{'PLAYING' if playing else 'PAUSED '} x{speed:g}"
    cur_seg, _ = video.locate(fidx)
    rl.draw_text_ex(font, f"seg {seg_idxs[cur_seg]}  frame {fidx:4d}/{n_frames - 1}  t={t:6.2f}s  [{state}]  camera lanes: bus {args.bus}",
                    rl.Vector2(ix, iy), 16, 0, white)
    if car_sample is not None or lka_sample is not None:
      cs = f"vEgo {car_sample['vEgo'] * 3.6:5.1f} km/h  steer {car_sample['steeringAngleDeg']:+6.2f} deg" if car_sample else ""
      lk = ""
      if lka_sample:
        lk = f"   LKA(0x3F2): STATUS={lka_sample['STATUS']:.0f}  SET_ANGLE={lka_sample['SET_ANGLE']:+6.1f}  TORQUE_FACTOR={lka_sample['TORQUE_FACTOR']:3.0f}"
      rl.draw_text_ex(font, cs + lk, rl.Vector2(ix, iy + 20), 16, 0, white)
    rl.draw_text_ex(font, fmt_line("LEFT", *cam_state["left"]), rl.Vector2(ix, iy + 44), 16, 0, rl.Color(255, 220, 60, 255))
    rl.draw_text_ex(font, fmt_line("RIGHT", *cam_state["right"]), rl.Vector2(ix, iy + 64), 16, 0, rl.Color(90, 255, 120, 255))
    ls, la = cam_state["left"]
    rs, ra = cam_state["right"]
    if ls and rs and ls.get("LINE_LATERAL_POSITION") is not None and rs.get("LINE_LATERAL_POSITION") is not None:
      width = ls["LINE_LATERAL_POSITION"] - rs["LINE_LATERAL_POSITION"]
      center = (ls["LINE_LATERAL_POSITION"] + rs["LINE_LATERAL_POSITION"]) / 2
      rl.draw_text_ex(font, f"lane width {width:4.2f}m   car offset from center {-center:+5.2f}m (+ = right of center)",
                      rl.Vector2(ix, iy + 84), 16, 0, gray)
    if inj_state["left"][0] or inj_state["right"][0]:
      rl.draw_text_ex(font, "magenta dashed = openpilot injected (bus 128)", rl.Vector2(ix + 620, iy), 14, 0, rl.Color(255, 80, 255, 255))
    rl.draw_text_ex(font, "space=pause  arrows=seek/speed  ,/.=frame  pgup/pgdn=segment  home=start  click bar=scrub",
                    rl.Vector2(ix, iy + info_h - 16), 14, 0, gray)

    rl.draw_rectangle_rec(bar_rect, rl.Color(40, 40, 46, 255))
    runs = np.flatnonzero(np.diff(np.concatenate(([0], tracked_strip.astype(np.int8), [0]))))
    for a, b in zip(runs[::2], runs[1::2], strict=True):
      rl.draw_rectangle_rec(rl.Rectangle(bar_rect.x + a / n_frames * bar_rect.width, bar_rect.y + bar_rect.height - 4,
                                         max(1.0, (b - a) / n_frames * bar_rect.width), 4), rl.Color(60, 180, 90, 255))
    for si, off in enumerate(video.offsets[:-1]):
      bx = bar_rect.x + off / n_frames * bar_rect.width
      if si > 0:
        rl.draw_rectangle_rec(rl.Rectangle(bx, bar_rect.y, 1, bar_rect.height), rl.Color(130, 130, 140, 255))
      rl.draw_text_ex(font, str(seg_idxs[si]), rl.Vector2(bx + 4, bar_rect.y - 1), 13, 0, rl.Color(150, 150, 160, 255))
    px = bar_rect.x + fidx / max(1, n_frames - 1) * bar_rect.width
    rl.draw_rectangle_rec(rl.Rectangle(px - 2, bar_rect.y, 4, bar_rect.height), rl.WHITE)

    rl.end_drawing()

    if args.screenshot and have_frame:
      drawn += 1
      if drawn >= 10:
        rl.take_screenshot(args.screenshot)
        break

  video.stop()
  rl.close_window()


if __name__ == "__main__":
  main()
