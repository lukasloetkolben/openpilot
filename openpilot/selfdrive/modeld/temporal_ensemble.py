"""
Temporal ensembling of the model's plan.

The model predicts a multi-second plan every tick, but only the very beginning of it is
consumed before the next tick throws the rest away. This module keeps the recent plans
around, brings them into the current ego frame, re-indexes them over travelled distance
and fuses them per road point.

The fusion output is deliberately *not* a replacement for the model's desired curvature.
It is a correction term (`curvature_delta`) that is added to whatever the model asked for.
That keeps the learned absolute value as the anchor, cancels systematic errors of the
plan -> curvature conversion, and has a natural zero: an empty or single-member buffer
produces exactly 0.0, so the fallback reproduces today's behaviour bit for bit.
"""

import math
import numpy as np
from collections import deque
from dataclasses import dataclass

from openpilot.selfdrive.controls.lib.drive_helpers import MIN_SPEED, MIN_STABLE_DELAY
from openpilot.selfdrive.modeld.constants import ModelConstants, Plan

T_IDXS = np.array(ModelConstants.T_IDXS)


@dataclass(frozen=True)
class EnsembleConfig:
  # buffer
  buffer_len: int = 10            # plans kept, 10 @ 20Hz = 0.5s of history
  max_dt: float = 0.15            # s, gap between model outputs that invalidates the buffer

  # weighting
  age_tau: float = 0.5            # s, decay of the age weight. old contributions come from the
                                  # far, least accurate end of their own horizon, not just older
  sigma_floor: float = 0.02       # rad, lower bound on the assumed yaw uncertainty
  max_member_weight: float = 0.6  # cap on a single member's share, against overconfident outliers

  # validity of a single member
  max_lateral_offset: float = 2.5   # m, closest approach beyond which a member is a different path
  max_heading_mismatch: float = 0.15  # rad, disagreement between what a member predicted the ego
                                      # heading would do and what it actually did. Beyond this the
                                      # member's reference frame is not trustworthy, which happens
                                      # whenever the car is not following its own plan
  min_coverage: float = 0.5         # fraction of the target grid a member must span

  # gates and limits
  gate_psi: float = 0.06          # rad, divergence current vs ensemble that flushes the buffer
  max_curvature_delta: float = 0.01  # 1/m, hard cap on the authority of this module
  min_speed: float = 3.0          # m/s, below this the ensemble is passed through

  def __post_init__(self):
    assert self.buffer_len >= 0
    assert self.sigma_floor > 0.0
    assert 0.0 < self.max_member_weight <= 1.0


# parametrisation that reproduces the current behaviour exactly
FALLBACK_CONFIG = EnsembleConfig(buffer_len=0)


@dataclass
class _Snapshot:
  t: float          # s, frame timestamp (timestampEof), not receive time
  x: float          # world pose of the ego frame this plan was predicted in
  y: float
  psi: float
  px: np.ndarray    # (N,) plan positions in its own ego frame
  py: np.ndarray
  s: np.ndarray     # (N,) arclength along its own plan
  yaw: np.ndarray   # (N,) yaw profile in its own ego frame
  yaw_std: np.ndarray  # (N,)


def _wrap(a: float) -> float:
  return (a + math.pi) % (2 * math.pi) - math.pi


def _arclength(px: np.ndarray, py: np.ndarray) -> np.ndarray:
  d = np.hypot(np.diff(px), np.diff(py))
  return np.concatenate(([0.0], np.cumsum(d)))


def _closest_approach(qx: np.ndarray, qy: np.ndarray, s: np.ndarray) -> tuple[float, float]:
  """Arclength of, and distance to, the point on the polyline closest to the origin."""
  dx, dy = np.diff(qx), np.diff(qy)
  seg2 = dx * dx + dy * dy
  safe = np.maximum(seg2, 1e-9)
  u = np.clip(-(qx[:-1] * dx + qy[:-1] * dy) / safe, 0.0, 1.0)
  u = np.where(seg2 > 1e-9, u, 0.0)
  cx = qx[:-1] + u * dx
  cy = qy[:-1] + u * dy
  d2 = cx * cx + cy * cy
  j = int(np.argmin(d2))
  return float(s[j] + u[j] * (s[j + 1] - s[j])), float(math.sqrt(d2[j]))


class TemporalEnsemble:
  """Sliding buffer of past plans, fused per road point into a curvature correction."""

  def __init__(self, cfg: EnsembleConfig | None = None):
    self.cfg = cfg if cfg is not None else EnsembleConfig()
    self._buf: deque[_Snapshot] = deque(maxlen=max(self.cfg.buffer_len, 1))
    # world frame is arbitrary and session local, only differences between snapshots matter
    self._x = 0.0
    self._y = 0.0
    self._psi = 0.0
    self._last_t: float | None = None
    self._last_pose: np.ndarray | None = None

    # diagnostics, read by the caller for logging
    self.n_members = 0
    self.gated = False
    self.delta = 0.0
    # per member (age, signed yaw difference at the evaluation point, weight there), and why
    # members were dropped. Used to answer whether old predictions survive at all in curves.
    self.member_info: list[tuple[float, float, float]] = []
    self.rejected = {'heading': 0, 'lateral': 0, 'coverage': 0}
    # the fused yaw profile over the current plan's arclength grid, for inspection and plotting.
    # Equal to the current plan's own profile whenever nothing was fused.
    self.s_grid: np.ndarray | None = None
    self.fused_yaw: np.ndarray | None = None

  def reset(self) -> None:
    """Drop all history. The world pose keeps integrating, only correlations are cut."""
    self._buf.clear()
    self.n_members = 0
    self.delta = 0.0

  def _integrate_pose(self, t: float, pose: np.ndarray) -> bool:
    """Advance the world pose to t. Returns False if the step was not usable."""
    prev_t, prev_pose = self._last_t, self._last_pose
    self._last_t, self._last_pose = t, np.asarray(pose, dtype=np.float64).copy()
    if prev_t is None or prev_pose is None:
      return False

    dt = t - prev_t
    if not (0.0 < dt <= self.cfg.max_dt):
      self.reset()
      return False

    # trapezoidal, the model pose is an instantaneous rate at the frame timestamp
    vx = 0.5 * (pose[0] + prev_pose[0])
    vy = 0.5 * (pose[1] + prev_pose[1])
    wz = 0.5 * (pose[5] + prev_pose[5])
    psi_mid = self._psi + 0.5 * wz * dt
    self._x += dt * (vx * math.cos(psi_mid) - vy * math.sin(psi_mid))
    self._y += dt * (vx * math.sin(psi_mid) + vy * math.cos(psi_mid))
    self._psi = _wrap(self._psi + wz * dt)
    return True

  def _project(self, snap: _Snapshot, s_target: np.ndarray,
               t: float) -> tuple[np.ndarray, np.ndarray, np.ndarray] | None:
    """Bring a snapshot into the current ego frame and onto the target arclength grid."""
    dpsi = _wrap(snap.psi - self._psi)

    # did the ego actually do what this member predicted it would do? If not, re-indexing it
    # onto the current plan lines up the wrong road points
    psi_predicted = float(np.interp(t - snap.t, T_IDXS, snap.yaw))
    if abs(_wrap(-dpsi - psi_predicted)) > self.cfg.max_heading_mismatch:
      self.rejected['heading'] += 1
      return None

    c, s_ = math.cos(self._psi), math.sin(self._psi)
    dx, dy = snap.x - self._x, snap.y - self._y
    ox = c * dx + s_ * dy
    oy = -s_ * dx + c * dy

    cd, sd = math.cos(dpsi), math.sin(dpsi)
    qx = ox + cd * snap.px - sd * snap.py
    qy = oy + sd * snap.px + cd * snap.py

    s0, lat_off = _closest_approach(qx, qy, snap.s)
    if lat_off > self.cfg.max_lateral_offset:
      self.rejected['lateral'] += 1
      return None

    s_member = snap.s - s0
    covered = (s_target >= s_member[0]) & (s_target <= s_member[-1])
    if covered.mean() < self.cfg.min_coverage:
      self.rejected['coverage'] += 1
      return None

    yaw = np.interp(s_target, s_member, snap.yaw + dpsi)
    std = np.interp(s_target, s_member, snap.yaw_std)
    return yaw, std, covered

  def update(self, t: float, plan: np.ndarray, plan_stds: np.ndarray | None, pose: np.ndarray,
             v_ego: float, lat_action_t: float, yaw_std: np.ndarray | None = None) -> float:
    """One tick. Returns the curvature correction to add to the model's desired curvature.

    t             frame timestamp in seconds (timestampEof), the capture time of the image
    plan          (N, PLAN_WIDTH) current plan
    plan_stds     (N, PLAN_WIDTH) its stds, may be None if yaw_std is given
    pose          (6,) ego pose, m/s and rad/s in the device frame at t
    yaw_std       (N,) override for the yaw uncertainty. The logged modelV2 does not carry
                  orientation stds, so offline analysis has to supply a proxy
    """
    self.gated = False
    self.delta = 0.0
    self.member_info = []
    self.s_grid = None
    self.fused_yaw = None

    pose_ok = self._integrate_pose(t, pose)

    px = plan[:, Plan.POSITION][:, 0].astype(np.float64)
    py = plan[:, Plan.POSITION][:, 1].astype(np.float64)
    yaw = plan[:, Plan.T_FROM_CURRENT_EULER][:, 2].astype(np.float64)
    if yaw_std is None:
      assert plan_stds is not None, "either plan_stds or yaw_std is required"
      yaw_std = plan_stds[:, Plan.T_FROM_CURRENT_EULER][:, 2]
    yaw_std = np.asarray(yaw_std, dtype=np.float64)
    s_target = _arclength(px, py)

    snap = _Snapshot(t=t, x=self._x, y=self._y, psi=self._psi,
                     px=px, py=py, s=s_target, yaw=yaw, yaw_std=yaw_std)
    self.s_grid, self.fused_yaw = s_target, yaw

    members = []
    if pose_ok and self.cfg.buffer_len > 0 and v_ego >= self.cfg.min_speed:
      for old in self._buf:
        p = self._project(old, s_target, t)
        if p is None:
          continue
        age = t - old.t
        members.append((p[0], p[1], p[2], math.exp(-age / self.cfg.age_tau), age))

    self.n_members = len(members) + 1
    if not members:
      self._push(snap)
      return 0.0

    yaws = np.stack([yaw] + [m[0] for m in members])
    stds = np.stack([yaw_std] + [m[1] for m in members])
    valid = np.stack([np.ones_like(s_target, dtype=bool)] + [m[2] for m in members])
    ages = np.array([1.0] + [m[3] for m in members])[:, None]

    sigma = np.maximum(stds, self.cfg.sigma_floor)
    w = np.where(valid, ages / (sigma * sigma), 0.0)
    w /= w.sum(axis=0, keepdims=True)

    # a single member may not dominate, but the cap must not fight a small ensemble
    cap = np.maximum(self.cfg.max_member_weight, 1.0 / np.maximum(valid.sum(axis=0), 1))
    w = np.minimum(w, cap)
    w /= w.sum(axis=0, keepdims=True)

    yaw_fused = (w * yaws).sum(axis=0)
    dyaw = yaw_fused - yaw
    self.fused_yaw = yaw_fused

    # reactivity gate: a large divergence means the model just saw something the history
    # does not contain. Follow the new plan alone rather than averaging the surprise away.
    if float(np.max(np.abs(dyaw[valid.sum(axis=0) > 1]), initial=0.0)) > self.cfg.gate_psi:
      self.gated = True
      self.reset()
      self._push(snap)
      self.n_members = 1
      self.fused_yaw = yaw
      return 0.0

    # same evaluation point and scaling as get_curvature_from_plan, so the correction is
    # exactly the delta of what the controller would have consumed
    if lat_action_t < MIN_STABLE_DELAY:
      t_eval, scale = MIN_STABLE_DELAY, lat_action_t / MIN_STABLE_DELAY
    else:
      t_eval, scale = lat_action_t, 1.0
    s_eval = float(np.interp(t_eval, T_IDXS, s_target))
    dpsi_target = float(np.interp(s_eval, s_target, dyaw)) * scale

    # record what each member contributed at the evaluation point
    for m, m_yaw, m_w in zip(members, yaws[1:], w[1:], strict=True):
      self.member_info.append((m[4],
                               float(np.interp(s_eval, s_target, m_yaw - yaw)),
                               float(np.interp(s_eval, s_target, m_w))))

    delta = 2 * dpsi_target / (max(v_ego, MIN_SPEED) * max(lat_action_t, 1e-3))
    delta = float(np.clip(delta, -self.cfg.max_curvature_delta, self.cfg.max_curvature_delta))

    self._push(snap)
    self.delta = delta
    return delta

  def _push(self, snap: _Snapshot) -> None:
    if self.cfg.buffer_len > 0:
      self._buf.append(snap)


  @property
  def ego_pose(self) -> tuple[float, float, float]:
    """Integrated ego pose (x, y, psi) in the session local world frame."""
    return self._x, self._y, self._psi


def path_from_yaw(s: np.ndarray, yaw: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
  """Reconstruct a path from a yaw profile over arclength, trapezoidal.

  The fusion works on yaw over distance, so turning it back into waypoints for plotting means
  integrating it. The result starts at the origin of the frame the profile is expressed in.
  """
  ds = np.diff(s)
  cx = np.cos(yaw)
  cy = np.sin(yaw)
  x = np.concatenate(([0.0], np.cumsum(0.5 * (cx[1:] + cx[:-1]) * ds)))
  y = np.concatenate(([0.0], np.cumsum(0.5 * (cy[1:] + cy[:-1]) * ds)))
  return x, y


def stds_monotonic(stds: np.ndarray) -> tuple[bool, float]:
  """Do these stds rise over the horizon? Returns (non decreasing, spearman rho).

  Used to decide whether inverse variance weighting is worth anything at all. If this comes
  back non monotonic on real data, set sigma_floor high enough that the weighting degenerates
  to pure age weighting.
  """
  v = np.asarray(stds, dtype=np.float64).ravel()
  if np.ptp(v) == 0.0:
    return True, 0.0
  rank = np.argsort(np.argsort(v)).astype(np.float64)
  idx = np.arange(len(v), dtype=np.float64)
  rho = float(np.corrcoef(rank, idx)[0, 1])
  return bool(np.all(np.diff(v) >= 0.0)), rho
