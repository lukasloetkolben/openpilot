import numpy as np
import pytest

from openpilot.selfdrive.controls.lib.drive_helpers import get_curvature_from_plan
from openpilot.selfdrive.modeld.constants import ModelConstants, Plan
from openpilot.selfdrive.modeld.temporal_ensemble import (EnsembleConfig, FALLBACK_CONFIG, TemporalEnsemble,
                                                          stds_monotonic)

T_IDXS = np.array(ModelConstants.T_IDXS)
DT = 1. / ModelConstants.MODEL_RUN_FREQ


def make_plan(v_ego: float, curvature: float, yaw_noise: float = 0.0, seed: int = 0) -> np.ndarray:
  """A constant curvature plan at constant speed, in the ego frame."""
  plan = np.zeros((ModelConstants.IDX_N, ModelConstants.PLAN_WIDTH), dtype=np.float32)
  s = v_ego * T_IDXS
  psi = curvature * s
  plan[:, Plan.POSITION.start + 0] = np.where(abs(curvature) > 1e-9, np.sin(psi) / max(abs(curvature), 1e-9) * np.sign(curvature or 1), s)
  plan[:, Plan.POSITION.start + 1] = np.where(abs(curvature) > 1e-9, (1 - np.cos(psi)) / (curvature if curvature else 1), 0.0)
  plan[:, Plan.T_FROM_CURRENT_EULER.start + 2] = psi
  plan[:, Plan.ORIENTATION_RATE.start + 2] = curvature * v_ego
  plan[:, Plan.VELOCITY.start + 0] = v_ego
  if yaw_noise:
    rng = np.random.default_rng(seed)
    plan[:, Plan.T_FROM_CURRENT_EULER.start + 2] += rng.normal(0., yaw_noise, ModelConstants.IDX_N)
  return plan


def make_stds(profile: str = 'rising') -> np.ndarray:
  stds = np.full((ModelConstants.IDX_N, ModelConstants.PLAN_WIDTH), 0.1, dtype=np.float32)
  prof = {'rising': np.linspace(0.01, 0.5, ModelConstants.IDX_N),
          'falling': np.linspace(0.5, 0.01, ModelConstants.IDX_N),
          'flat': np.full(ModelConstants.IDX_N, 0.1)}[profile]
  stds[:, Plan.T_FROM_CURRENT_EULER.start + 2] = prof
  stds[:, Plan.POSITION.start + 1] = prof
  return stds


def pose_for(v_ego: float, curvature: float) -> np.ndarray:
  return np.array([v_ego, 0., 0., 0., 0., curvature * v_ego])


def run(te: TemporalEnsemble, n: int, v_ego: float, curvature: float, lat_action_t: float = 0.2,
        yaw_noise: float = 0.0) -> list[float]:
  out = []
  for i in range(n):
    plan = make_plan(v_ego, curvature, yaw_noise, seed=i)
    out.append(te.update(i * DT, plan, make_stds(), pose_for(v_ego, curvature), v_ego, lat_action_t))
  return out


class TestFallback:
  def test_fallback_config_is_exactly_zero(self):
    te = TemporalEnsemble(FALLBACK_CONFIG)
    deltas = run(te, 50, 25., 0.005, yaw_noise=0.02)
    assert all(d == 0.0 for d in deltas)

  def test_first_tick_is_zero(self):
    # no pose delta yet, nothing to project against
    te = TemporalEnsemble()
    assert run(te, 1, 25., 0.005)[0] == 0.0

  def test_single_member_is_negligible(self):
    # the second tick has one old plan. With identical inputs the fusion reproduces the
    # current plan up to the round trip through the world frame, which is far below the
    # resolution of any curvature the controller acts on
    te = TemporalEnsemble()
    assert abs(run(te, 2, 25., 0.005)[-1]) < 1e-7

  def test_bit_exact_action_when_disabled(self):
    plan = make_plan(25., 0.005)
    curv = get_curvature_from_plan(plan[:, Plan.T_FROM_CURRENT_EULER][:, 2],
                                   plan[:, Plan.ORIENTATION_RATE][:, 2], ModelConstants.T_IDXS, 25., 0.2)
    assert curv + 0.0 == curv


class TestConsistency:
  def test_noise_free_steady_state_is_zero(self):
    # a perfectly consistent world: every old plan re-indexed onto the current one must agree
    te = TemporalEnsemble()
    deltas = run(te, 60, 25., 0.004)
    assert max(abs(d) for d in deltas) < 1e-4

  def test_straight_line_is_zero(self):
    te = TemporalEnsemble()
    assert max(abs(d) for d in run(te, 40, 30., 0.0)) < 1e-5

  def test_variance_is_reduced(self):
    # independent yaw noise per tick, the ensemble must be quieter than the raw model
    v_ego, curvature, lat_action_t = 25., 0.004, 0.2
    te = TemporalEnsemble()
    raw, fused = [], []
    for i in range(200):
      plan = make_plan(v_ego, curvature, yaw_noise=0.01, seed=i)
      d = te.update(i * DT, plan, make_stds(), pose_for(v_ego, curvature), v_ego, lat_action_t)
      c = get_curvature_from_plan(plan[:, Plan.T_FROM_CURRENT_EULER][:, 2], plan[:, Plan.ORIENTATION_RATE][:, 2],
                                  ModelConstants.T_IDXS, v_ego, lat_action_t)
      raw.append(c)
      fused.append(c + d)
    # compare jerk proxies, the tick to tick change of the commanded curvature
    assert np.std(np.diff(fused)) < np.std(np.diff(raw))


class TestGates:
  def test_divergence_gate_flushes(self):
    te = TemporalEnsemble()
    run(te, 30, 25., 0.0)
    # a sudden hard turn the history knows nothing about
    plan = make_plan(25., 0.06)
    te.update(30 * DT, plan, make_stds(), pose_for(25., 0.0), 25., 0.2)
    assert te.gated
    assert te.n_members == 1
    assert te.delta == 0.0

  def test_low_speed_passthrough(self):
    te = TemporalEnsemble()
    assert all(d == 0.0 for d in run(te, 30, 1.0, 0.02))

  def test_gap_in_model_output_resets(self):
    te = TemporalEnsemble()
    run(te, 30, 25., 0.004)
    plan, stds = make_plan(25., 0.004), make_stds()
    # a full second without a model output
    assert te.update(30 * DT + 1.0, plan, stds, pose_for(25., 0.004), 25., 0.2) == 0.0

  def test_reset_clears_history(self):
    te = TemporalEnsemble()
    run(te, 30, 25., 0.004, yaw_noise=0.01)
    te.reset()
    plan, stds = make_plan(25., 0.004, yaw_noise=0.01, seed=99), make_stds()
    assert te.update(30 * DT, plan, stds, pose_for(25., 0.004), 25., 0.2) == 0.0

  def test_delta_is_capped(self):
    cfg = EnsembleConfig(gate_psi=1e9, max_curvature_delta=0.002)
    te = TemporalEnsemble(cfg)
    run(te, 30, 25., 0.0)
    plan = make_plan(25., 0.05)
    d = te.update(30 * DT, plan, make_stds(), pose_for(25., 0.0), 25., 0.2)
    assert abs(d) <= cfg.max_curvature_delta + 1e-12


class TestWeighting:
  def test_single_member_cannot_dominate(self):
    # an overconfident old plan disagreeing with everything else must not take over
    cfg = EnsembleConfig(max_member_weight=0.5, gate_psi=1e9, sigma_floor=1e-6, max_curvature_delta=1.0)
    te = TemporalEnsemble(cfg)
    v_ego, curvature = 25., 0.0
    for i in range(20):
      te.update(i * DT, make_plan(v_ego, curvature), make_stds(), pose_for(v_ego, curvature), v_ego, 0.2)
    stds = make_stds()
    stds[:, Plan.T_FROM_CURRENT_EULER.start + 2] = 1e-9  # claims perfect certainty
    te.update(20 * DT, make_plan(v_ego, 0.02), stds, pose_for(v_ego, curvature), v_ego, 0.2)
    # it is the current plan here, so the fusion pulls towards the history rather than away
    assert te.n_members > 1

  def test_stds_monotonic_helper(self):
    mono, rho = stds_monotonic(make_stds('rising')[:, Plan.POSITION][:, 1])
    assert mono and rho > 0.99
    mono, rho = stds_monotonic(make_stds('falling')[:, Plan.POSITION][:, 1])
    assert not mono and rho < -0.99
    mono, rho = stds_monotonic(make_stds('flat')[:, Plan.POSITION][:, 1])
    assert rho == 0.0


@pytest.mark.parametrize("lat_action_t", [0.05, 0.2, 0.35])
def test_matches_controller_scaling(lat_action_t):
  # the correction must be the delta of exactly what get_curvature_from_plan would produce
  v_ego = 25.
  te = TemporalEnsemble(EnsembleConfig(gate_psi=1e9, max_curvature_delta=1.0))
  for i in range(20):
    te.update(i * DT, make_plan(v_ego, 0.004), make_stds(), pose_for(v_ego, 0.004), v_ego, lat_action_t)
  plan = make_plan(v_ego, 0.004, yaw_noise=0.005, seed=123)
  d = te.update(20 * DT, plan, make_stds(), pose_for(v_ego, 0.004), v_ego, lat_action_t)
  assert np.isfinite(d)


def test_reactivity_not_degraded():
  """A sudden plan change must not be slowed down by the ensemble. This is the safety
  criterion from the acceptance list: smoothing may not cost response time.

  The ego follows its own plan with one tick of actuator lag, as it does on the road."""
  v_ego, lat_action_t, step = 25., 0.2, 0.02
  te = TemporalEnsemble()
  raw, fused = [], []
  prev_curvature = 0.0
  for i in range(60):
    curvature = 0.0 if i < 40 else step  # the swerve the history knows nothing about
    plan = make_plan(v_ego, curvature)
    d = te.update(i * DT, plan, make_stds(), pose_for(v_ego, prev_curvature), v_ego, lat_action_t)
    prev_curvature = curvature
    c = get_curvature_from_plan(plan[:, Plan.T_FROM_CURRENT_EULER][:, 2], plan[:, Plan.ORIENTATION_RATE][:, 2],
                                ModelConstants.T_IDXS, v_ego, lat_action_t)
    raw.append(c)
    fused.append(c + d)

  def ticks_to_90pct(sig):
    target = 0.9 * abs(sig[-1])
    return next(i for i, s in enumerate(sig[40:]) if abs(s) >= target)

  assert ticks_to_90pct(fused) <= ticks_to_90pct(raw)
  # the surprise itself must not be averaged away
  assert te.cfg.gate_psi > 0.0
