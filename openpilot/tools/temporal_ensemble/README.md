# Temporal ensembling

The model predicts a multi second plan every 50 ms and only the very beginning of it is used.
This keeps the recent plans, brings them into the current ego frame, indexes them over travelled
distance and fuses them per road point. Implementation: `selfdrive/modeld/temporal_ensemble.py`.

## What the controller actually consumes

`modeld` publishes the waypoints (`modelV2.position`, `.orientation`, ...), but nothing outside
`modeld` and the UI reads them: `controlsd` consumes the scalar `modelV2.action.desiredCurvature`
(`selfdrive/controls/controlsd.py:126`). What feeds that scalar depends on which model runs,
and the two supercombos differ:

| model | `action` head | source of `desiredCurvature` |
| --- | --- | --- |
| `driving_supercombo.onnx` | no | the waypoints, via `get_curvature_from_plan` |
| `big_driving_supercombo.onnx` (chestnut) | yes, `slice(2062, 2066)` | the net's own scalar |

So on the small model the waypoints do drive lateral control, on the big model they do not.
A single mechanism covers both: a **correction term**, the fused yaw profile minus the current
one, evaluated at exactly the point and with exactly the scaling `get_curvature_from_plan` uses,
added to whatever the model asked for.

On the small model that is not an approximation. It is algebraically identical to substituting
the fused yaw profile and recomputing the curvature from scratch, because the conversion is
linear in `psi_target`. The instantaneous `psi_rate` term is deliberately left un-fused: it is a
t=0 quantity, and the current tick is the best informed about it. On the big model the correction
is the only available route, and it keeps the learned absolute value as the anchor. Either way
the natural zero makes the fallback exact.

## Offline first

```
./replay_ensemble.py <route>                # metrics
./replay_ensemble.py <route> --plot
./replay_ensemble.py <route> --pose both    # correlated pose error control
./replay_ensemble.py <route> --sigma age    # how much survives pure age weighting
```

Reports gate rate, mean ensemble size, correction magnitude, and lateral jerk split by
straight and curvy, plus the monotonicity of the position stds over distance. The run asserts
that the fallback parametrisation reproduces the recorded output exactly.

Note: the logged `modelV2` carries position stds but no orientation stds, so offline runs use
`yStd(s) / s` as the heading uncertainty proxy (`--sigma posy`) or drop it (`--sigma age`).
Onroad, `modeld` has the real yaw stds from `plan_stds`.

## Onroad

Off by default. Enable with the `TemporalEnsembleEnabled` param or `TEMPORAL_ENSEMBLE=1`.
The buffer is dropped on lane change intent, calibration change, pose jump, a gap in the model
output, a model swap, and disengage. Below `min_speed` the correction is passed through as zero.
`max_curvature_delta` caps the authority of the module; the curvature and rate limits in
`clip_curvature` and in the safety layer are untouched and still bound everything downstream.

## Measured result on a real route

`44f710dd409f4ee6/0000003b--8e12ef104c`, 2 x 8 segments, 9600 model ticks each, small model
(`modelV2.big` false on 100 % of ticks), reconstructed action time from the logged `lateralDelay`
(the reconstructed curvature correlates 0.992 with the recorded scalar, rms residual 2.2e-04).

Lateral jerk rms, straight / curvy, against a one line EMA on the same signal:

| signal | 20:28 straight | 20:28 curvy | 100:108 straight | lag |
| --- | --- | --- | --- | --- |
| plan curvature, no smoothing | 1.076 | 5.302 | 0.501 | 0 |
| + temporal ensemble | 1.073 (-0.2 %) | 5.223 (-1.5 %) | 0.404 (-19 %) | +1 tick |
| + EMA tau=0.13 | 0.749 (-30 %) | 5.160 (-2.7 %) | 0.295 (-41 %) | +2 ticks |

**The premise does not hold on this model.** Section 1 of the brief assumes the per tick errors are
independent, so that averaging cancels them. They are not: the lag 1 autocorrelation of the plan
curvature residual is **+0.98** on the mixed section and **+0.93** on the straight one. Consecutive
ticks are not independent draws, because the supercombo feeds its own `hidden_state` back in as
`prev_feat` and shares frame context, so it already integrates over time internally. There is very
little independent noise left for an external ensemble to average away.

A single first order filter beats the whole mechanism by roughly 2x on the smoothing metric, for
one line of code. Note also that this branch has `LAT_SMOOTH_SECONDS = 0.0` while the recorded
route was clearly logged with smoothing enabled (recorded jerk 0.61 against 1.08 reconstructed),
so the cheapest available win is that constant, not this module.

Per section 4 of the brief ("shows the effect is small, the task ends here, which would be a good
result") the code stays in the tree as a measured negative result and an A/B reference. It is off
by default. Before investing further, the control comparisons from section 6 (wide angle ablation,
modality dropout) should be measured against the same baseline.
