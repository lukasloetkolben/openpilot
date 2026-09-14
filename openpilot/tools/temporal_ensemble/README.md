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

## Does it help in curves? No, the sign is inverted

Segments 6:10 of the same route, picked by scanning all 166 qlogs for curvature at speed
(note `modelV2` has decimation `None` and is absent from qlogs, use `controlsState` to scan).
4800 ticks, 1600 of them with |curvature| in 0.005..0.012 at ~20 m/s, gate rate 4.90 %.

Members are used: zero rejections by the heading, lateral offset or coverage checks, mean
ensemble size 7.65 in curves, effective weighted age 0.19 s. The mechanism runs as designed.

But the signed disagreement between an old prediction and the current one at the same road
point, positive meaning the old one asks for *more* curvature into the turn, is negative and
grows monotonically with age:

| member age | mean signed dyaw | sd | mean weight |
| --- | --- | --- | --- |
| 0.0-0.1 s | -0.00010 rad | 0.00079 | 0.190 |
| 0.1-0.2 s | -0.00023 rad | 0.00149 | 0.139 |
| 0.2-0.3 s | -0.00052 rad | 0.00215 | 0.100 |
| 0.3-0.4 s | -0.00079 rad | 0.00259 | 0.077 |
| 0.4-0.6 s | -0.00118 rad | 0.00308 | 0.059 |

**Older predictions are systematically flatter, not sharper.** Section 1 of the brief assumes the
opposite: that a curve seen from 100 m in the narrow FOV is better understood than the same curve
at 20 m in the wide angle. The data inverts that. The net effect of the ensemble in curves is
`delta * sign(curvature)` = -8.5e-05 1/m, a 1 to 1.7 % *reduction* of the commanded curvature,
and the phase of the fused signal against the raw one is 0 ticks: no earlier turn in.

The cause is the one section 5 already names. Under arclength indexing, an old member's
contribution to the road point in front of us comes from the far end of its own horizon, and the
far end of a plan is flatter than reality, because the model regresses toward straight where it is
uncertain. That is a bias, so age weighting can attenuate it but never remove it, and averaging
cannot fix it. Curve cutting is therefore not addressed by this mechanism, and cannot be.

### The one constructive reading

The flattening is not noise: it is monotonic in age and repeatable, -0.00118 rad at 0.4-0.6 s.
That makes it a measurable, signed model bias. The useful move is the opposite of ensembling:
do not average *toward* the older, flatter prediction, extrapolate *away* from it, using the
measured age slope to estimate how much the current prediction is itself under curving. That is a
bias correction rather than a variance reduction, it needs its own validation on several routes
before it means anything, and it does not reuse the fusion machinery, only the projection and
re-indexing part of it.

## Would a driver feel it?

Same curvy segments, corrections converted with the route's logged vehicle parameters
(Rivian R1: wheelbase 3.08 m, steer ratio 15.01) into what reaches the wheel.

In curves (1604 ticks at 20.3 m/s), |correction| as a steering wheel angle and as a change in
lateral acceleration:

| | p50 | p90 | p99 | max |
| --- | --- | --- | --- | --- |
| steering wheel | 0.39 deg | 1.39 deg | 2.90 deg | 16.92 deg |
| lateral accel | 0.061 m/s^2 | 0.216 m/s^2 | 0.418 m/s^2 | 1.91 m/s^2 |

Exceedance rates in curves: 41.6 % of ticks above 0.5 deg, **18.8 % above 1 deg**, 4.5 % above
2 deg; 33.4 % above 0.1 m/s^2, **11.8 % above 0.2 m/s^2**. At 20 Hz that is several perceptible
nudges per second of curve driving. The p99 correction is 183 % of the ISO jerk headroom for one
model tick, so those get clipped by `clip_curvature` - the module would be fighting the rate
limiter rather than steering the car.

So the answer is not "no difference". It is **a difference you would feel, that buys nothing**:
the median is below the perception threshold, the tail is well above it, and the measured content
of that tail is noise plus the wrong signed 1 to 2 % flattening from the section above. Net jerk
moves by -0.2 % straight and -1.5 % curvy, so the smoothing gain and the injected restlessness
roughly cancel.

If the module were ever enabled, `max_curvature_delta` and `gate_psi` are far too loose for the
value delivered: the gate at 0.06 rad lets corrections of up to 6.4e-03 1/m through, which is the
16.9 deg outlier above. Capping at ~5e-04 1/m would keep every correction below the perception
threshold. But with the benefit measured at roughly zero, the correct setting is off.
