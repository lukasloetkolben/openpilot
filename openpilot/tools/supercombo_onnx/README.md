# supercombo, standalone

Runs `selfdrive/modeld/models/driving_supercombo.onnx` under onnxruntime, outside
`modeld`: no tinygrad, no msgq, no VisionIPC. Feed it NV12 frames and it produces the
same parsed output dict `modeld` publishes.

The point is to have the model's full input contract written out in one readable place.
Onroad, preprocessing is fused into the tinygrad JIT and the recurrent state lives inside
the graph, so neither is easy to read off or to reimplement elsewhere. `preprocess.py` and
`runner.py` are deliberately literal transcriptions of `compile_modeld.py`, intended as the
reference for a port to another runtime or language.

## The input contract

```
img                uint8   (1, 12, 128, 256)   2 frames x 6ch, 200 ms apart
big_img            uint8   (1, 12, 128, 256)   wide camera, same
features_buffer    fp16    (1, 24, 512)        recurrent, from the previous output
desire_pulse       fp16    (1, 25, 8)
traffic_convention fp16    (1, 2)
action_t           fp16    (1, 2)              [lat, long] delays
outputs            fp16    (1, 2576)           one flat vector
```

Four things are easy to get wrong:

- **`img` is not a resize.** The camera frame is inverse-warped, nearest-neighbour, by
  `get_warp_matrix(rpyCalib, intrinsics, bigmodel_frame)` into a 512x256 Y plane, then
  subsampled into 6 channels of 128x256. `big_img` uses the *wide* intrinsics and
  `bigmodel_frame=True`, so the two transforms differ.
- **It is recurrent.** The `hidden_state` slice of the output feeds back as the next tick's
  feature. Frames must be run in order at `MODEL_RUN_FREQ` (20 Hz); one skipped tick
  corrupts the state.
- **Only every 4th entry is used.** `frame_skip = MODEL_RUN_FREQ // MODEL_CONTEXT_FREQ = 4`.
  The queues run at 20 Hz but the model sees 5 Hz: images at t and t-4, features
  subsampled the same way, desire max-pooled over each group of 4.
- **The output layout is in the file.** `output_slices` is a base64 pickle in the ONNX
  `metadata_props`, so nothing needs hardcoded offsets.

## Validating against a real route

`run_route.py` rebuilds modeld's per-tick inputs from a log — calibration, intrinsics,
traffic convention, `action_t` from the logged `lateralDelay` and
`carParams.longitudinalActuatorDelay`, and desire from `DesireHelper` driven by our own
lane-change probability — so the only thing that differs from onroad is the runtime.

```
./run_route.py 44f710dd409f4ee6/0000003b--8e12ef104c --limit 250
```

Comparisons start after 96 ticks (~4.8 s), the depth of `features_buffer`; before that the
recurrent state has not converged and the output is not comparable to a mid-route log.

Measured on `44f710dd409f4ee6/0000003b--8e12ef104c/0`, tizi/ox03c10, 155 compared ticks:

| quantity | mean abs diff vs logged `modelV2` |
| --- | --- |
| `laneLineProbs` | 0.006 |
| `position` @ t=0.16 s | 22 mm x, 4 mm y |
| `position` @ t=2.50 s | 0.28 m x, 0.12 m y |
| `position` @ t=10.0 s | 4.24 m x, 1.71 m y |

The error is monotonic in range and negligible over the horizon anything actually consumes
(`lat_action_t` is ~0.4 s). That shape — near-exact vision outputs, error growing only with
extrapolation distance — is fp16 accumulation order, not a preprocessing mismatch. A wrong
warp or a wrong channel packing shows up as a large error at *every* index, including t=0.

## Running it

onnxruntime is not an openpilot dependency:

```
uv run --with onnxruntime python tools/supercombo_onnx/run_route.py <route>
```

~33 ms/tick on CPU on an M-series Mac, so faster than realtime at 20 Hz.

## Using it on other footage

`SupercomboRunner.run()` takes raw NV12 plus the two warp matrices, so any frame source
works. What it does *not* do is invent the inputs the model assumes:

- **Two cameras.** `big_img` is a ~120 deg wide camera. Feeding it the narrow frame runs, but
  the wide-derived features are then wrong and the output degrades.
- **Calibration.** `get_warp_matrix` needs real intrinsics and a real `rpyCalib`. Guessing
  the focal length tilts the whole model frame.
- **20 Hz, in order.** See above.
