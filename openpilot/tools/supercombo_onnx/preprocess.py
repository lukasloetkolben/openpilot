"""NV12 -> supercombo `img` tensor, in numpy.

This is a straight transcription of the tinygrad graph in
`selfdrive/modeld/compile_modeld.py` (`warp_perspective_tinygrad`,
`make_frame_prepare`, `frames_to_tensor`). Onroad that graph is fused into the
model JIT; here it runs standalone so the ONNX file can be driven from any
frame source. Keep the two in sync -- this module is the reference for a port
to another language, so it is deliberately literal rather than idiomatic.
"""
import numpy as np

# Y-plane size the camera image is warped into. The model input is this
# subsampled by 2 in both axes: (6, 128, 256) per frame, two frames stacked.
MODEL_W, MODEL_H = 512, 256


def warp_nn(src_flat: np.ndarray, M_inv: np.ndarray, dst_wh: tuple[int, int],
            src_hw: tuple[int, int], stride_pad: int) -> np.ndarray:
  """Nearest-neighbour inverse perspective warp over a flat, possibly padded plane."""
  w_dst, h_dst = dst_wh
  h_src, w_src = src_hw

  x = np.tile(np.arange(w_dst, dtype=np.float32), h_dst)
  y = np.repeat(np.arange(h_dst, dtype=np.float32), w_dst)

  src_x = M_inv[0, 0] * x + M_inv[0, 1] * y + M_inv[0, 2]
  src_y = M_inv[1, 0] * x + M_inv[1, 1] * y + M_inv[1, 2]
  src_w = M_inv[2, 0] * x + M_inv[2, 1] * y + M_inv[2, 2]

  src_x = src_x / src_w
  src_y = src_y / src_w

  x_nn = np.clip(np.round(src_x), 0, w_src - 1).astype(np.int32)
  y_nn = np.clip(np.round(src_y), 0, h_src - 1).astype(np.int32)
  return src_flat[y_nn * (w_src + stride_pad) + x_nn]


# UV_SCALE @ M_inv @ UV_SCALE_INV simplifies to this elementwise scaling
_UV_SCALE = np.array([[1.0, 1.0, 0.5], [1.0, 1.0, 0.5], [2.0, 2.0, 1.0]], dtype=np.float32)


def frames_to_tensor(yuv: np.ndarray) -> np.ndarray:
  """(H*3//2, W) planar YUV420 -> (6, H//2, W//2), the model's per-frame channel order."""
  H = (yuv.shape[0] * 2) // 3
  W = yuv.shape[1]
  return np.concatenate([
    yuv[0:H:2, 0::2],
    yuv[1:H:2, 0::2],
    yuv[0:H:2, 1::2],
    yuv[1:H:2, 1::2],
    yuv[H:H + H // 4].reshape((H // 2, W // 2)),
    yuv[H + H // 4:H + H // 2].reshape((H // 2, W // 2)),
  ], axis=0).reshape((6, H // 2, W // 2))


def frame_prepare(nv12: np.ndarray, M_inv: np.ndarray, cam_w: int, cam_h: int,
                  stride: int | None = None, uv_offset: int | None = None) -> np.ndarray:
  """One NV12 frame + warp matrix -> (6, MODEL_H//2, MODEL_W//2) uint8.

  `stride`/`uv_offset` default to the unpadded layout ffmpeg produces. Onroad the
  VisionBuf is padded, so pass the values from `system.camerad.cameras.nv12_info`.
  """
  stride = cam_w if stride is None else stride
  uv_offset = stride * cam_h if uv_offset is None else uv_offset
  uv_height = cam_h // 2
  stride_pad = stride - cam_w

  M_inv = np.asarray(M_inv, dtype=np.float32)
  M_inv_uv = M_inv * _UV_SCALE

  uv = nv12[uv_offset:uv_offset + uv_height * stride].reshape(uv_height, stride)

  y = warp_nn(nv12[:cam_h * stride], M_inv, (MODEL_W, MODEL_H), (cam_h, cam_w), stride_pad)
  u = warp_nn(np.ascontiguousarray(uv[:cam_h // 2, 0:cam_w:2]).ravel(), M_inv_uv,
              (MODEL_W // 2, MODEL_H // 2), (cam_h // 2, cam_w // 2), 0)
  v = warp_nn(np.ascontiguousarray(uv[:cam_h // 2, 1:cam_w:2]).ravel(), M_inv_uv,
              (MODEL_W // 2, MODEL_H // 2), (cam_h // 2, cam_w // 2), 0)

  yuv = np.concatenate([y, u, v]).reshape((MODEL_H * 3 // 2, MODEL_W))
  return frames_to_tensor(yuv)
