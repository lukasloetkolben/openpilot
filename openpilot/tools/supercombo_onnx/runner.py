"""Drive driving_supercombo.onnx from onnxruntime, outside modeld.

Owns the four rolling input queues that live in the tinygrad JIT onroad
(`make_input_queues` / `shift_and_sample` in compile_modeld.py): the two image
histories, the feature history that carries the model's recurrent state, and the
desire history. Feed it frames in order at MODEL_RUN_FREQ and it produces the
same parsed output dict modeld publishes.
"""
from pathlib import Path

import numpy as np
import onnxruntime as ort

from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.modeld.get_model_metadata import make_metadata_dict
from openpilot.selfdrive.modeld.parse_model_outputs import Parser
from openpilot.tools.supercombo_onnx.preprocess import frame_prepare

DEFAULT_MODEL = Path(__file__).parents[2] / 'selfdrive/modeld/models/driving_supercombo.onnx'


class SupercomboRunner:
  def __init__(self, model_path=DEFAULT_MODEL, providers=None, intra_threads: int | None = None):
    self.model_path = Path(model_path)
    opts = ort.SessionOptions()
    if intra_threads is not None:
      opts.intra_op_num_threads = intra_threads
    self.sess = ort.InferenceSession(str(self.model_path), opts,
                                     providers=providers or ['CPUExecutionProvider'])
    self.dtypes = {i.name: np.dtype(_ORT_DTYPES[i.type]) for i in self.sess.get_inputs()}
    self.shapes = {i.name: tuple(i.shape) for i in self.sess.get_inputs()}
    # the output layout is embedded in the ONNX metadata, the same source modeld reads
    self.output_slices = make_metadata_dict(self.model_path)['output_slices']
    self.parser = Parser()

    self.frame_skip = ModelConstants.MODEL_RUN_FREQ // ModelConstants.MODEL_CONTEXT_FREQ
    img = self.shapes['img']                    # (1, 12, 128, 256)
    fb = self.shapes['features_buffer']         # (1, 24, 512)
    self.n_frames = img[1] // 6
    self.feat_dim = int(np.prod(fb[2:]))
    self._img_shape = (self.frame_skip * (self.n_frames - 1) + 1, 6, img[2], img[3])
    self._fb_shape = fb
    self.reset()

  def reset(self) -> None:
    self.img_q = np.zeros(self._img_shape, dtype=np.uint8)
    self.big_img_q = np.zeros(self._img_shape, dtype=np.uint8)
    self.feat_q = np.zeros((self.frame_skip * self._fb_shape[1], self._fb_shape[0], self.feat_dim), dtype=np.float32)
    self.desire_q = np.zeros((self.frame_skip * self.shapes['desire_pulse'][1], 1, ModelConstants.DESIRE_LEN), dtype=np.float32)
    self.prev_feat = np.zeros((self._fb_shape[0], self.feat_dim), dtype=np.float32)
    self.prev_desire = np.zeros(ModelConstants.DESIRE_LEN, dtype=np.float32)
    self.frames_seen = 0

  @staticmethod
  def _shift(buf: np.ndarray, new_val: np.ndarray) -> np.ndarray:
    buf[:-1] = buf[1:].copy()
    buf[-1] = new_val
    return buf

  def _sample_skip(self, buf: np.ndarray) -> np.ndarray:
    return buf[::self.frame_skip].reshape(1, -1, *buf.shape[2:])

  def _sample_desire(self, buf: np.ndarray) -> np.ndarray:
    return buf.reshape(-1, self.frame_skip, *buf.shape[1:]).max(1).reshape(1, -1, buf.shape[-1])

  def run(self, nv12_main, nv12_extra, tfm_main, tfm_extra, cam_w, cam_h,
          desire=None, traffic_convention=None, action_t=None,
          stride=None, uv_offset=None) -> dict[str, np.ndarray]:
    warped = frame_prepare(nv12_main, tfm_main, cam_w, cam_h, stride, uv_offset)
    warped_big = frame_prepare(nv12_extra, tfm_extra, cam_w, cam_h, stride, uv_offset)

    img = self._sample_skip(self._shift(self.img_q, warped))
    big_img = self._sample_skip(self._shift(self.big_img_q, warped_big))

    # desire is a pulse on the rising edge -- the model decides when the action is done
    desire = np.zeros(ModelConstants.DESIRE_LEN, dtype=np.float32) if desire is None else np.asarray(desire, dtype=np.float32)
    desire = desire.copy()
    desire[0] = 0
    pulse = np.where(desire - self.prev_desire > .99, desire, 0)
    self.prev_desire[:] = desire

    desire_buf = self._sample_desire(self._shift(self.desire_q, pulse.reshape(1, -1)))
    feat_buf = self._sample_skip(self._shift(self.feat_q, self.prev_feat)).reshape(self._fb_shape)

    if traffic_convention is None:
      traffic_convention = np.array([1., 0.], dtype=np.float32)  # LHD
    if action_t is None:
      action_t = np.array([0.2, 0.5], dtype=np.float32)

    inputs = {
      'img': img,
      'big_img': big_img,
      'features_buffer': feat_buf,
      'desire_pulse': desire_buf,
      'traffic_convention': np.asarray(traffic_convention, dtype=np.float32).reshape(self.shapes['traffic_convention']),
      'action_t': np.asarray(action_t, dtype=np.float32).reshape(self.shapes['action_t']),
    }
    inputs = {k: v.astype(self.dtypes[k], copy=False) for k, v in inputs.items()}

    raw = self.sess.run(None, inputs)[0].astype(np.float32)[0]
    self.prev_feat[:] = raw[self.output_slices['hidden_state']]
    self.frames_seen += 1

    parsed = self.parser.parse_outputs({k: raw[np.newaxis, v] for k, v in self.output_slices.items()})
    parsed['raw_pred'] = raw
    return parsed

  @property
  def warm(self) -> bool:
    """Outputs are only comparable to onroad once the recurrent history is full.

    The image history fills in 5 ticks, but features_buffer spans
    frame_skip * 24 ticks (~4.8 s at 20 Hz) and dominates.
    """
    return self.frames_seen >= len(self.feat_q)


_ORT_DTYPES = {
  'tensor(uint8)': np.uint8,
  'tensor(float16)': np.float16,
  'tensor(float)': np.float32,
}
