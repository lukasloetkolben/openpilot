#!/usr/bin/env python3
"""Fine-tuned big model: compiled on the device the first time the Chestnut is connected.

Drop the ONNX at TUNED_ONNX (with its sha256 in TUNED_SHA). This process compiles it for the Chestnut, keeps the
original pkl once as .orig and swaps the new one in. Until then modeld leaves the Chestnut alone and runs the small
model, so the two never share it. Roll back: delete TUNED_ONNX and copy the .orig back.
"""
import json
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path

from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.common.transformations.model import sbigmodel_fl
from openpilot.selfdrive.modeld.helpers import chestnut_present, modeld_pkl_path

BASEDIR = Path(__file__).resolve().parents[3]
TUNED_DIR = Path(os.environ.get("TUNED_MODEL_DIR", "/data/opft"))
TUNED_ONNX = TUNED_DIR / "tuned.onnx"
TUNED_SHA = TUNED_DIR / "tuned.onnx.sha256"
# optional {"sha256": "<onnx sha>", "wide_focal": 305.08}: wide view the tuned policy was trained on (default 455 = 59 deg)
TUNED_CFG = TUNED_DIR / "tuned.json"
BIG_PKL = modeld_pkl_path(chestnut=True)
INFO = BIG_PKL.with_name(BIG_PKL.name + ".info")  # "<onnx sha256> <pkl size>" of the installed tuned model
COMPILE_ENV = {"DEV": "USB+AMD:LLVM", "GMMU": "0", "AM_POWER_LIMIT": "100"}  # as modeld runs it


def _expected_sha() -> str | None:
  try:
    return TUNED_SHA.read_text().split()[0] if TUNED_ONNX.is_file() else None
  except (OSError, IndexError):
    return None


def installed() -> bool:
  """the big pkl is the compiled tuned model"""
  sha = _expected_sha()
  try:
    info_sha, size = INFO.read_text().split()
    return sha is not None and info_sha == sha and int(size) == BIG_PKL.stat().st_size
  except (OSError, ValueError):
    return False


def wide_focal() -> float:
  """wide model-frame focal of the installed tuned model; its config must name the same onnx, else stock 455"""
  try:
    cfg = json.loads(TUNED_CFG.read_text())
    fl = float(cfg["wide_focal"])
    if installed() and cfg["sha256"] == _expected_sha() and 200.0 <= fl <= 600.0:
      return fl
  except (OSError, ValueError, KeyError, TypeError):
    pass
  return sbigmodel_fl


def pending() -> bool:
  return _expected_sha() is not None and not installed()


def compile_tuned() -> bool:
  sha = _expected_sha()
  out = TUNED_DIR / "big_driving_tinygrad.pkl"
  cloudlog.warning(f"tuned model: compiling {TUNED_ONNX} ({sha})")
  st = time.monotonic()
  env = os.environ | COMPILE_ENV | {"PYTHONPATH": f"{BASEDIR}:{BASEDIR / 'tinygrad_repo'}"}
  cmd = [sys.executable, str(BASEDIR / "tinygrad_repo/examples/openpilot/compile_onnx.py"), str(TUNED_ONNX), str(out),
         "--out-of-band", "--benchmark-runs", "3"]
  proc = subprocess.run(cmd, env=env, cwd=BASEDIR, capture_output=True, text=True, check=False)
  if proc.returncode != 0:
    cloudlog.error(f"tuned model: compile failed ({proc.returncode}): {(proc.stdout + proc.stderr)[-2000:]}")
    return False

  orig = BIG_PKL.with_name(BIG_PKL.name + ".orig")
  if not orig.exists() and BIG_PKL.is_file():
    shutil.copy2(BIG_PKL, orig)
  shutil.move(out, BIG_PKL)
  INFO.write_text(f"{sha} {BIG_PKL.stat().st_size}\n")
  cloudlog.warning(f"tuned model: installed in {time.monotonic() - st:.0f} s")
  return True


def main():
  # the compile is long and CPU heavy: only idle CPU time, and the first thing the kernel kills when memory runs out
  os.sched_setscheduler(0, os.SCHED_IDLE, os.sched_param(0))
  Path("/proc/self/oom_score_adj").write_text("1000")
  params = Params()
  failures = 0
  while True:
    if pending() and chestnut_present() and failures < 3:
      params.put_bool("TunedModelCompiling", True)
      ok = compile_tuned()
      params.put_bool("TunedModelCompiling", False)
      failures = 0 if ok else failures + 1
      if ok and not params.get_bool("IsOffroad") and not params.get_bool("IsEngaged"):
        params.put_bool("OnroadCycleRequested", True)  # modeld picks up the big model now, not on the next drive
    time.sleep(10)


if __name__ == "__main__":
  main()
