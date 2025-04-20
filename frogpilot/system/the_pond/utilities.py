#!/usr/bin/env python3
import json
import subprocess

from datetime import datetime
from pathlib import Path

from openpilot.common.conversions import Conversions as CV
from openpilot.common.params import Params
from openpilot.system.loggerd.config import get_available_bytes, get_used_bytes

from openpilot.frogpilot.common.frogpilot_variables import params, params_tracking

def format_git_date(raw_date: str) -> str:
  date_object = datetime.strptime(raw_date.split()[1], "%Y-%m-%d")

  day = date_object.day
  suffix = "th" if 11 <= day <= 13 else {1: "st", 2: "nd", 3: "rd"}.get(day % 10, "th")

  return date_object.strftime(f"%B {day}{suffix}, %Y")

def get_available_cameras(segment_path) -> list:
  segment_path = Path(segment_path)
  return [
    name for name, file in {
      "driver": "dcamera.hevc",
      "forward": "qcamera.ts",
      "wide": "ecamera.hevc"
    }.items() if (segment_path / file).exists()
  ]

def get_disk_usage():
  free = get_available_bytes()
  used = get_used_bytes()
  total = used + free

  def to_gb(b):
    return f"{b // (2**30)} GB"

  return [{
    "free": to_gb(free),
    "size": to_gb(total),
    "used": to_gb(used),
    "usedPercentage": f"{(used / total) * 100:.2f}%"
  }]

def get_drive_stats():
  stats = json.loads(params.get("ApiCache_DriveStats", encoding="utf-8") or "{}")

  is_metric = params.get_bool("IsMetric")
  conversion = 1 if is_metric else CV.KPH_TO_MPH
  unit = "kilometers" if is_metric else "miles"

  def process(timeframe):
    data = stats.get(timeframe, {})
    return {
      "distance": data.get("distance", 0) * conversion,
      "drives": data.get("routes", 0),
      "hours": data.get("minutes", 0) / 60,
      "unit": unit
    }

  stats["all"] = process("all")
  stats["week"] = process("week")
  stats["frogpilot"] = {
    "distance": params_tracking.get_int("FrogPilotKilometers") * conversion,
    "hours": params_tracking.get_int("FrogPilotMinutes") / 60,
    "drives": params_tracking.get_int("FrogPilotDrives"),
    "unit": unit
  }

  return stats

def get_repo_owner(git_normalized_origin) -> str:
  parts = git_normalized_origin.split('/')
  return parts[1] if len(parts) >= 2 else "unknown"

def get_video_duration(input_path) -> float:
  result = subprocess.run([
    "ffprobe", "-v", "error", "-show_entries", "format=duration",
    "-of", "default=noprint_wrappers=1:nokey=1", str(input_path)
  ], stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
  return float(result.stdout)

def run_ffmpeg(args) -> None:
  subprocess.run(["ffmpeg", "-hide_banner", "-loglevel", "error", "-y"] + args)

def video_to_gif(input_path, output_path) -> None:
  output_path = Path(output_path)
  if output_path.exists():
    return

  sped_up = output_path.with_suffix(".mp4")
  run_ffmpeg(["-i", str(input_path), "-an", "-vf", "setpts=PTS/35", str(sped_up)])
  run_ffmpeg(["-i", str(sped_up), "-loop", "0", str(output_path)])
  sped_up.unlink()

def video_to_png(input_path, output_path) -> None:
  output_path = Path(output_path)
  if not output_path.exists():
    run_ffmpeg(["-i", str(input_path), "-ss", "2", "-vframes", "1", str(output_path)])
