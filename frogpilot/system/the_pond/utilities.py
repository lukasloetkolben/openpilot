#!/usr/bin/env python3
import json
import os
import subprocess

from datetime import datetime
from io import BytesIO
from pathlib import Path

from openpilot.common.conversions import Conversions as CV
from openpilot.common.params import Params
from openpilot.system.loggerd.config import get_available_bytes, get_used_bytes
from openpilot.system.loggerd.deleter import PRESERVE_ATTR_NAME, PRESERVE_ATTR_VALUE

from openpilot.frogpilot.common.frogpilot_variables import params, params_tracking

def format_git_date(raw_date: str):
  date_object = datetime.strptime(raw_date.split()[1], "%Y-%m-%d")

  day = date_object.day
  suffix = "th" if 11 <= day <= 13 else {1: "st", 2: "nd", 3: "rd"}.get(day % 10, "th")

  return date_object.strftime(f"%B {day}{suffix}, %Y")

def get_available_cameras(segment_path):
  segment_path = Path(segment_path)
  return [
    name for name, file in {
      "driver": "dcamera.hevc",
      "forward": "fcamera.hevc",
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

def get_repo_owner(git_normalized_origin):
  parts = git_normalized_origin.split('/')
  return parts[1] if len(parts) >= 2 else "unknown"

def get_video_duration(input_path):
  result = subprocess.run([
    "ffprobe", "-v", "error", "-show_entries", "format=duration",
    "-of", "default=noprint_wrappers=1:nokey=1", str(input_path)
  ], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, check=True)
  return float(result.stdout)

def has_preserve_attr(path: str):
  return PRESERVE_ATTR_NAME in os.listxattr(path) and os.getxattr(path, PRESERVE_ATTR_NAME) == PRESERVE_ATTR_VALUE

def run_ffmpeg(args):
  process = subprocess.Popen(["ffmpeg", "-hide_banner", "-loglevel", "error"] + args, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
  stdout, stderr = process.communicate()
  return stdout

def video_to_png(input_path, output_path):
  run_ffmpeg([
    "-i", str(input_path),
    "-ss", "2",
    "-vframes", "1",
    str(output_path)
  ])

def video_to_gif(input_path, output_path):
  sped_up_path = output_path.replace(".gif", ".mp4")

  run_ffmpeg(["-i", input_path, "-an", "-vf", "setpts=PTS/35", sped_up_path])
  run_ffmpeg(["-i", sped_up_path, "-loop", "0", output_path])

  os.remove(sped_up_path)
