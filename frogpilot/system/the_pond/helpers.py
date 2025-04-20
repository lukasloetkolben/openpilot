#!/usr/bin/env python3
import os
import re
import subprocess

from datetime import datetime, timedelta, timezone

from typing import List

from openpilot.system.loggerd.uploader import listdir_by_creation
from openpilot.tools.lib.logreader import LogReader
from openpilot.tools.lib.route import SegmentName

SEGMENT_RE = re.compile(r'^[0-9a-fA-F]{8}--[0-9a-fA-F]{10}--\d+$')

LOG_CANDIDATES = [
  "qlog",
  "qlog.bz2",
  "rlog",
  "rlog.bz2",
  "raw_log.bz2",
]

def ffmpeg_mp4_wrap_process_builder(filename: str) -> subprocess.Popen:
  is_raw_hevc = filename.rsplit(".", 1)[-1] == "hevc"

  command = [
    "ffmpeg",
    *(["-f", "hevc"] if is_raw_hevc else []),
    "-r", "20",
    "-i", filename,
    "-c", "copy",
    "-map", "0",
    *(["-vtag", "hvc1"] if is_raw_hevc else []),
    "-f", "mp4",
    "-movflags", "empty_moov",
    "-"
  ]

  return subprocess.Popen(command, stdout=subprocess.PIPE)

def get_all_segment_names(footage_path: str) -> List[SegmentName]:
  segment_names: List[SegmentName] = []
  entries = listdir_by_creation(footage_path)

  for entry in entries:
    if not SEGMENT_RE.fullmatch(entry):
      continue
    segment_names.append(segment_to_segment_name(footage_path, entry))
    continue
  return segment_names

def get_routes_names(footage_path: str) -> List[str]:
  segments = get_all_segment_names(footage_path)
  route_times = {segment.route_name.time_str for segment in segments}
  return sorted(route_times, reverse=True)

def get_route_start_time(log_file_path):
  if os.path.getsize(log_file_path) == 0:
    return None

  log_reader = LogReader(log_file_path)
  for msg in log_reader:
    if msg.which() == "initData":
      return datetime.fromtimestamp(msg.initData.wallTimeNanos / 1e9)

def get_segments_in_route(route_time_str: str, footage_path: str) -> List[str]:
  return [
    f"{segment.time_str}--{segment.segment_num}"
    for segment in get_all_segment_names(footage_path)
    if segment.time_str == route_time_str
  ]

def list_file(path: str) -> List[str]:
  return sorted(os.listdir(path), reverse=True)

def segment_to_segment_name(data_dir: str, segment: str) -> SegmentName:
  full_path = os.path.join(data_dir, f"FakeDongleID1337|{segment}")
  return SegmentName(full_path)
