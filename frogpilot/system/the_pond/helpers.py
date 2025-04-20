#!/usr/bin/env python3
import os
import re
import subprocess

from typing import List

from openpilot.system.loggerd.uploader import listdir_by_creation
from openpilot.tools.lib.route import SegmentName

SEGMENT_RE = re.compile(r"\d{4}-\d{2}-\d{2}--\d{2}-\d{2}-\d{2}--\d+$")

def ffmpeg_mp4_wrap_process_builder(filename: str) -> subprocess.Popen:
  extension = filename.rsplit(".", 1)[-1]
  command = [
    "ffmpeg",
    *(["-f", "hevc"] if extension == "hevc" else []),
    "-r", "20",
    "-i", filename,
    "-c", "copy",
    "-map", "0",
    *(["-vtag", "hvc1"] if extension == "hevc" else []),
    "-f", "mp4",
    "-movflags", "empty_moov",
    "-"
  ]
  return subprocess.Popen(command, stdout=subprocess.PIPE)

def get_all_segment_names(footage_path: str) -> List[SegmentName]:
  segment_names: List[SegmentName] = []
  for entry in listdir_by_creation(footage_path):
    if not SEGMENT_RE.fullmatch(entry):
      continue
    try:
      segment_names.append(segment_to_segment_name(footage_path, entry))
    except AssertionError:
      continue
  return segment_names

def get_routes_names(footage_path: str) -> List[str]:
  route_times = {segment.route_name.time_str for segment in get_all_segment_names(footage_path)}
  return sorted(route_times, reverse=True)

def get_segments_in_route(route_time_str: str, footage_path: str) -> List[str]:
  return [
    f"{segment.time_str}--{segment.segment_num}"
    for segment in get_all_segment_names(footage_path)
    if segment.time_str == route_time_str
  ]

def list_file(path: str) -> List[str]:
  return sorted(os.listdir(path), reverse=True)

def segment_to_segment_name(data_dir: str, segment: str) -> SegmentName:
  return SegmentName(os.path.join(data_dir, f"FakeDongleID1337|{segment}"))
