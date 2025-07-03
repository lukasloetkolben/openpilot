#!/usr/bin/env python3
import os
import re
import subprocess

from typing import List

from openpilot.system.loggerd.uploader import listdir_by_creation
from openpilot.tools.lib.route import SegmentName

SEGMENT_RE = re.compile(r'^[0-9a-fA-F]{8}--[0-9a-fA-F]{10}--\d+$')

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
  try:
    entries = listdir_by_creation(footage_path)
    print(f"[get_all_segment_names] entries in {footage_path}: {entries}")
  except Exception as e:
    print(f"[get_all_segment_names] Failed to list {footage_path}")
    traceback.print_exc()
    return segment_names

  for entry in entries:
    print(f"[get_all_segment_names] Checking entry: {entry}")
    if not SEGMENT_RE.fullmatch(entry):
      print(f"[get_all_segment_names] Rejected by SEGMENT_RE: {entry}")
      continue
    try:
      seg = segment_to_segment_name(footage_path, entry)
      print(f"[get_all_segment_names] Parsed segment: {seg}")
      segment_names.append(seg)
    except AssertionError:
      print(f"[get_all_segment_names] AssertionError on {entry}")
      continue
    except Exception as e:
      print(f"[get_all_segment_names] Unexpected error on {entry}")
      traceback.print_exc()
      continue
  return segment_names

def get_routes_names(footage_path: str) -> List[str]:
  segments = get_all_segment_names(footage_path)
  print(f"[get_routes_names] Segments from {footage_path}: {[str(s) for s in segments]}")
  route_times = {segment.route_name.time_str for segment in segments}
  print(f"[get_routes_names] route_times: {route_times}")
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
  full_path = os.path.join(data_dir, f"FakeDongleID1337|{segment}")
  print(f"[segment_to_segment_name] Full path for segment: {full_path}")
  return SegmentName(full_path)
