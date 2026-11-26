#!/usr/bin/env python3
"""Add the drives still on the device to NavTrips: where each one started and ended, from the first and last GPS fix."""
import argparse
import os
from collections import defaultdict

from openpilot.common.params import Params
from openpilot.selfdrive.navigation.places import MAX_TRIPS, TRIPS
from openpilot.common.hardware.hw import Paths
from openpilot.tools.lib.logreader import LogReader

GPS = ("gpsLocationExternal", "gpsLocation")


def fixes(path: str):
  """(unix time, lat, lon) of each GPS fix in a log"""
  try:
    for m in LogReader(path, sort_by_time=True):
      if m.which() in GPS:
        gps = getattr(m, m.which())
        if gps.hasFix and gps.unixTimestampMillis > 0 and (gps.latitude or gps.longitude):
          yield gps.unixTimestampMillis / 1e3, gps.latitude, gps.longitude
  except Exception as e:
    print(f"  skipping {path}: {e}")


def route_trip(segments: list[str]) -> dict | None:
  first = last = None
  for seg in segments:
    first = next(fixes(seg), None)
    if first is not None:
      break
  for seg in reversed(segments):
    for fix in fixes(seg):
      last = fix
    if last is not None:
      break
  if first is None or last is None:
    return None
  return {"start": [first[1], first[2]], "end": [last[1], last[2]], "t": first[0], "t_end": last[0]}


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("--dir", default=Paths.log_root())
  parser.add_argument("--dry-run", action="store_true")
  args = parser.parse_args()

  routes: dict[str, list[tuple[int, str]]] = defaultdict(list)
  for name in os.listdir(args.dir):
    route, _, seg = name.rpartition("--")
    qlog = os.path.join(args.dir, name, "qlog.zst")
    if route and seg.isdigit() and os.path.isfile(qlog):
      routes[route].append((int(seg), qlog))

  params = Params()
  trips = params.get(TRIPS) or []
  known = {round(t["t"]) for t in trips}
  added = []
  for route, segs in sorted(routes.items()):
    trip = route_trip([path for _, path in sorted(segs)])
    if trip is None:
      print(f"{route}: no GPS fix")
    elif round(trip["t"]) in known:
      print(f"{route}: already in {TRIPS}")
    else:
      print(f"{route}: {trip['start']} -> {trip['end']}")
      added.append(trip)

  trips = sorted(trips + added, key=lambda t: t["t"])[-MAX_TRIPS:]
  print(f"{len(added)} drives added, {len(trips)} in total")
  if not args.dry_run:
    params.put(TRIPS, trips)


if __name__ == "__main__":
  main()
