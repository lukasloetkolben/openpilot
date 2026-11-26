"""Places for picking a destination on the device: where past drives started and ended, destinations picked before,
and Mapbox search. navd records the drives (TripRecorder), the UI does the rest."""
import math
import re
import time
from dataclasses import dataclass

import requests

from openpilot.cereal import log

TRIPS = "NavTrips"                    # [{"start": [lat, lon], "end": [lat, lon], "t": unix start, "t_end": unix end}]
RECENTS = "NavRecentDestinations"     # [{"lat", "lon", "name", "address", "t"}] newest first
NAMES = "NavPlaceNames"               # [{"lat", "lon", "name", "address"}] reverse geocoded once per place
MAX_TRIPS = 300
MAX_RECENTS = 20
MAX_NAMES = 200
SAME_PLACE_M = 150    # stops closer than this are the same place (parking spots around a building)
MIN_TRIP_M = 300      # moving the car around the driveway goes nowhere
MAPBOX_TOKEN_RE = re.compile(r"pk\.[A-Za-z0-9_-]+\.[A-Za-z0-9_-]+")
MAPBOX = "https://api.mapbox.com"


@dataclass
class Place:
  lat: float
  lon: float
  name: str = ""
  address: str = ""
  count: int = 0   # visits
  t: float = 0.0   # last time it was visited or picked


def distance_m(a: tuple[float, float], b: tuple[float, float]) -> float:
  """(lat, lon) pairs, equirectangular: exact enough at city scale"""
  lat = math.radians((a[0] + b[0]) / 2)
  dx = math.radians(b[1] - a[1]) * math.cos(lat)
  dy = math.radians(b[0] - a[0])
  return 6371e3 * math.hypot(dx, dy)


def is_mapbox_token(text: str) -> bool:
  return MAPBOX_TOKEN_RE.fullmatch(text.strip()) is not None


def _stops(trips) -> list[tuple[float, float, float]]:
  """(lat, lon, t) of each visit: every arrival, and a start only where the drive before didn't end (a gap in the
  history), so parking somewhere and driving off again is one visit"""
  stops = []
  prev_end = None
  for trip in sorted(trips, key=lambda t: t["t"]):
    start, end = tuple(trip["start"]), tuple(trip["end"])
    if distance_m(start, end) < MIN_TRIP_M:
      continue
    if prev_end is None or distance_m(prev_end, start) >= SAME_PLACE_M:
      stops.append((*start, trip["t"]))
    stops.append((*end, trip.get("t_end", trip["t"])))
    prev_end = end
  return stops


def _cluster(stops) -> list[Place]:
  """greedy, newest first: each stop joins the first place within SAME_PLACE_M, else starts one"""
  places: list[Place] = []
  for lat, lon, t in sorted(stops, key=lambda s: -s[2]):
    place = next((p for p in places if distance_m((p.lat, p.lon), (lat, lon)) < SAME_PLACE_M), None)
    if place is None:
      places.append(Place(lat, lon, count=1, t=t))
    else:
      place.lat += (lat - place.lat) / (place.count + 1)
      place.lon += (lon - place.lon) / (place.count + 1)
      place.count += 1
      place.t = max(place.t, t)
  return places


def name_of(names, lat: float, lon: float) -> tuple[str, str]:
  """the name you picked for the place, else the nearest address found for it"""
  near = [n for n in names if distance_m((n["lat"], n["lon"]), (lat, lon)) < SAME_PLACE_M]
  if not near:
    return "", ""
  n = min(near, key=lambda n: (not n.get("picked"), distance_m((n["lat"], n["lon"]), (lat, lon))))
  return n["name"], n.get("address", "")


def common_places(trips, names, n: int = 5) -> list[Place]:
  """the places drives start and end at most often, at least twice"""
  places = [p for p in _cluster(_stops(trips)) if p.count >= 2]
  places.sort(key=lambda p: (-p.count, -p.t))
  for p in places[:n]:
    p.name, p.address = name_of(names, p.lat, p.lon)
  return places[:n]


def recent_places(trips, recents, names, n: int = 4, exclude: list[Place] | None = None) -> list[Place]:
  """the last places driven to or picked, without repeats and without the ones in exclude"""
  candidates = [Place(r["lat"], r["lon"], r.get("name", ""), r.get("address", ""), t=r["t"]) for r in recents]
  candidates += [Place(*trip["end"], t=trip.get("t_end", trip["t"])) for trip in trips
                 if distance_m(tuple(trip["start"]), tuple(trip["end"])) >= MIN_TRIP_M]
  taken = list(exclude or [])
  out: list[Place] = []
  for p in sorted(candidates, key=lambda p: -p.t):
    if any(distance_m((p.lat, p.lon), (q.lat, q.lon)) < SAME_PLACE_M for q in taken):
      continue
    if not p.name:
      p.name, p.address = name_of(names, p.lat, p.lon)
    out.append(p)
    taken.append(p)
    if len(out) == n:
      break
  return out


def set_destination(params, place: Place):
  params.put("NavDestination", {"latitude": place.lat, "longitude": place.lon, "place_name": place.name})
  recents = [r for r in (params.get(RECENTS) or []) if distance_m((r["lat"], r["lon"]), (place.lat, place.lon)) >= SAME_PLACE_M]
  recents.insert(0, {"lat": place.lat, "lon": place.lon, "name": place.name, "address": place.address, "t": time.time()})  # noqa: TID251 dates outlive reboots
  params.put(RECENTS, recents[:MAX_RECENTS])
  if place.name:
    remember_name(params, place.lat, place.lon, place.name, place.address, picked=True)


def remember_name(params, lat: float, lon: float, name: str, address: str, picked: bool = False):
  names = [n for n in (params.get(NAMES) or []) if distance_m((n["lat"], n["lon"]), (lat, lon)) >= SAME_PLACE_M / 3]
  names.insert(0, {"lat": lat, "lon": lon, "name": name, "address": address, "picked": picked})
  params.put(NAMES, names[:MAX_NAMES])


def current_position(params) -> tuple[float, float] | None:
  """(lat, lon) of the last GPS fix, else where the last drive ended"""
  try:
    with log.Event.from_bytes(params.get("LastGPSPosition")) as evt:
      gps = evt.gpsLocationExternal
      if gps.latitude or gps.longitude:
        return gps.latitude, gps.longitude
  except Exception:
    pass
  trips = params.get(TRIPS) or []
  return tuple(trips[-1]["end"]) if trips else None


def language(params) -> str:
  lang = params.get("LanguageSetting") or "main_en"
  return lang.replace("main_", "").split("-", 1)[0]


def _short_address(text: str) -> str:
  """'65529 Waldems, Germany' -> '65529 Waldems'"""
  return text.split(",", maxsplit=1)[0].strip() if text else ""


def search(query: str, token: str, near: tuple[float, float] | None = None, lang: str = "en", limit: int = 6) -> list[Place]:
  """addresses and points of interest, closest to near first"""
  params = {"q": query, "limit": limit, "language": lang, "access_token": token}
  if near is not None:
    params["proximity"] = f"{near[1]},{near[0]}"
  resp = requests.get(f"{MAPBOX}/search/searchbox/v1/forward", params=params, timeout=10)
  resp.raise_for_status()
  places = []
  for f in resp.json().get("features", []):
    lon, lat = f["geometry"]["coordinates"]
    props = f["properties"]
    name = props.get("name") or props.get("full_address") or query
    places.append(Place(lat, lon, name, _short_address(props.get("place_formatted") or "")))
  return places


def reverse(lat: float, lon: float, token: str, lang: str = "en") -> tuple[str, str] | None:
  """(street and number, postcode and town) of the address at the point"""
  params = {"longitude": lon, "latitude": lat, "types": "address", "limit": 1, "language": lang, "access_token": token}
  resp = requests.get(f"{MAPBOX}/search/geocode/v6/reverse", params=params, timeout=10)
  resp.raise_for_status()
  features = resp.json().get("features", [])
  if not features:
    return None
  props = features[0]["properties"]
  return props.get("name", ""), _short_address(props.get("place_formatted", ""))


class TripRecorder:
  """Where this drive started and where it is now, kept in TRIPS so the last drive survives a power cut."""
  SAVE_PERIOD = 30  # s

  def __init__(self, params):
    self.params = params
    self.trip: dict | None = None
    self.last_save = 0.0

  def update(self, lat: float, lon: float, now: float | None = None):
    now = time.time() if now is None else now  # noqa: TID251 dates outlive reboots
    if self.trip is None:
      self.trip = {"start": [lat, lon], "end": [lat, lon], "t": now, "t_end": now}
      self.save(new=True)
      return
    self.trip["end"], self.trip["t_end"] = [lat, lon], now
    if now - self.last_save > self.SAVE_PERIOD:
      self.save()

  def save(self, new: bool = False):
    if self.trip is None:
      return
    trips = self.params.get(TRIPS) or []
    if not new and trips and trips[-1]["t"] == self.trip["t"]:
      trips[-1] = dict(self.trip)
    else:
      trips.append(dict(self.trip))
    self.params.put(TRIPS, trips[-MAX_TRIPS:])
    self.last_save = self.trip["t_end"]
