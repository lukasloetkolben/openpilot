from unittest.mock import MagicMock, patch

from openpilot.selfdrive.navigation import places
from openpilot.selfdrive.navigation.places import Place, TripRecorder, common_places, distance_m, is_mapbox_token, recent_places, set_destination

HOME = (50.23653, 8.31531)
WORK = (50.13759, 8.51163)
SHOP = (50.11890, 8.52700)
GYM = (50.26638, 8.27823)


class FakeParams:
  def __init__(self, **values):
    self.values = values

  def get(self, key):
    return self.values.get(key)

  def put(self, key, value):
    self.values[key] = value


def trip(start, end, t):
  return {"start": list(start), "end": list(end), "t": t, "t_end": t + 600}


def near(p, offset_m):
  return p[0] + offset_m / 111e3, p[1]


def test_distance():
  assert 50 < distance_m(HOME, near(HOME, 60)) < 70
  assert 13e3 < distance_m(HOME, WORK) < 18e3


def test_mapbox_token():
  assert is_mapbox_token("pk.eyJ1IjoibHVrYXMiLCJhIjoiY2xx.abc-DEF_123")
  assert is_mapbox_token("  pk.abc.def\n")
  assert not is_mapbox_token("sk.abc.def")  # secret tokens don't belong on the device
  assert not is_mapbox_token("pk.abc")
  assert not is_mapbox_token("LPA:1$rsp.example.com$CODE")


def test_common_places_by_count():
  trips = [trip(HOME, WORK, 1), trip(WORK, HOME, 2), trip(HOME, SHOP, 3), trip(near(SHOP, 80), near(HOME, 40), 4),
           trip(HOME, GYM, 5)]
  common = common_places(trips, [], n=5)
  assert [round(p.count) for p in common] == [5, 2, 2]  # home: 5 stops, shop and work twice, gym once is no habit
  assert distance_m((common[0].lat, common[0].lon), HOME) < 30
  assert distance_m((common[1].lat, common[1].lon), SHOP) < 60  # newer of the two-stop places first


def test_short_moves_are_no_places():
  trips = [trip(HOME, near(HOME, 100), t) for t in range(5)]
  assert common_places(trips, []) == []
  assert recent_places(trips, [], []) == []


def test_names_from_cache():
  names = [{"lat": near(WORK, 50)[0], "lon": WORK[1], "name": "Krautweide 24", "address": "65812 Bad Soden"}]
  common = common_places([trip(HOME, WORK, 1), trip(WORK, HOME, 2)], names)
  work = next(p for p in common if distance_m((p.lat, p.lon), WORK) < 100)
  assert (work.name, work.address) == ("Krautweide 24", "65812 Bad Soden")


def test_recent_places_newest_first_without_repeats():
  trips = [trip(HOME, WORK, 10), trip(WORK, HOME, 20), trip(HOME, near(WORK, 50), 30)]
  recents = [{"lat": GYM[0], "lon": GYM[1], "name": "Pflasterwiese 7", "address": "", "t": 625}]  # trips end at t + 600
  out = recent_places(trips, recents, [], n=4)
  assert [p.name for p in out] == ["", "Pflasterwiese 7", ""]
  assert distance_m((out[0].lat, out[0].lon), WORK) < 100 and distance_m((out[2].lat, out[2].lon), HOME) < 10


def test_recent_places_skip_excluded():
  trips = [trip(HOME, WORK, 10), trip(WORK, HOME, 20)]
  out = recent_places(trips, [], [], exclude=[Place(*HOME)])
  assert len(out) == 1 and distance_m((out[0].lat, out[0].lon), WORK) < 10


def test_set_destination():
  params = FakeParams(NavRecentDestinations=[{"lat": SHOP[0], "lon": SHOP[1], "name": "old", "address": "", "t": 1}])
  set_destination(params, Place(*near(SHOP, 30), "Main-Taunus-Zentrum", "Sulzbach"))
  assert params.values["NavDestination"]["place_name"] == "Main-Taunus-Zentrum"
  assert [r["name"] for r in params.values["NavRecentDestinations"]] == ["Main-Taunus-Zentrum"]  # replaces the old pick
  assert params.values["NavPlaceNames"][0]["name"] == "Main-Taunus-Zentrum"


def test_trip_recorder():
  params = FakeParams(NavTrips=[trip(WORK, HOME, 1)])
  rec = TripRecorder(params)
  rec.update(*HOME, now=1000)
  assert len(params.values["NavTrips"]) == 2  # the start is saved right away
  rec.update(*near(HOME, 500), now=1010)
  assert params.values["NavTrips"][-1]["end"] == list(HOME)  # not yet, saves every SAVE_PERIOD
  rec.update(*GYM, now=1000 + TripRecorder.SAVE_PERIOD + 1)
  assert params.values["NavTrips"][-1]["end"] == list(GYM)
  rec.update(*SHOP, now=1040)
  rec.save()  # navd exits
  assert len(params.values["NavTrips"]) == 2 and params.values["NavTrips"][-1]["end"] == list(SHOP)


def test_search_parses_features():
  resp = MagicMock()
  resp.json.return_value = {"features": [
    {"geometry": {"coordinates": [8.5273, 50.1183]},
     "properties": {"name": "Main-Taunus-Zentrum", "feature_type": "poi", "address": "Main-Taunus-Zentrum 1",
                    "place_formatted": "65843 Sulzbach, Germany"}},
    {"geometry": {"coordinates": [8.3153, 50.2365]},
     "properties": {"name": "Hauptstraße 58a", "feature_type": "address", "place_formatted": "65529 Waldems, Germany"}},
  ]}
  with patch.object(places.requests, "get", return_value=resp) as get:
    out = places.search("mtz", "pk.a.b", near=HOME, lang="de")
  assert get.call_args.kwargs["params"]["proximity"] == f"{HOME[1]},{HOME[0]}"
  assert [(p.name, p.address) for p in out] == [("Main-Taunus-Zentrum", "65843 Sulzbach"),
                                                ("Hauptstraße 58a", "65529 Waldems")]
  assert (out[1].lat, out[1].lon) == (50.2365, 8.3153)
