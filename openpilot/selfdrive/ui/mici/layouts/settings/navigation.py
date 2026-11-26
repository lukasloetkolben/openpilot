"""Pick a destination on the device: search, the places you drive to most, the last ones, and the Mapbox token."""
import threading
import time
from collections.abc import Callable

import numpy as np
import raylib_nb as rl

from openpilot.cereal import log
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.navigation import places
from openpilot.selfdrive.navigation.places import Place
from openpilot.selfdrive.ui.mici.layouts.settings.network.esim_ui import QRScannerDialog
from openpilot.selfdrive.ui.mici.widgets.button import BigButton, GreyBigButton
from openpilot.selfdrive.ui.mici.widgets.dialog import BigConfirmationDialog, BigDialog, BigInputDialog
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.widgets.scroller import NavScroller

COMMON_PLACES = 5
RECENT_PLACES = 4
TOKEN_DESCRIPTION = ("Search and routes use your Mapbox public token.\n" +
                     "Make a QR code of it on the token page and hold it in front of the driver camera.")


def _icon(size: int, inside: Callable[[np.ndarray, np.ndarray], np.ndarray]) -> rl.Texture:
  """white icon from a shape test on the unit square, 4x supersampled for smooth edges"""
  px = round(size * gui_app._scale)
  n = px * 4
  y, x = np.mgrid[0:n, 0:n]
  alpha = inside((x + 0.5) / n, (y + 0.5) / n).reshape(px, 4, px, 4).mean(axis=(1, 3))
  rgba = np.zeros((px, px, 4), dtype=np.uint8)
  rgba[..., :3] = 255
  rgba[..., 3] = np.round(alpha * 255).astype(np.uint8)
  texture = rl.load_texture_from_image(rl.Image(None, px, px, 1, rl.PixelFormat.PIXELFORMAT_UNCOMPRESSED_R8G8B8A8))
  rl.update_texture(texture, np.ascontiguousarray(rgba))
  rl.set_texture_filter(texture, rl.TextureFilter.TEXTURE_FILTER_BILINEAR)
  texture.width = texture.height = size
  return texture


def _arrow(x, y):
  """navigation arrow pointing up"""
  def side(ax, ay, bx, by):
    return (bx - ax) * (y - ay) - (by - ay) * (x - ax)
  tip, right, notch, left = (0.5, 0.06), (0.9, 0.94), (0.5, 0.72), (0.1, 0.94)
  right_half = (side(*tip, *right) >= 0) & (side(*right, *notch) >= 0) & (side(*notch, *tip) >= 0)
  left_half = (side(*tip, *notch) >= 0) & (side(*notch, *left) >= 0) & (side(*left, *tip) >= 0)
  return right_half | left_half


def _magnifier(x, y):
  r = np.hypot(x - 0.42, y - 0.42)
  # distance to the handle segment from (0.64, 0.64) to (0.9, 0.9)
  t = np.clip(((x - 0.64) + (y - 0.64)) / 2 / 0.26, 0, 1) * 0.26
  handle = np.hypot(x - 0.64 - t, y - 0.64 - t) < 0.075
  return ((r < 0.33) & (r > 0.22)) | handle


def nav_icon(size: int = 64) -> rl.Texture:
  return _icon(size, _arrow)


class PlaceButton(BigButton):
  def __init__(self, place: Place, value: str, on_pick: Callable[[Place], None]):
    super().__init__(place.name or "dropped pin", value, scroll=True)
    self.place = place
    self.set_click_callback(lambda: on_pick(self.place))


def _subtitle(place: Place, count: bool = False) -> str:
  """when or how often on top, the town below: long addresses get cut at the end, not the useful part"""
  first = f"{place.count} visits" if count else _ago(place.t) if place.t else ""
  second = place.address or f"{place.lat:.4f}, {place.lon:.4f}"
  return f"{first}\n{second}" if first else second


def _ago(t: float) -> str:
  days = (time.time() - t) / 86400  # noqa: TID251 dates outlive reboots
  if days < 1:
    return "today"
  return "yesterday" if days < 2 else f"{int(days)} days ago"


def _online() -> bool:
  return ui_state.sm["deviceState"].networkType != log.DeviceState.NetworkType.none


class SearchResultsLayout(NavScroller):
  """Mapbox results for one search, nearest first"""

  def __init__(self, query: str, on_pick: Callable[[Place], None]):
    super().__init__()
    self._on_pick = on_pick
    self._status = GreyBigButton("", "searching...")
    self._scroller.add_widget(self._status)
    self._results: list[Place] | None = None
    self._error = ""
    threading.Thread(target=self._search, args=(query,), daemon=True).start()

  def _search(self, query: str):
    params = ui_state.params
    try:
      self._results = places.search(query, params.get("MapboxToken") or "", places.current_position(params), places.language(params))
    except Exception:
      cloudlog.exception("navigation search failed")
      self._error = "search failed, check the internet connection"

  def _update_state(self):
    super()._update_state()
    if self._error:
      self._status.set_value(self._error)
      self._error = ""
    elif self._results is not None:
      results, self._results = self._results, None
      if not results:
        self._status.set_value("nothing found")
        return
      self._scroller.items.clear()
      self._scroller.add_widgets([PlaceButton(p, p.address, self._on_pick) for p in results])


class NavigationLayoutMici(NavScroller):
  def __init__(self):
    super().__init__()
    self._params = ui_state.params

    self._current_btn = BigButton("navigating", "", scroll=True)
    self._current_btn.set_click_callback(self._on_stop)
    self._search_btn = BigButton("search", "address or place", _icon(64, _magnifier))
    self._search_btn.set_click_callback(self._on_search)
    self._token_btn = BigButton("mapbox token", "", description=TOKEN_DESCRIPTION)
    self._token_btn.set_click_callback(self._on_scan_token)
    self._names_dirty = False
    self._needs_rebuild = False
    self._naming = False
    self._last_refresh = 0.0

  def show_event(self):
    super().show_event()
    self._rebuild()
    self._name_places()

  def _update_state(self):
    super()._update_state()
    if self._names_dirty or rl.get_time() - self._last_refresh > 2:
      self._names_dirty = False
      self._refresh_current()
      if self._naming is False and self._needs_rebuild:
        self._rebuild()

  # ***** list *****

  def _places(self) -> tuple[list[Place], list[Place]]:
    trips = self._params.get(places.TRIPS) or []
    names = self._params.get(places.NAMES) or []
    common = places.common_places(trips, names, COMMON_PLACES)
    recent = places.recent_places(trips, self._params.get(places.RECENTS) or [], names, RECENT_PLACES, exclude=common)
    return common, recent

  def _rebuild(self):
    self._needs_rebuild = False
    common, recent = self._places()
    self._refresh_current()
    has_token = bool(self._params.get("MapboxToken"))
    self._token_btn.set_value("saved" if has_token else "scan QR code")
    self._search_btn.set_enabled(has_token)

    items: list = [self._current_btn, self._search_btn]
    items += [PlaceButton(p, _subtitle(p, count=True), self._on_pick) for p in common]
    items += [PlaceButton(p, _subtitle(p), self._on_pick) for p in recent]
    items.append(self._token_btn)
    self._scroller.items.clear()
    self._scroller.add_widgets(items)

  def _refresh_current(self):
    self._last_refresh = rl.get_time()
    dest = self._params.get("NavDestination")
    self._current_btn.set_visible(dest is not None)
    if dest is not None:
      self._current_btn.set_text(dest.get("place_name") or "dropped pin")
      self._current_btn.set_value("navigating")

  # ***** names: reverse geocode places from drives once, in the background *****

  def _name_places(self):
    token = self._params.get("MapboxToken")
    if self._naming or not token or not _online():
      return
    unnamed = [p for p in sum(self._places(), []) if not p.name]
    if not unnamed:
      return
    self._naming = True
    threading.Thread(target=self._reverse_geocode, args=(unnamed, token), daemon=True).start()

  def _reverse_geocode(self, unnamed: list[Place], token: str):
    lang = places.language(self._params)
    try:
      for p in unnamed:
        found = places.reverse(p.lat, p.lon, token, lang)
        if found is not None:
          places.remember_name(self._params, p.lat, p.lon, *found)
    except Exception:
      cloudlog.exception("navigation reverse geocode failed")
    finally:
      self._naming = False
      self._needs_rebuild = True
      self._names_dirty = True

  # ***** actions *****

  def _on_pick(self, place: Place):
    places.set_destination(self._params, place)
    cloudlog.warning("navigation destination picked on device")
    gui_app.pop_widgets_to(self, self._rebuild)

  def _on_stop(self):
    def stop():
      self._params.remove("NavDestination")
      self._refresh_current()
    gui_app.push_widget(BigConfirmationDialog("slide to stop", gui_app.texture("icons_mici/settings/network/new/trash.png", 54, 64), stop))

  def _on_search(self):
    if not _online():
      gui_app.push_widget(BigDialog("", "Connect to the internet to search."))
      return
    gui_app.push_widget(BigInputDialog("where to?", text_validator=lambda text: bool(text.strip()),
                                       confirm_callback=lambda text: gui_app.push_widget(SearchResultsLayout(text.strip(), self._on_pick))))

  def _on_scan_token(self):
    gui_app.push_widget(QRScannerDialog(on_qr_detected=self._on_token_scanned, validator=places.is_mapbox_token,
                                        invalid_text="not a mapbox public token (pk.)"))

  def _on_token_scanned(self, token: str):
    self._params.put("MapboxToken", token.strip())
    cloudlog.warning("mapbox token scanned")
    self._rebuild()
    self._name_places()
    gui_app.push_widget(BigDialog("token saved", "Routes use it from the next drive."))
