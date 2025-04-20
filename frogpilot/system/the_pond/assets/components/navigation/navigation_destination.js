import { html, reactive } from "https://esm.sh/@arrow-js/core"
import {
  addRouteToMap,
  formatMetersToHuman,
  formatSecondsToHuman,
  getCoordinatesFromSearch,
  getRoutes,
  removeRouteFromMap,
  getOrdinalSuffix
} from "./navigation_utils.js"

function sha1hex (str) {
  const rot = (v, s) => (v << s) | (v >>> (32 - s))
  const bytes = new TextEncoder().encode(str)
  const words = []
  for (let i = 0; i < bytes.length; i++) {
    words[i >> 2] |= bytes[i] << ((3 - (i & 3)) << 3)
  }
  const bitLen = bytes.length << 3
  words[(bitLen >> 5)] |= 0x80 << (24 - (bitLen & 31))
  words[((bitLen + 64 >> 9) << 4) + 15] = bitLen
  let [a, b, c, d, e] = [0x67452301, 0xefcdab89, 0x98badcfe, 0x10325476, 0xc3d2e1f0]
  const w = new Array(80)
  for (let i = 0; i < words.length; i += 16) {
    for (let t = 0; t < 16; t++) w[t] = words[i + t] | 0
    for (let t = 16; t < 80; t++) w[t] = rot(w[t - 3] ^ w[t - 8] ^ w[t - 14] ^ w[t - 16], 1)
    let [aa, bb, cc, dd, ee] = [a, b, c, d, e]
    for (let t = 0; t < 80; t++) {
      const k = t < 20 ? 0x5a827999 : t < 40 ? 0x6ed9eba1 : t < 60 ? 0x8f1bbcdc : 0xca62c1d6
      const f = t < 20 ? (bb & cc) | (~bb & dd)
              : t < 40 ? bb ^ cc ^ dd
              : t < 60 ? (bb & cc) | (bb & dd) | (cc & dd)
              : bb ^ cc ^ dd
      const tmp = (rot(aa, 5) + f + ee + k + w[t]) >>> 0
      ee = dd; dd = cc; cc = rot(bb, 30) >>> 0; bb = aa; aa = tmp
    }
    a = (a + aa) >>> 0; b = (b + bb) >>> 0; c = (c + cc) >>> 0
    d = (d + dd) >>> 0; e = (e + ee) >>> 0
  }
  return [a, b, c, d, e].map(x => x.toString(16).padStart(8, '0')).join('')
}

async function geometryHashFromRoute (route) {
  const flat = route.geometry.coordinates.flat().join(',')
  if (crypto?.subtle?.digest && window.isSecureContext) {
    const buf = await crypto.subtle.digest('SHA-1', new TextEncoder().encode(flat))
    return [...new Uint8Array(buf)].map(b => b.toString(16).padStart(2, '0')).join('')
  }
  return sha1hex(flat)
}

export function NavDestination () {
  let map
  let destinationMarker

  const state = reactive({
    mapboxPublic      : undefined,
    mapboxSecret      : undefined,
    amap1Key          : undefined,
    amap2Key          : undefined,
    initialized       : false,
    lastPosition      : undefined,
    destination       : undefined,
    selectedRoute     : null,
    confirmedRoute    : null,
    suggestions       : "[]",
    previousDestinations: "[]",
    missingKeys       : null,
    canToggleProvider : false,
    searchProvider    : "mapbox",
    isMetric          : true,
    favoritesCount : 0,
    favoritesVisible: false,
    confirmedRouteRefresh: 0,
    loadingRoute: false,
  })

  const searchFieldState = reactive({ value: "" })

  function areRoutesEqual (a, b) {
    return a?.routeHash && b?.routeHash && a.routeHash === b.routeHash
  }

  async function getNavigationData () {
    const res  = await fetch("/api/navigation")
    const data = await res.json()

    state.mapboxPublic = data.mapboxPublic.trim()
    state.mapboxSecret = data.mapboxSecret.trim()
    state.amap1Key = data.amap1Key?.trim() || ""
    state.amap2Key = data.amap2Key?.trim() || ""
    state.isMetric = data.isMetric ?? true

    const hasMapbox = !!state.mapboxPublic && !!state.mapboxSecret
    const hasAMap   = !!state.amap1Key && !!state.amap2Key

    state.missingKeys = !hasMapbox
    state.canToggleProvider = hasMapbox && hasAMap
    state.searchProvider = hasMapbox ? "mapbox" : ""

    if (state.missingKeys) return

    state.lastPosition = {
      latitude : parseFloat(data.lastPosition.latitude),
      longitude: parseFloat(data.lastPosition.longitude),
    }

    try { state.destination = JSON.parse(data.destination) } catch {}
    try {
      const prev = JSON.parse(data.previousDestinations)
      state.previousDestinations = prev.map(d => ({ name: d.place_name }))
      state.suggestions = JSON.stringify(state.previousDestinations)
    } catch {}
    setupMap()
    loadFavoritesAlphabetically()
  }

  const setupMap = async () => {
    if (!state.mapboxPublic || state.initialized) return

    const container = document.getElementById('map')
    if (!container) {
      requestAnimationFrame(setupMap)
      return
    }

    state.initialized = true
    mapboxgl.accessToken = state.mapboxPublic

    map = new mapboxgl.Map({
      container,
      center: [state.lastPosition.longitude, state.lastPosition.latitude],
      zoom: 15,
      pitch: 45,
      speed: 1,
      curve: 1,
      attributionControl: false,
      logoPosition: 'bottom-right',
      style: 'mapbox://styles/frogsgomoo/cmcfv151j000o01rcdxebhl76',
    })

    destinationMarker = new mapboxgl.Marker()

    new mapboxgl.Marker()
      .setLngLat([state.lastPosition.longitude, state.lastPosition.latitude])
      .addTo(map)

    map.on("style.load", async () => {
      const layers = map.getStyle().layers
      const labelLayer = layers.find(l => l.type === "symbol" && l.layout["text-field"]).id

      map.addLayer({
        id: "add-3d-buildings",
        source: "composite",
        "source-layer": "building",
        filter: ["==", "extrude", "true"],
        type: "fill-extrusion",
        minzoom: 15,
        paint: {
          "fill-extrusion-color": "#aaa",
          "fill-extrusion-height": ["interpolate", ["linear"], ["zoom"], 15, 0, 15.05, ["get", "height"]],
          "fill-extrusion-base" : ["interpolate", ["linear"], ["zoom"], 15, 0, 15.05, ["get", "min_height"]],
          "fill-extrusion-opacity": 0.6,
        },
      }, labelLayer)

      if (state.destination) {
        const coords = [state.destination.longitude, state.destination.latitude]
        destinationMarker.setLngLat(coords).addTo(map)

        const routes = await getRoutes(
          `${state.lastPosition.longitude},${state.lastPosition.latitude}`,
          `${coords[0]},${coords[1]}`,
          state.mapboxPublic
        )

        removeRouteFromMap(map)

        const active = localStorage.getItem('activeRouteId')
        const last = localStorage.getItem('lastRouteId')
        const remembered = active ?? last ?? 'main'

        addRouteToMap(
          map,
          routes,
          [state.lastPosition.longitude, state.lastPosition.latitude],
          coords,
          async (route, routeId) => {
            const routeHash = await geometryHashFromRoute(route)
            state.selectedRoute = {
              name: state.destination.name,
              duration: route.duration,
              distance: route.distance,
              destinationCoordinates: coords,
              startingCoordinates: [state.lastPosition.longitude, state.lastPosition.latitude],
              routeId,
              routeHash,
              steps: route?.legs?.[0]?.steps || []
            }
            localStorage.setItem('lastRouteId', routeId)
          },
          state.isMetric,
          remembered
        )

        const picked = routes.find((r, i) => {
          const rid = i === 0 ? 'main' : `alt-${i}`
          return rid === remembered
        }) ?? routes[0]

        const routeHash = await geometryHashFromRoute(picked)
        const defaultRoute = {
          name: state.destination.name,
          duration: picked.duration,
          distance: picked.distance,
          destinationCoordinates: coords,
          startingCoordinates: [state.lastPosition.longitude, state.lastPosition.latitude],
          routeId: remembered,
          routeHash,
          steps: picked?.legs?.[0]?.steps || []
        }

        state.selectedRoute = defaultRoute
        state.confirmedRoute = defaultRoute

        map.flyTo({
          center: [state.lastPosition.longitude, state.lastPosition.latitude],
          zoom: 18,
          pitch: 45,
          speed: 1,
          curve: 1
        })
      }
    })
  }

  async function searchInput (e) {
    const newVal = e.target.value.trim()
    searchFieldState.value = e.target.value
    clearTimeout(window.searchTimeout)

    window.searchTimeout = setTimeout(async () => {
      const val = newVal
      if (val.length < 3) {
        if (val.length === 0) {
          state.suggestions = "[]"
        }
        return
      }

      state.selectedRoute = null
      state.confirmedRoute = null
      state.suggestions = "[]"

      if (state.searchProvider === "mapbox") {
        const prox = `${state.lastPosition.longitude},${state.lastPosition.latitude}`
        const params = new URLSearchParams({ proximity: prox, access_token: state.mapboxPublic, session_token: "sfsf", q: val, limit: 4 })
        const res = await fetch(`https://api.mapbox.com/search/searchbox/v1/suggest?${params.toString()}`)
        const data = await res.json()
        state.suggestions = JSON.stringify(data.suggestions)
      } else {
        const auto = new AMap.Autocomplete({ city: "auto" })
        auto.search(val, (status, result) => {
          if (status === "complete" && result.tips) {
            state.suggestions = JSON.stringify(result.tips)
          }
        })
      }
    }, 800)
  }

  const sessionToken = crypto.randomUUID?.() || Math.random().toString(36).slice(2)

  function isRouteFavorited(route, favorites) {
    return favorites.some(fav =>
      fav.latitude === route.destinationCoordinates[1] &&
      fav.longitude === route.destinationCoordinates[0] &&
      fav.routeId === route.routeId
    )
  }

  async function loadFavoritesAlphabetically () {
    try {
      const res = await fetch("/api/navigation/favorite")
      const json = await res.json()
      const sorted = json.favorites.sort((a, b) => (a.name || '').localeCompare(b.name || ''))

      state.favoritesCount = sorted.length
      state.favoriteRoutes = sorted

      if (state.favoritesVisible) {
        state.suggestions = JSON.stringify(sorted)
      }

      return sorted
    } catch (err) {
      console.error("Failed to load favorites", err)
      showSnackbar("Failed to load favorites...")
      return []
    }
  }

  async function editFavoriteName (fav, newName) {
    try {
      handleFavoritesClick()

      showSnackbar("Updating favorite...")

      await fetch("/api/navigation/favorite", {
        method : "DELETE",
        headers: { "Content-Type": "application/json" },
        body   : JSON.stringify(fav)
      })

      const payload = {
        name      : newName,
        longitude : fav.longitude,
        latitude  : fav.latitude,
        routeId   : fav.routeId
      }

      await fetch("/api/navigation/favorite", {
        method : "POST",
        headers: { "Content-Type": "application/json" },
        body   : JSON.stringify(payload)
      })

      await handleFavoritesClick()
    } catch (err) {
      console.error("Failed to edit favorite", err)
      showSnackbar("Failed to edit favorite…")
      if (!state.favoritesVisible) {
        await handleFavoritesClick()
      }
    }
  }

  async function removeFavorite({ name, latitude, longitude, routeId }) {
    try {
      await fetch("/api/navigation/favorite", {
        method: "DELETE",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ name, latitude, longitude, routeId })
      })
      await loadFavoritesAlphabetically()
      showSnackbar("Favorite removed!")
    } catch (err) {
      console.error("Failed to remove favorite", err)
      showSnackbar("Failed to remove favorite...")
    }
  }

  async function selectSuggestion(sugg) {
    const label = sugg.full_address || sugg.name || sugg.address || "Unnamed Location"
    let coords

    state.selectedRoute = null
    state.confirmedRoute = null
    state.loadingRoute = true

    try {
      if (state.searchProvider === 'mapbox') {
        if (sugg.geometry && Array.isArray(sugg.geometry.coordinates)) {
          coords = sugg.geometry.coordinates
        } else if (sugg.mapbox_id) {
          const url = new URL(`https://api.mapbox.com/search/searchbox/v1/retrieve/${encodeURIComponent(sugg.mapbox_id)}`)
          url.searchParams.set('access_token', state.mapboxPublic)
          url.searchParams.set('session_token', sessionToken)
          const ret = await fetch(url)
          const retJson = await ret.json()
          coords = retJson.features[0].geometry.coordinates
        } else {
          coords = await getCoordinatesFromSearch(label, state.mapboxPublic)
        }
      } else {
        coords = [sugg.location.lng, sugg.location.lat]
      }

      const inputEl = document.getElementById('search-field')
      if (inputEl) inputEl.value = label

      destinationMarker.setLngLat(coords).addTo(map)

      const routes = await getRoutes(
        `${state.lastPosition.longitude},${state.lastPosition.latitude}`,
        `${coords[0]},${coords[1]}`,
        state.mapboxPublic
      )

      removeRouteFromMap(map)

      const route = routes[0]
      const routeHash = await geometryHashFromRoute(route)
      const selected = {
        name: label,
        duration: route.duration,
        distance: route.distance,
        destinationCoordinates: coords,
        startingCoordinates: [state.lastPosition.longitude, state.lastPosition.latitude],
        routeId: 'main',
        routeHash,
        steps: route?.legs?.[0]?.steps || []
      }

      state.selectedRoute = selected
      localStorage.setItem('lastRouteId', 'main')

      addRouteToMap(
        map,
        routes,
        [state.lastPosition.longitude, state.lastPosition.latitude],
        coords,
        () => {},
        state.isMetric,
        'main'
      )

      state.suggestions = '[]'
    } catch (err) {
      console.error("Failed to calculate route", err)
      showSnackbar("Failed to calculate route...")
    } finally {
      state.loadingRoute = false
    }
  }

  getNavigationData()

  async function handleFavoritesClick () {
    if (state.favoritesVisible) {
      state.suggestions = '[]'
      state.favoritesVisible = false
      return
    }

    searchFieldState.value = ''
    state.selectedRoute = null
    state.confirmedRoute = null

    const sorted = await loadFavoritesAlphabetically()
    state.suggestions = JSON.stringify(sorted)
    state.favoritesVisible = true
  }

  async function handleSearchKey(e) {
    if (e.key === "Enter") {
      clearTimeout(window.searchTimeout)

      const val = e.target.value.trim()
      searchFieldState.value = e.target.value

      if (val.length < 3) {
        if (val.length === 0) state.suggestions = "[]"
        return
      }

      state.selectedRoute = null
      state.confirmedRoute = null
      state.suggestions = "[]"

      if (state.searchProvider === "mapbox") {
        const prox = `${state.lastPosition.longitude},${state.lastPosition.latitude}`
        const params = new URLSearchParams({
          proximity: prox,
          access_token: state.mapboxPublic,
          session_token: "sfsf",
          q: val,
          limit: 4
        })
        const res = await fetch(`https://api.mapbox.com/search/searchbox/v1/suggest?${params}`)
        const data = await res.json()
        state.suggestions = JSON.stringify(data.suggestions)
      } else {
        const auto = new AMap.Autocomplete({ city: "auto" })
        auto.search(val, (status, result) => {
          if (status === "complete" && result.tips) {
            state.suggestions = JSON.stringify(result.tips)
          }
        })
      }
    }
  }

  return html`
    <div class="navigation-container">
      ${() => {
        if (state.missingKeys === null) {
          return html``
        }

        return state.missingKeys
          ? html`
              <section class="keys-required-wrapper">
                <div class="keys-required-widget">
                  <div class="keys-required-title">Mapbox Keys Required</div>
                  <p class="keys-required-text">
                    You must set both your public and secret Mapbox keys before using navigation features.
                  </p>
                  <a href="/navigation_keys" class="keys-required-button">Go to "Manage Keys"</a>
                </div>
              </section>

            `
          : html`
              <div class="map-wrapper">
                <div class="search-wrapper">
                  <div class="search-controls">
                    <input
                      autocomplete="off"
                      id="search-field"
                      placeholder="Search here"
                      value="${() => searchFieldState.value}"
                      @input="${searchInput}"
                      @keydown="${handleSearchKey}"
                    />

                    ${() =>
                      state.favoritesCount > 0
                        ? html`
                            <button class="favorites-toggle-button" @click="${handleFavoritesClick}">
                              ❤️ Favorites
                            </button>
                          `
                        : ''}

                    ${() =>
                      state.canToggleProvider
                        ? html`
                            <div class="search-provider-toggle">
                              <button
                                class="${() => (state.searchProvider === 'amap' ? 'active' : '')}"
                                @click="${() => {
                                  state.searchProvider = 'amap'
                                  state.suggestions = '[]'
                                }}"
                              >
                                AMap
                              </button>
                              <button
                                class="${() => (state.searchProvider === 'mapbox' ? 'active' : '')}"
                                @click="${() => {
                                  state.searchProvider = 'mapbox'
                                  state.suggestions = '[]'
                                }}"
                              >
                                Mapbox
                              </button>
                            </div>
                          `
                        : ''}
                  </div>

                  <div id="infobox">
                    ${() => {
                      if (state.loadingRoute) {
                        return html`
                          <div class="navigation-summary-widget loading-status">
                            <span class="spinner"></span> Calculating route...
                          </div>
                        `
                      } else if (state.selectedRoute) {
                        return NavigationDestination({
                          ...state.selectedRoute,
                          isFavorited: isRouteFavorited(state.selectedRoute, state.favoriteRoutes),
                          isConfirmed: () => areRoutesEqual(state.selectedRoute, state.confirmedRoute),
                          map,
                          isMetric: state.isMetric,
                          cancelNavigationFn: () => {
                            state.selectedRoute = null
                            state.confirmedRoute = null
                            state.suggestions = state.previousDestinations
                            destinationMarker.remove()
                          },
                          onConfirm: () => {
                            state.confirmedRoute = JSON.parse(JSON.stringify(state.selectedRoute))
                            state.confirmedRouteRefresh = Math.random()
                          },
                          loadFavorites: loadFavoritesAlphabetically,
                          removeFavorite
                        }, state.confirmedRouteRefresh)
                      } else if (JSON.parse(state.suggestions).length > 0) {
                        return SearchSuggestions({
                          suggestions: JSON.parse(state.suggestions),
                          selectSuggestion,
                          removeFavorite,
                          editFavoriteName
                        })
                      }
                    }}
                  </div>
                </div>
                <div id="map"></div>
              </div>
            `}}
    </div>
  `
}

function SearchSuggestions ({ suggestions, selectSuggestion, removeFavorite, editFavoriteName }) {
  const isFavorite = s =>
    s.name && s.latitude != null && s.longitude != null && s.routeId

    const item = s => html`
      <span>
        <p @click="${() => selectSuggestion(s)}">${s.name || s.address}</p>

        ${isFavorite(s) ? html`
          <button
            class="edit-favorite-button"
            title="Rename Favorite"
            @click="${e => {
              e.stopPropagation()
              const newName = prompt("Edit favorite name:", s.name)
              if (newName && newName.trim() && newName !== s.name) {
                editFavoriteName(s, newName.trim())
              }
            }}"
          >✏️</button>

          <button
            class="remove-favorite-button"
            title="Remove from Favorites"
            @click="${e => {
              e.stopPropagation()
              removeFavorite(s)
            }}"
          >🗑️</button>
        ` : ''}
      </span>
    `

  return html`<div id="searchSuggestions">${suggestions.map(item)}</div>`
}

function NavigationDestination ({
  name,
  duration,
  distance,
  routeId,
  routeHash,
  isConfirmed,
  destinationCoordinates,
  startingCoordinates,
  isMetric,
  map,
  cancelNavigationFn,
  onConfirm,
  loadFavorites,
  removeFavorite,
  isFavorited,
  steps = []
}) {
  async function confirmDestination () {
    // Trigger route confirmation first to force UI update
    onConfirm?.()

    showSnackbar("Navigation set!")
    localStorage.setItem('activeRouteId', routeId)

    await fetch("/api/navigation", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({
        name,
        longitude: destinationCoordinates[0],
        latitude: destinationCoordinates[1]
      }),
    })

    await loadFavorites()

    map.flyTo({
      center: startingCoordinates,
      zoom: 18,
      pitch: 45,
      speed: 1,
      curve: 1
    })
  }

  async function cancelNavigation () {
    showSnackbar("Navigation cancelled...")
    removeRouteFromMap(map)
    cancelNavigationFn()
    localStorage.removeItem('activeRouteId')

    map.flyTo({
      center: startingCoordinates,
      zoom: 15,
      pitch: 45,
      speed: 1,
      curve: 1
    })

    await fetch("/api/navigation", { method: "DELETE" })
  }

  async function favoriteDestination () {
    const payload = {
      name,
      longitude: destinationCoordinates[0],
      latitude: destinationCoordinates[1],
      routeId
    }

    try {
      const res = await fetch("/api/navigation/favorite", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(payload)
      })
      const { message } = await res.json()
      showSnackbar(message || "Added to favorites!")
      await loadFavorites()
    } catch (err) {
      console.error("Favorite error:", err)
      showSnackbar("Failed to add to favorites...")
    }
  }

  async function toggleFavorite () {
    if (isFavorited) {
      await removeFavorite({
        name,
        latitude: destinationCoordinates[1],
        longitude: destinationCoordinates[0],
        routeId
      })
    } else {
      await favoriteDestination()
    }
  }

  const eta = new Date(Date.now() + duration * 1000)
  const isLong = duration > 86400

  const timeStr = eta.toLocaleTimeString([], { hour: 'numeric', minute: '2-digit' })
  const month = eta.toLocaleString([], { month: 'long' })
  const day = eta.getDate()
  const year = eta.getFullYear()
  const suffix = getOrdinalSuffix(day)
  const etaString = isLong ? `${month} ${day}${suffix}, ${year}, ${timeStr}` : timeStr

  return html`
    <div class="navigation-summary-widget">
      <div class="navigation-summary-title">${name}</div>

      <div class="summary-row">
        <span class="emoji">🛣️</span>
        <span class="label">Distance:</span>
        <span class="value">${formatMetersToHuman(distance, isMetric)}</span>
      </div>

      <div class="summary-row">
        <span class="emoji">⌛</span>
        <span class="label">Duration:</span>
        <span class="value">${formatSecondsToHuman(duration)}</span>
      </div>

      <div class="summary-row">
        <span class="emoji">🕗</span>
        <span class="label">ETA:</span>
        <span class="value">${etaString}</span>
      </div>

      <div class="buttonCluster">
        ${() => isConfirmed()
          ? html`<button class="cancel" @click="${cancelNavigation}">
              <i class="bi bi-x-lg"></i> Cancel Navigation
            </button>`
          : html`<button class="directions" @click="${confirmDestination}">
              <i class="bi bi-sign-turn-right"></i> Start Navigation
            </button>`}

        <button class="favorite" @click="${toggleFavorite}">
          ${isFavorited ? "💔 Unfavorite" : "❤️ Favorite"}
        </button>
      </div>
    </div>
  `
}
