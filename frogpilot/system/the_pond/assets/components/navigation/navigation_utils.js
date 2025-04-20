export async function getCoordinatesFromSearch (searchValue, mapboxPublic) {
  const params = new URLSearchParams({ access_token: mapboxPublic, q: searchValue })
  const response = await fetch(
    `https://api.mapbox.com/search/geocode/v6/forward?${params.toString()}`
  )
  const data = await response.json()
  return data.features[0].geometry.coordinates
}

export async function getRoutes (from, to, mapboxPublic) {
  const url =
    `https://api.mapbox.com/directions/v5/mapbox/driving-traffic/${from};${to}` +
    `?geometries=geojson&annotations=congestion&overview=full&alternatives=true` +
    `&access_token=${mapboxPublic}`
  const response = await fetch(url)
  const data = await response.json()
  return data.routes
}

/**
 * addRouteToMap now accepts a sixth arg: selectedRouteId
 * and uses it for initial paint (width/opacity) so that the
 * remembered “live” route comes back highlighted on refresh.
 */
export function addRouteToMap (
  map,
  routes,
  start,
  dest,
  onRouteSelect,
  useMetric = true,
  selectedRouteId = 'main'
) {
  routes.forEach((route, idx) => {
    const routeId = idx === 0 ? 'main' : `alt-${idx}`
    const sourceId = `route-${routeId}`
    const layerId  = `route-line-${routeId}`

    if (map.getSource(sourceId)) {
      map.removeLayer(layerId)
      map.removeSource(sourceId)
    }

    const feature = {
      type: 'Feature',
      geometry: { type: 'LineString', coordinates: route.geometry.coordinates },
      properties: {
        congestion: route.legs[0].annotation.congestion,
        routeId,
        duration : route.duration,
        distance : route.distance,
      },
    }

    map.addSource(sourceId, {
      type : 'geojson',
      data : { type: 'FeatureCollection', features: [feature] },
      lineMetrics: true,
    })

    map.addLayer({
      id   : layerId,
      type : 'line',
      source: sourceId,
      layout: { 'line-cap': 'round', 'line-join': 'round' },
      paint : {
        'line-width'   : routeId === selectedRouteId ? 5 : 3,
        'line-opacity' : routeId === selectedRouteId ? 1 : 0.5,
        'line-gradient': buildGradientExpression(
          route.geometry.coordinates,
          route.legs[0].annotation.congestion
        ),
      },
    })

    map.on('click', layerId, () => {
      onRouteSelect(route, routeId)

      routes.forEach((r, i) => {
        const rid = i === 0 ? 'main' : `alt-${i}`
        const lid = `route-line-${rid}`
        if (!map.getLayer(lid)) return
        const sel = rid === routeId
        map.setPaintProperty(lid, 'line-width',   sel ? 5 : 3)
        map.setPaintProperty(lid, 'line-opacity', sel ? 1 : 0.5)
        if (sel) { try { map.moveLayer(lid) } catch (_) {} }
      })
    })

    map.on('mouseenter', layerId, e => {
      map.getCanvas().style.cursor = 'pointer'
      const props = feature.properties
      const duration = useMetric
        ? formatSecondsToHuman(props.duration)
        : formatSecondsToAmerican(props.duration)
      const distance = useMetric
        ? formatMetersToHuman(props.distance, true)
        : formatMetersToMiles(props.distance)

      const arrival = new Date(Date.now() + props.duration * 1000)
      const isLong  = props.duration > 24 * 3600
      const timeStr = arrival.toLocaleTimeString([], { hour: 'numeric', minute: '2-digit' })
      const month   = arrival.toLocaleString([], { month: 'long' })
      const day     = arrival.getDate()
      const year    = arrival.getFullYear()
      const suffix  = getOrdinalSuffix(day)
      const eta     = isLong ? `${month} ${day}${suffix}, ${year}, ${timeStr}` : timeStr

      const tooltip = document.createElement('div')
      tooltip.className = 'custom-tooltip'
      tooltip.style.whiteSpace = 'nowrap'
      tooltip.innerHTML =
        `<div class="tooltip-row"><span class="emoji">🛣️</span><span class="label">Distance:</span><span class="value">${distance}</span></div>
         <div class="tooltip-row"><span class="emoji">⌛</span><span class="label">Duration:</span><span class="value">${duration}</span></div>
         <div class="tooltip-row"><span class="emoji">🕗</span><span class="label">ETA:</span><span class="value">${eta}</span></div>`

      new mapboxgl.Popup({ closeButton: false, closeOnClick: false, className: 'route-tooltip', maxWidth: 'none' })
        .setLngLat(e.lngLat)
        .setDOMContent(tooltip)
        .addTo(map)
    })

    map.on('mouseleave', layerId, () => {
      map.getCanvas().style.cursor = ''
      document.querySelectorAll('.mapboxgl-popup').forEach(p => p.remove())
    })
  })

  const padding = window.innerWidth < 600 ? 20 : 100
  map.fitBounds([start, dest], { padding, duration: 1000 })
}

function buildGradientExpression (coords, congestion) {
  const count = congestion.length
  if (count === 0 || coords.length < 2) {
    return ['interpolate', ['linear'], ['line-progress'], 0, '#ccc', 1, '#ccc']
  }
  const stops = []
  for (let i = 0; i < count; i++) {
    stops.push(i / (count - 1), congestionToColor(congestion[i] || 'unknown'))
  }
  return ['interpolate', ['linear'], ['line-progress'], ...stops]
}

export function removeRouteFromMap (map) {
  if (!map || !map.getStyle || !map.getStyle()) return
  const layers = map.getStyle().layers || []
  layers.forEach(l => {
    if (l.id.startsWith('route-line-') && map.getLayer(l.id)) map.removeLayer(l.id)
  })
  const sources = map.getStyle().sources || {}
  Object.keys(sources).forEach(id => {
    if (id.startsWith('route-') && map.getSource(id)) map.removeSource(id)
  })
}

export function formatSecondsToHuman   (s) { const h = Math.floor(s / 3600); const m = Math.floor((s % 3600) / 60); return h > 0 ? `${h}h ${m} min` : `${m} min` }
export function formatMetersToHuman    (m, metric = true) { return metric ? (m >= 1000 ? `${(m/1000).toFixed(1)} km` : `${Math.round(m)} m`) : (m*3.28084 >= 5280 ? `${((m*3.28084)/5280).toFixed(1)} mi` : `${Math.round(m*3.28084)} ft`) }
export function formatSecondsToAmerican(s) { const mins = Math.round(s / 60); const h = Math.floor(mins / 60); const m = mins % 60; return h > 0 ? `${h} hr ${m} min` : `${m} min` }
export function formatMetersToMiles    (m) { const miles = m / 1609.34; return miles >= 0.1 ? `${miles.toFixed(1)} mi` : `${(miles*5280).toFixed(0)} ft` }

function congestionToColor (lvl) {
  const map = { low:'#2ecc71', moderate:'#f1c40f', heavy:'#e67e22', severe:'#e74c3c', unknown:'#2ecc71' }
  return map[lvl] || '#999'
}
export function getOrdinalSuffix (n) { const s=['th','st','nd','rd']; const v=n%100; return s[(v-20)%10] || s[v] || s[0] }
