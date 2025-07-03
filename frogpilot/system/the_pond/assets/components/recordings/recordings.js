import { html, reactive } from "https://esm.sh/@arrow-js/core"

const routesStore = reactive({
  list: null,
  loading: false,
  error: null
})

async function loadRoutes() {
  if (routesStore.list || routesStore.loading) return
  routesStore.loading = true
  try {
    const res = await fetch("/api/routes")
    routesStore.list = await res.json()
  } catch (err) {
    routesStore.error = err
  } finally {
    routesStore.loading = false
  }
}

export function RecordedRoutes() {
  loadRoutes()
  return html`
    <h1>Dashcam Routes</h1>
    <p>View & download recorded routes.</p>
    <div class="route_grid">
      ${() => {
        if (routesStore.loading) return html`<div>Loading…</div>`
        if (routesStore.error) return html`<div>Failed to load</div>`
        if (!routesStore.list?.length) return html`<div>No routes found</div>`
        return routesStore.list.map(r => {
          const date = new Date(r.date).toLocaleString()
          return html`
            <a href="/routes/${r.name}" class="route_card">
              <div class="route_preview">
                <img src="${r.gif}" />
                <img class="image_preview" src="${r.png}" />
              </div>
              <p class="route_name">${date}</p>
            </a>`
        })
      }}
    </div>
  `
}

const routeCache = new Map()

async function getRoute(name) {
  if (routeCache.has(name)) return routeCache.get(name)
  const p = fetch(`/api/routes/${name}.json`).then(r => r.json())
  routeCache.set(name, p)
  return p
}

export function RecordedRoute({ params }) {
  const state = reactive({
    route: null,
    selectedCamera: "front",
    currentIndex: 0,
    playing: true,
    isSeeking: false,
    currentTime: 0
  })

  getRoute(params.name).then(data => state.route = data)

  function playPauseHandler() {
    const v = document.getElementById("video")
    state.playing = v.paused
    state.playing ? v.play() : v.pause()
  }

  function fullscreenHandler() {
    const v = document.getElementById("video")
    v.requestFullscreen?.() || v.webkitRequestFullscreen?.()
  }

  function videoEndedHandler(e) {
    const v = e.target
    state.currentIndex = (state.currentIndex + 1) % state.route.segment_urls.length
    v.src = state.route.segment_urls[state.currentIndex]
    v.load()
    v.play()
  }

  function timeupdateHandler(e) {
    if (state.isSeeking) return
    const v = e.target
    state.currentTime = state.currentIndex * 60 + Math.round(v.currentTime)
  }

  function mousedownHandler() {
    state.isSeeking = true
  }

  async function mouseupHandler(e) {
    state.isSeeking = false
    const v = document.getElementById("video")
    const s = e.target
    const val = Number(s.value)
    const idx = Math.floor(val / 60)
    if (idx !== state.currentIndex) {
      state.currentIndex = idx
      v.src = urlForIdx(state.route, state.selectedCamera, idx)
      v.load()
      v.play()
      for (let i = 0; i < 10; i++) {
        await new Promise(r => setTimeout(r, 100))
        if (v.duration > 2) break
      }
    }
    v.currentTime = val - state.currentIndex * 60
  }

  function urlForIdx(r, cam, i) {
    let u = r.segment_urls[i]
    if (cam === "driver") u += "?camera=driver"
    else if (cam === "wide") u += "?camera=wide"
    return u
  }

  function fmt(sec) {
    const h = Math.floor(sec / 3600)
    const m = Math.floor((sec % 3600) / 60)
    const s = Math.floor(sec % 60)
    return `${h ? h + ":" : ""}${m < 10 ? "0" : ""}${m}:${s < 10 ? "0" : ""}${s}`
  }

  function render(r) {
    const date = new Date(r.date).toLocaleString()
    const dur = fmt(r.total_duration)
    const isWide = r.available_cameras.includes("wide")
    const isDriver = r.available_cameras.includes("driver")
    return html`
      <h1 id="route_name">${date}</h1>
      <div class="camera_selector">
        <div class="selected_camera" id="forward"><p>Forward Camera</p></div>
        <div class="${isWide ? "" : "unavailable"}" id="wide"><p>Wide Camera</p></div>
        <div class="${isDriver ? "" : "unavailable"}" id="driver"><p>Driver Camera</p></div>
      </div>
      <div class="video_wrapper">
        <video
          id="video"
          autoplay
          muted
          playsinline
          @click="${playPauseHandler}"
          @fullscreenchange="${e => e.target.controls = !!document.fullscreenElement}"
          @ended="${videoEndedHandler}"
          @timeupdate="${timeupdateHandler}"
        >
          <source src="${r.segment_urls[0]}" type="video/mp4" />
        </video>
        <div class="videocontrols">
          <button id="playpause" @click="${playPauseHandler}">
            ${() => state.playing ? html`<i class="bi bi-pause-fill"></i>` : html`<i class="bi bi-play-fill"></i>`}
          </button>
          <input
            id="seekslider"
            type="range"
            min="0"
            max="${r.total_duration}"
            value="${() => state.currentTime}"
            @mousedown="${mousedownHandler}"
            @mouseup="${mouseupupHandler}"
            @input="${e => state.currentTime = e.target.value}"
            @change="${mouseupupHandler}"
            step="1"
          />
          <p>
            <span id="current-time">${() => fmt(state.currentTime)}</span>
            /
            <span id="duration">${dur}</span>
          </p>
          <button id="fullscreen" @click="${fullscreenHandler}">
            <i class="bi bi-fullscreen"></i>
          </button>
        </div>
      </div>
    `
  }

  return html`
    <div class="route">
      <a href="/routes" class="button">Back</a>
      ${() => state.route ? render(state.route) : "Loading…"}
    </div>
  `
}
