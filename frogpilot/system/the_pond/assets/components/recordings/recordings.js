import { html, reactive } from "https://esm.sh/@arrow-js/core"

/**
 * @typedef {Object} Route
 * @prop {string} date
 * @prop {string[]} segment_urls
 * @prop {string} gif
 * @prop {string} png
 * @prop {string[]} available_cameras
 * @prop {number} total_duration
 * @prop {string} name
 */

export function RecordedRoute({ params }) {
  const state = reactive({
    route: undefined,
    playing: true,
    currentIndex: 0,
    isSeeking: false,
    currentTime: "",
    selectedCamera: "forward",
  })

  fetch(`/api/routes/${params.routeDate}`)
    .then((res) => res.json())
    .then((data) => {
      state.route = data
    })

  function playPauseHandler() {
    const video = document.getElementById("video")
    state.playing = video.paused
    state.playing ? video.play() : video.pause()
  }

  function fullscreenHandler() {
    const video = document.getElementById("video")
    video.requestFullscreen?.() ||
      video.mozRequestFullScreen?.() ||
      video.webkitRequestFullscreen?.() ||
      video.msRequestFullscreen?.()
  }

  function videoEndedHandler(e) {
    const video = e.target
    state.currentIndex = (state.currentIndex + 1) % state.route.segment_urls.length
    video.src = state.route.segment_urls[state.currentIndex]
    video.load()
    video.play()
  }

  function timeupdateHandler(e) {
    if (state.isSeeking) return
    const video = e.target
    const totalTime = state.currentIndex * 60 + Math.round(video.currentTime)
    state.currentTime = totalTime
  }

  async function handleSeek(e) {
    const value = e.target.value
    const video = document.getElementById("video")
    const desiredIndex = Math.floor(value / 60)

    if (desiredIndex !== state.currentIndex) {
      state.currentIndex = desiredIndex
      video.src = getUrlForIndex(state.route, state.selectedCamera, desiredIndex)
      video.load()
      video.play()

      for (let i = 0; i < 10; i++) {
        await wait(100)
        if (video.duration > 2) break
      }
    }

    video.currentTime = value - state.currentIndex * 60
  }

  function renderRoute(route) {
    const isWide = route.available_cameras.includes("wide")
    const isDriver = route.available_cameras.includes("driver")
    const formattedDate = new Date(route.date).toLocaleString()
    const formattedDuration = formatSeconds(route.total_duration)

    return html`
      <h1 id="route_name">${formattedDate}</h1>
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
          @fullscreenchange="${(e) =>
            (e.target.controls = !!document.fullscreenElement)}"
          @ended="${videoEndedHandler}"
          @timeupdate="${timeupdateHandler}"
        >
          <source src="${route.segment_urls[0]}" type="video/mp4" />
          Your browser does not support the video tag.
        </video>
        <div class="videocontrols">
          <button id="playpause" @click="${playPauseHandler}">
            ${() =>
              state.playing
                ? html`<i class="bi bi-pause-fill"></i>`
                : html`<i class="bi bi-play-fill"></i>`}
          </button>
          <input
            id="seekslider"
            type="range"
            min="0"
            max="${route.total_duration}"
            value="${() => state.currentTime}"
            @mousedown="${() => (state.isSeeking = true)}"
            @mouseup="${() => (state.isSeeking = false)}"
            @input="${(e) => (state.currentTime = e.target.value)}"
            @change="${handleSeek}"
            step="1"
          />
          <p>
            <span id="current-time">${() => formatSeconds(state.currentTime)}</span>
            /
            <span id="duration">${formattedDuration}</span>
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
      ${() => (state.route ? renderRoute(state.route) : "Loading...")}
    </div>
  `
}

export function RecordedRoutes() {
  let state = reactive({
    routes: [],
    loading: true,
  })

  async function fetchRoutes() {
    try {
      const response = await fetch("/api/routes")
      state.routes = await response.json()
    } finally {
      state.loading = false
    }
  }

  fetchRoutes()

  return html`
    <h1>Dashcam Routes</h1>
    <p>View & download recorded routes.</p>
    <div class="route_grid">
      ${() => {
        if (state.loading) {
          return html`<div>Loading...</div>`
        }
        return state.routes.map(route => {
          const formattedDate = new Date(route.date).toLocaleString()
          return html`
            <a href="/routes/${route.name}" class="route_card">
              <div class="route_preview">
                <img src="${route.gif}" />
                <img class="image_preview" src="${route.png}" />
              </div>
              <p class="route_name">${formattedDate}</p>
            </a>
          `
        })
      }}
    </div>
  `
}

function getUrlForIndex(route, selectedCamera, segmentIndex) {
  let url = route.segment_urls[segmentIndex]
  if (selectedCamera === "driver") url += "?camera=driver"
  else if (selectedCamera === "wide") url += "?camera=wide"
  return url
}

function formatSeconds(seconds) {
  const h = Math.floor(seconds / 3600)
  const m = Math.floor((seconds % 3600) / 60)
  const s = Math.floor(seconds % 60)
  return `${h > 0 ? `${h}:` : ""}${m < 10 ? "0" : ""}${m}:${s < 10 ? "0" : ""}${s}`
}

const wait = (ms) => new Promise((res) => setTimeout(res, ms))
