import { html, reactive } from "https://esm.sh/@arrow-js/core"
import { formatSecondsToHuman, parseErrorLogToDate } from "../js/utils.js"

const state = reactive({
  loading: true,

  files: [],

  selectedLog: undefined,
})

;(async () => {
  const res = await fetch("/api/error_logs", {
    headers: { Accept: "application/json" }
  })
  const data = await res.json()

  state.files = data.map(f => {
    const date = parseErrorLogToDate(f)
    return {
      filename: f,
      date: date.toLocaleString(),
      timeSince: (Date.now() - date.getTime()) / 1000,
    }
  })

  state.loading = false
})()

export function ErrorLogs() {
  return html`
    <div class="error-logs-wrapper">
      <div id="errorLogs">
        <div id="fileList">
          ${() =>
            state.loading
              ? html`<div class="fileEntry"><p>Loading...</p></div>`
              : state.files.length === 0
                ? html`<div class="fileEntry"><p>No error logs!</p></div>`
                : state.files.map(file => html`
                    <div class="fileEntry"
                         @click="${() => {
                           state.selectedLog = state.selectedLog === file.filename ? undefined : file.filename
                         }}">
                      <p>${file.date}</p>
                      <p>${file.timeSince < 60 ? "just now" : `${formatSecondsToHuman(file.timeSince, "minutes")} ago`}</p>
                    </div>
                  `)
          }
        </div>
        ${() =>
          state.selectedLog ? Logviewer(state.selectedLog, () => (state.selectedLog = undefined)) : ""
        }
      </div>
    </div>
  `
}

function Logviewer(filename, closeFn) {
  const logState = reactive({
    loading: true,
    content: ""
  })

  ;(async () => {
    const res = await fetch(`/api/error_logs/${filename}`)
    logState.content = await res.text()
    logState.loading = false
  })()

  const deleteLog = async () => {
    await fetch(`/api/error_logs/${filename}`, {
      method: "DELETE"
    })
    state.files = state.files.filter(f => f.filename !== filename)
    closeFn()
  }

  return html`
    <div id="fileViewer">
      <div>
        <p>${filename}</p>
        <button @click="${closeFn}">
          <i class="bi bi-x-lg"></i>
        </button>
        <button @click="${deleteLog}">
          <i class="bi bi-trash"></i>
        </button>
        <a href="/api/error_logs/${filename}" download>
          <button>
            <i class="bi bi-download"></i>
          </button>
        </a>
      </div>
      <pre>${() => (logState.loading ? "Loading..." : logState.content)}</pre>
    </div>
  `
}
