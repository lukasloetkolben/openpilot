import { html, reactive } from "https://esm.sh/@arrow-js/core"

export function TmuxLog() {
  const state = reactive({
    paused: false,

    latest: '',
    log: ''
  })

  const event_source = new EventSource("/api/tmux_log/live")

  event_source.onmessage = e => {
    state.latest = e.data
    if (!state.paused) {
      state.log = state.latest
    }
  }

  event_source.onerror = err => {
    console.error("Error receiving tmux log:", err)
    event_source.close()
  }

  function togglePause () {
    state.paused = !state.paused
    if (!state.paused) {
      state.log = state.latest
    }
  }

  function captureLog() {
    fetch("/api/tmux_log/capture", { method: "POST" })
      .then(res => {
        if (!res.ok) {
          return res.text().then(msg => { throw new Error(msg) })
        }
        showSnackbar("Current session captured!", "success")
      })
      .catch(err => {
        showSnackbar(`Capture failed: ${err.message}`, "error")
      })
  }

  function downloadSessions() {
    fetch("/api/tmux_log/list")
      .then(res => {
        if (!res.ok) {
          return res.text().then(msg => { throw new Error(msg) })
        }
        return res.json()
      })
      .then(files => {
        if (files.length === 0) {
          showSnackbar("No logs available...", "error")
          return
        }

        const choice = prompt(
          "Enter the number of the log to download:\n" +
          files.map((f, i) => `${i + 1}. ${f}`).join("\n")
        )

        const idx = parseInt(choice, 10) - 1
        if (idx >= 0 && idx < files.length) {
          window.open(`/api/tmux_log/download/${encodeURIComponent(files[idx])}`, "_blank")
        } else {
          showSnackbar("Invalid selection...", "error")
        }
      })
      .catch(err => {
        showSnackbar(`Failed to fetch logs: ${err.message}`, "error")
      })
  }

  function deleteSession() {
    fetch("/api/tmux_log/list")
      .then(res => {
        if (!res.ok) {
          return res.text().then(msg => { throw new Error(msg) })
        }
        return res.json()
      })
      .then(files => {
        if (files.length === 0) {
          showSnackbar("No logs available...", "error")
          return
        }

        const choice = prompt(
          "Enter the number of the log to delete:\n" +
          files.map((f, i) => `${i + 1}. ${f}`).join("\n")
        )

        const idx = parseInt(choice, 10) - 1
        if (idx >= 0 && idx < files.length) {
          const filename = files[idx]
          if (!confirm(`Delete ${filename}?`)) {
            return
          }

          fetch(`/api/tmux_log/delete/${encodeURIComponent(filename)}`, { method: "DELETE" })
            .then(res => {
              if (!res.ok) {
                return res.text().then(msg => { throw new Error(msg) })
              }
              showSnackbar(`${filename} deleted successfully!`, "success")
            })
            .catch(err => {
              showSnackbar(`Delete failed: ${err.message}`, "error")
            })
        } else {
          showSnackbar("Invalid selection...", "error")
        }
      })
      .catch(err => {
        showSnackbar(`Delete failed: ${err.message}`, "error")
      })
  }

  function deleteAllSessions() {
    if (!confirm("Are you sure you want to delete all of your session logs?")) {
      return
    }

    fetch("/api/tmux_log/delete_all", { method: "DELETE" })
      .then(res => {
        if (!res.ok) {
          return res.text().then(msg => { throw new Error(msg) })
        }
        showSnackbar("All logs deleted successfully!", "success")
      })
      .catch(err => {
        showSnackbar(`Delete-all failed: ${err.message}`, "error")
      })
  }

  return html`
    <div class="tmux-block">
      <div class="tmux-wrapper">
        <div class="tmuxContainer">
          <div class="tmuxHeader">Tmux Live Log</div>
          <pre class="tmuxLog">${() => state.log}</pre>
        </div>
      </div>

      <div class="tmux-controls">
        <button class="tmux-control-button" @click="${captureLog}">💾 Capture Log</button>
        <button class="tmux-control-button" @click="${deleteSession}">🗑️ Delete Log</button>
        <button class="tmux-control-button" @click="${deleteAllSessions}">🧨 Delete All Logs</button>
        <button class="tmux-control-button" @click="${downloadSessions}">⬇️ Download Logs</button>
        <button class="tmux-control-button" @click="${togglePause}">${() => state.paused ? "▶️ Resume Log" : "⏸️ Pause Log"}</button>
      </div>
    </div>
  `
}
