import { html, reactive } from "https://esm.sh/@arrow-js/core"
import { formatSecondsToHuman, parseErrorLogToDate } from "../js/utils.js"

const logSelectorState = reactive({
  loading: false,
  files: [],
  logsLoadedOnce: false
});

async function loadTmuxLogs() {
  if (logSelectorState.loading || logSelectorState.logsLoadedOnce) {
    return;
  }

  logSelectorState.loading = true;
  try {
    const res = await fetch("/api/tmux_log/list");
    if (!res.ok) {
      throw new Error(await res.text());
    }
    const data = await res.json();
    logSelectorState.files = data.map(f => {
      const date = parseErrorLogToDate(f.replace("tmux_log_", "").replace(".json", "").replace("_", "--"));
      return {
        filename: f,
        date: date.toLocaleString(),
        timeSince: (Date.now() - date.getTime()) / 1000,
      };
    });
  } catch (err) {
    showSnackbar(`Failed to fetch logs: ${err.message}`, "error");
    logSelectorState.files = [];
  } finally {
    logSelectorState.loading = false;
    logSelectorState.logsLoadedOnce = true;
  }
}

function TmuxLogSelector({ action, closeFn }) {
  loadTmuxLogs();

  async function handleFileClick(file) {
    if (action === "download") {
      const link = document.createElement("a");
      link.href = `/api/tmux_log/download/${encodeURIComponent(file.filename)}`;
      link.download = file.filename;

      document.body.appendChild(link);
      link.click();
      document.body.removeChild(link);

    } else if (action === "rename") {
      const newName = prompt(`Rename ${file.filename} to:`);
      if (!newName || newName.trim() === file.filename) {
        return;
      }
      try {
        const res = await fetch(`/api/tmux_log/rename/${encodeURIComponent(file.filename)}/${encodeURIComponent(newName.trim())}`, {
          method: "PUT"
        });
        if (!res.ok) {
          throw new Error(await res.text());
        }
        showSnackbar(`${file.filename} renamed to ${newName.trim()}`, "success");
        logSelectorState.files = logSelectorState.files.map(f =>
          f.filename === file.filename
            ? { ...f, filename: newName.trim() }
            : f
        );
      } catch (err) {
        showSnackbar(`Rename failed: ${err.message}`, "error");
      }

    } else if (action === "delete") {
      if (!confirm(`Delete ${file.filename}?`)) {
        return;
      }
      try {
        const res = await fetch(`/api/tmux_log/delete/${encodeURIComponent(file.filename)}`, { method: "DELETE" });
        if (!res.ok) {
          throw new Error(await res.text());
        }
        showSnackbar(`${file.filename} deleted successfully!`, "success");
        logSelectorState.files = logSelectorState.files.filter(f => f.filename !== file.filename);
        if (logSelectorState.files.length === 0) {
          logSelectorState.logsLoadedOnce = false;
        }
      } catch (err) {
        showSnackbar(`Delete failed: ${err.message}`, "error");
      }
    }
  }

  return html`
    <div class="tmux-log-selector-wrapper" @click="${(e) => e.target === e.currentTarget && closeFn()}">
      <div id="fileList">
        <div class="fileEntry header">
          <p>Filename</p>
          <p>Date</p>
          <p>Age</p>
        </div>
        ${() => {
          if (logSelectorState.loading && !logSelectorState.logsLoadedOnce) {
            return html`<div class="fileEntry"><p>Loading...</p></div>`;
          }
          if (logSelectorState.files.length === 0) {
            return html`<div class="fileEntry"><p>No tmux logs found!</p></div>`;
          }
          return logSelectorState.files.map(file => html`
            <div class="fileEntry" @click="${() => handleFileClick(file)}">
              <p>${file.filename}</p>
              <p>${file.date}</p>
              <p>${file.timeSince < 60 ? "just now" : `${formatSecondsToHuman(file.timeSince, "minutes")} ago`}</p>
            </div>
          `);
        }}
        <button @click="${closeFn}" class="cancel-button">Close</button>
      </div>
    </div>
  `
}

export function TmuxLog() {
  const state = reactive({
    paused: false,
    latest: '',
    log: '',
    selectorAction: null,
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
        logSelectorState.files = []
        logSelectorState.logsLoadedOnce = false
      })
      .catch(err => {
        showSnackbar(`Capture failed: ${err.message}`, "error")
      })
  }

  function downloadSessions() {
    state.selectorAction = "download"
  }

  function deleteSession() {
    state.selectorAction = "delete"
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
        logSelectorState.files = []
        logSelectorState.logsLoadedOnce = false
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
        <button class="tmux-control-button" @click="${downloadSessions}">⬇️ Download Log</button>
        <button class="tmux-control-button" @click="${togglePause}">${() => state.paused ? "▶️ Resume Log" : "⏸️ Pause Log"}</button>
        <button class="tmux-control-button" @click="${() => state.selectorAction = 'rename'}">✏️ Rename Log</button>
      </div>

      ${() => state.selectorAction
        ? TmuxLogSelector({
            action: state.selectorAction,
            closeFn: () => (state.selectorAction = null)
          })
        : ""
      }
    </div>
  `
}
