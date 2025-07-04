import { html, reactive } from "https://esm.sh/@arrow-js/core"

export function TSKManager() {
  const state = reactive({
    keys: [],
    message: "",
    error: "",
    visible: false
  })

  let clearTimer = null
  let fadeTimer = null

  function showMessage(type, text) {
    clearTimeout(clearTimer)
    clearTimeout(fadeTimer)

    state.error = type === "error" ? text : ""
    state.message = type === "message" ? text : ""
    state.visible = true

    clearTimer = setTimeout(() => {
      state.message = ""
      state.error = ""
    }, 5000)

    fadeTimer = setTimeout(() => {
      state.visible = false
    }, 5000)
  }

  const util = {
    req: async (url, opts) => {
      const response = await fetch(url, opts)
      return {
        ok: response.ok,
        data: await response.json().catch(() => ({}))
      }
    }
  }

  const api = {
    path: "/api/tsk_keys",

    load: async () => {
      const { ok, data } = await util.req(api.path)
      if (!ok) {
        return showMessage("error", "Failed to load keys")
      }

      state.keys = Array.isArray(data)
        ? data.map(k => ({ name: k.name ?? "", value: k.value ?? "" }))
        : []
    },

    save: async () => {
      const payload = state.keys
        .map(k => ({ name: k.name.trim(), value: k.value.trim() }))
        .filter(k => k.name && k.value)

      const { ok, data } = await util.req(api.path, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(payload)
      })

      if (!ok) {
        return showMessage("error", data.error || "Save failed")
      }

      state.keys = data
      showMessage("message", "Saved!")
    }
  }

  function addKey() {
    state.keys.push({ name: "", value: "" })
  }

  queueMicrotask(api.load)

  return html`
    <div class="tskkeys-wrapper tskkeys-offset-top">
      <div class="tskkeys-container">
        <div class="tskkeys-title">Toyota Security Key Manager</div>

        ${() => state.keys.map((key, i) => html`
          <div>
            <label class="tskkeys-label" for="key-name-${i}">Key Name</label>
            <div class="tskkeys-row">
              <input
                id="key-name-${i}"
                class="tskkeys-input"
                placeholder="Enter key name..."
                value="${() => key.name}"
                @input="${e => key.name = e.target.value}"
              />
            </div>

            <label class="tskkeys-label" for="key-value-${i}">Key Value</label>
            <div class="tskkeys-row">
              <input
                id="key-value-${i}"
                class="tskkeys-input"
                placeholder="Enter key value..."
                value="${() => key.value}"
                @input="${e => key.value = e.target.value}"
              />
            </div>
            <hr />
          </div>
        `)}

        <div class="tskkeys-row">
          <button class="tskkeys-btn" @click="${addKey}">➕ Add Key</button>
          <button class="tskkeys-btn" @click="${api.save}">💾 Save All</button>
        </div>

        <div class="tskkeys-status">
          <div
            class="tskkeys-message"
            style="${() => state.message ? `opacity: ${state.visible ? 1 : 0}` : "opacity: 0"}">
            ${() => state.message}
          </div>
          <div
            class="tskkeys-error"
            style="${() => state.error ? `opacity: ${state.visible ? 1 : 0}` : "opacity: 0"}">
            ${() => state.error}
          </div>
        </div>
      </div>
    </div>
  `
}
