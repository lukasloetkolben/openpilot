import { html, reactive } from "https://esm.sh/@arrow-js/core"

export function TSKManager () {
  const state = reactive({
    keys: [],
    newName: "",
    newValue: "",
    message: "",
    error: "",
    visible: false
  })

  let clearTimer = null
  let fadeTimer  = null

  function toast (type, text) {
    clearTimer && clearTimeout(clearTimer)
    fadeTimer  && clearTimeout(fadeTimer)
    state.error   = type === "error"   ? text : ""
    state.message = type === "message" ? text : ""
    state.visible = true
    clearTimer = setTimeout(() => { state.message = ""; state.error = "" }, 5000)
    fadeTimer  = setTimeout(() =>  state.visible = false, 5000)
  }

  const util = {
    req  : async (url, opts) => {
      const res = await fetch(url, opts)
      return { ok: res.ok, data: await res.json().catch(() => ({})) }
    },
    mask : k => k ? "x".repeat(k.length) : "",
    parse: txt => { try { return JSON.parse(txt) } catch { return [] } }
  }

  const api = {
    param: "/api/params?key=SecOCKeys",
    keys : "/api/tsk_keys",
    load : async () => {
      const { ok, data } = await util.req(api.param)
      if (!ok) return toast("error", "Failed to fetch keys…")
      state.keys = util.parse(data).map(k => ({ ...k, saved: true, edit: false }))
      if (state.keys.length > 0) {
        state.newName = state.keys[0].name
      }
    },
    save : async idx => {
      const isNew = idx == null
      const name  = isNew ? state.newName.trim()        : state.keys[idx].name.trim()
      const value = isNew ? state.newValue.trim()       : state.keys[idx].value.trim()
      if (!name || !value) return toast("error", "Both name and key are required!")
      const next = isNew
        ? [...state.keys, { name, value }]
        : state.keys.map((k, i) => i === idx ? { name, value } : k)
      const { ok, data } = await util.req(api.keys, {
        method : "POST",
        headers: { "Content-Type": "application/json" },
        body   : JSON.stringify(next)
      })
      if (!ok) return toast("error", data.error || "Save failed…")
      state.keys     = data.map(k => ({ ...k, saved: true, edit: false }))
      state.newValue = ""
      toast("message", isNew ? "Saved!" : "Updated!")
    },
    del  : async idx => {
      const doomed = state.keys[idx].name
      const { ok, data } = await util.req(`${api.keys}?name=${encodeURIComponent(doomed)}`, { method: "DELETE" })
      if (!ok) return toast("error", data.error || "Delete failed…")
      state.keys = state.keys.filter((_, i) => i !== idx)
      toast("message", "Deleted!")
    }
  }

  queueMicrotask(api.load)

  const row = (k, idx) => html`
    <label class="tskkeys-label" for="key-${idx}">${k.name}</label>
    <div class="tskkeys-row">
      <input
        autocomplete="off"
        class="tskkeys-input"
        id="key-${idx}"
        placeholder="xxxxxxxx…"
        value="${() => k.saved ? util.mask(k.value) : k.value}"
        @keydown="${e => {
          if (k.saved && !k.edit) {
            k.edit  = true
            k.saved = false
            k.value = ""
            e.target.value = ""
          }
        }}"
        @input="${e => k.value = e.target.value}"
      />
      <button
        class="${() => `tskkeys-btn ${k.saved ? "delete" : ""}`}"
        @click="${() => k.saved ? api.del(idx) : api.save(idx)}">
        ${() => k.saved ? "🗑️" : "💾"}
      </button>
    </div>
  `

  return html`
    <div class="tskkeys-wrapper tskkeys-offset-top">
      <div class="tskkeys-container">
        <div class="tskkeys-title">
          Toyota Security Key Manager
        </div>

        ${() => state.keys.map(row)}

        <div class="tskkeys-group">
          <label class="tskkeys-label" for="new-name">Key Name</label>
          <div class="tskkeys-row">
            <select
              class="tskkeys-input"
              id="new-name"
              @change="${e => state.newName = e.target.value}">
              ${() => state.keys.map(k => html`
                <option value="${k.name}" selected="${() => k.name === state.newName}">${k.name}</option>
              `)}
              <option value="">-- Add a new key --</option>
            </select>
          </div>

          <div class="tskkeys-row" style="${() => state.newName === '' ? '' : 'display:none;'}">
            <input
              autocomplete="off"
              class="tskkeys-input"
              placeholder="Toyota Rav4 Prime"
              value="${() => state.newName}"
              @input="${e => state.newName = e.target.value}"
            />
          </div>

          <label class="tskkeys-label" for="new-value">Key Value</label>
          <div class="tskkeys-row">
            <input
              autocomplete="off"
              class="tskkeys-input"
              id="new-value"
              placeholder="xxxxxxxx…"
              value="${() => state.newValue}"
              @input="${e => state.newValue = e.target.value}"
            />
            <button class="tskkeys-btn" @click="${() => api.save()}">💾</button>
          </div>
        </div>

        <div class="tskkeys-status">
          <div
            class="tskkeys-message"
            style="${() => state.message ? `opacity:${state.visible ? 1 : 0}` : "opacity:0"}">
            ${() => state.message}
          </div>
          <div
            class="tskkeys-error"
            style="${() => state.error ? `opacity:${state.visible ? 1 : 0}` : "opacity:0"}">
            ${() => state.error}
          </div>
        </div>
      </div>
    </div>
  `
}
