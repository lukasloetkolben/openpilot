import { html } from "https://esm.sh/@arrow-js/core"

export function ToggleControl () {
  const fileInput = document.createElement("input")
  fileInput.type = "file"
  fileInput.accept = ".json"
  fileInput.style.display = "none"
  fileInput.addEventListener("change", restoreToggles)
  document.body.appendChild(fileInput)

  async function backupToggles () {
    const response = await fetch("/api/toggles/backup", { method: "POST" })
    const blob = await response.blob()

    const downloadUrl = URL.createObjectURL(blob)
    const downloadLink = document.createElement("a")
    downloadLink.href = downloadUrl
    downloadLink.download = "toggle-backup.json"
    downloadLink.click()
    URL.revokeObjectURL(downloadUrl)

    showSnackbar("Toggles backed up!")
  }

  async function restoreToggles (event) {
    const uploadedFile = event.target.files[0]
    if (uploadedFile) {
      const fileContents = await uploadedFile.text()
      const toggleData = JSON.parse(fileContents)

      const response = await fetch("/api/toggles/restore", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(toggleData)
      })

      const result = await response.json()
      showSnackbar(result.message || "Toggles restored!")

      event.target.value = ""
    }
  }

  async function resetTogglesToDefault () {
    const confirmed = confirm("Are you sure you want to reset all toggles to their default FrogPilot values?")
    if (!confirmed) {
      return
    }

    const response = await fetch("/api/toggles/reset_default", { method: "POST" })
    const result = await response.json()
    showSnackbar(result.message || "Toggles reset to default!")
  }

  async function resetTogglesToStock () {
    const confirmed = confirm("Are you sure you want to reset all toggles to stock openpilot values?")
    if (!confirmed) {
      return
    }

    const response = await fetch("/api/toggles/reset_stock", { method: "POST" })
    const result = await response.json()
    showSnackbar(result.message || "Toggles reset to stock!")
  }

  function triggerRestorePrompt () {
    fileInput.click()
  }

  return html`
    <div class="toggle-control-wrapper">
      <section class="toggle-control-widget">
        <div class="toggle-control-title">Backup/Restore Toggles</div>
        <p class="toggle-control-text">
          Use the buttons below to backup or restore your toggles.
        </p>
        <button class="toggle-control-button" @click="${backupToggles}">Backup Toggles</button>
        <button class="toggle-control-button" @click="${triggerRestorePrompt}">Restore Toggles</button>
      </section>

      <section class="toggle-control-widget" style="margin-left: 1.5rem">
        <div class="toggle-control-title">Reset Toggles to Default FrogPilot/Stock openpilot</div>
        <p class="toggle-control-text">
          Reset all toggles to default FrogPilot/stock openpilot settings.
        </p>
        <button class="toggle-control-button" @click="${resetTogglesToDefault}">
          Reset to Default
        </button>
        <button class="toggle-control-button" @click="${resetTogglesToStock}">
          Reset to Stock
        </button>
      </section>
    </div>
  `
}
