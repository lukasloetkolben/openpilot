import { html, reactive } from "https://esm.sh/@arrow-js/core"

export function TailscaleControl() {
  const installState = reactive({ status: "idle", installed: false })

  async function checkInstallStatus() {
    try {
      const response = await fetch("/api/tailscale/installed")
      const result = await response.json()
      installState.installed = result.installed
    } catch (error) {
      console.error("Failed to check Tailscale install status:", error)
    }
  }

  async function handleInstall() {
    if (installState.status !== "idle") {
      return
    }

    const action = installState.installed ? "uninstall" : "install"
    installState.status = installState.installed ? "uninstalling" : "installing"

    showSnackbar(`${action.charAt(0).toUpperCase() + action.slice(1)} started...`)

    const endpoint = installState.installed ? "/api/tailscale/uninstall" : "/api/tailscale/setup"
    const response = await fetch(endpoint, { method: "POST" })
    const result = await response.json()

    showSnackbar(result.message || `${action.charAt(0).toUpperCase() + action.slice(1)} triggered...`)
    installState.status = installState.installed ? "uninstalled" : "installed"

    if (result.auth_url) {
      window.open(result.auth_url, "_blank")
    }
  }

  checkInstallStatus()

  return html`
    <div class="tailscale-wrapper">
      <section class="tailscale-widget">
        <div class="tailscale-title">
          ${() => installState.installed ? 'Uninstall Tailscale' : 'Install Tailscale'}
        </div>
        <p class="tailscale-text">
          Tailscale creates a secure, private connection between your openpilot device and your phone or PC so you can access and control it from anywhere!
        </p>
        <div class="tailscale-button-wrapper">
          <button
            class="tailscale-button"
            @click="${handleInstall}"
            disabled="${() => installState.status === 'installing' || installState.status === 'uninstalling' || installState.status === 'installed' || installState.status === 'uninstalled'}"
          >
            ${() => {
              if (installState.status === 'installing') return 'Installing...'
              if (installState.status === 'uninstalling') return 'Uninstalling...'
              if (installState.status === 'installed') return 'Installed!'
              if (installState.status === 'uninstalled') return 'Uninstalled!'
              return installState.installed ? 'Uninstall' : 'Install'
            }}
          </button>
          <a class="tailscale-link" href="https://tailscale.com/download" target="_blank">
            Download Tailscale on your other devices
          </a>
        </div>
      </section>
    </div>
  `
}
