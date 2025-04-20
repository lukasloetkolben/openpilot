import { html, reactive } from "https://esm.sh/@arrow-js/core"
import { Link } from "./router.js"

export function Home() {
  const state = reactive({
    data: null,
    unit: "miles",
  })

  fetchData()
  initUnitPreference()

  async function fetchData() {
    const response = await fetch("/api/stats")
    const json = await response.json()

    state.data = json
  }

  async function initUnitPreference() {
    const response = await fetch("/api/params?key=IsMetric")
    const isMetric = (await response.text()).trim() === "1"

    state.unit = isMetric ? "kilometers" : "miles"
    localStorage.setItem("isMetric", isMetric)
  }

  return html`
    <div>
      ${() => {
        const data = state.data
        if (!data) {
          return html`<p>Loading...</p>`
        }

        return html`
          <h1>The Pond</h1>

          <div class="drivingStats">
            ${DriveStat("All Time", data.driveStats?.all, state.unit)}
            ${DriveStat("Past Week", data.driveStats?.week, state.unit)}
            ${DriveStat("FrogPilot", data.driveStats?.frogpilot, state.unit)}
          </div>

          <h2>Disk Usage</h2>
          <div class="diskUsage">
            ${data.diskError
              ? html`<p>${data.diskError.join("<br>")}</p>`
              : (data.diskUsage?.length ? data.diskUsage.map(DiskUsage) : DiskUsage({ size: "0 GB", used: "0 GB", usedPercentage: "0" })
                )}
          </div>

          <h2>Firehose Segments</h2>
          <div class="firehoseStats">
            <p>
              <strong>${(data.firehoseStats?.segments ?? 0).toLocaleString("en-US")}</strong> segments in training data.
            </p>
          </div>

          <h2>Software Info</h2>
          <div class="softwareInfo">
            <div class="softwareGrid">
              ${renderSoftwareInfo(data.softwareInfo)}
            </div>
          </div>
        `
      }}
    </div>
  `
}

function DiskUsage(disk) {
  const used = parseFloat(disk.usedPercentage) || 0
  const rightRadius = used >= 100 ? "0" : "var(--border-radius-md)"

  return html`
    <div class="disk">
      <p>${disk.used} used of ${disk.size}</p>
      <div class="progress">
        <div
          class="bar"
          style="
            border-bottom-right-radius: ${rightRadius};
            border-top-right-radius: ${rightRadius};
            width: ${100 - used}%;
          "
        ></div>
      </div>
    </div>
  `
}

function DriveStat(title, stats = {}, defaultUnit) {
  const format = (n) => n?.toLocaleString("en-US", { minimumFractionDigits: 0, maximumFractionDigits: 0 }) ?? "0"

  return html`
    <div class="drivingStat">
      <h2>${title}</h2>
      <div><p>${format(stats.drives)}</p><p>drives</p></div>
      <div><p>${format(stats.distance)}</p><p>${stats.unit ?? defaultUnit}</p></div>
      <div><p>${format(stats.hours)}</p><p>hours</p></div>
    </div>
  `
}

function renderSoftwareInfo(info = {}) {
  const fields = [
    ["Branch Name", info.branchName],
    ["Build", info.buildEnvironment],
    ["Commit Hash", info.commitHash],
    ["Fork Maintainer", info.forkMaintainer],
    ["Update Available", info.updateAvailable],
    ["Version Date", info.versionDate],
  ]

  return fields.map(([label, value]) =>
    html`<p><strong>${label}:</strong> ${value ?? "Unknown"}</p>`
  )
}
