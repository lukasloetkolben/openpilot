import { html, reactive } from "https://esm.sh/@arrow-js/core"
import { createBrowserHistory, createRouter } from "https://esm.sh/@remix-run/router@1.3.1"
import { hideSidebar } from "../js/utils.js"
import { DoorControl } from "./doors.js"
import { ErrorLogs } from "./error_logs.js"
import { Home } from "./home.js"
import { NavDestination } from "./navigation/navigation_destination.js"
import { NavKeys } from "./navigation/navigation_keys.js"
import { RouteRecordings } from "./recordings/recordings.js"
import { SettingsView } from "./settings/settings.js"
import { ScreenRecordings } from "./recordings/screen_recordings.js"
import { Sidebar } from "./sidebar.js"
import { SpeedLimits } from "./speed_limits.js"
import { TailscaleControl } from "./tailscale.js"
import { TmuxLog } from "./tmux.js"
import { ToggleControl } from "./toggles.js"

let router, routerState

function createRoute(id, path, component) {
  return {
    id,
    path,
    loader: () => {},
    element: component,
  }
}

function Root() {
  let routes = [
    createRoute("doors", "/doors", DoorControl),
    createRoute("errorLogs", "/error_logs", ErrorLogs),
    createRoute("navdestination", "/navigation", NavDestination),
    createRoute("navkeys", "/navigation_keys", NavKeys),
    createRoute("root", "/", Home),
    createRoute("routes", "/routes", RouteRecordings),
    createRoute("screen_recordings", "/screen_recordings", ScreenRecordings),
    createRoute("settings", "/settings/:section/:subsection?", SettingsView),
    createRoute("speed_limits", "/speed_limits", SpeedLimits),
    createRoute("tailscale", "/tailscale", TailscaleControl),
    createRoute("tmux", "/tmux", TmuxLog),
    createRoute("toggles", "/toggles", ToggleControl),
  ]

  router = createRouter({
    routes,
    history: createBrowserHistory(),
  }).initialize()

  routerState = reactive({
    activePath: "/",
    activePathFull: "/",
    initialized: false,
    navigation: { state: "loading" },
    errors: [],
    params: {},
  })

  router.subscribe(({ initialized, navigation, matches, errors }) => {
    const [match] = matches
    Object.assign(routerState, {
      initialized,
      activePath: match.route.path,
      activePathFull: match.pathname,
      navigation,
      params: match.params,
      errors,
    })
  })

  return html`
    ${() => Sidebar(routerState.activePathFull)}
    <div class="content">
      ${() => {
        if (!routerState.initialized || routerState.navigation.state === "loading") {
          return html`<div>Loading...</div>`
        }

        if (routerState.errors?.root?.status === 404) {
          return html`<h1>Not Found</h1>`
        }

        const match = routes.find(r => r.path === routerState.activePath)
        return match.element({ params: routerState.params })
      }}
    </div>
  `
}

export function Link(href, children, onClick, classes = "") {
  return html`<a
    class="${classes}"
    href="${() => href}"
    @click="${(e) => {
      e.preventDefault()
      router.navigate(e.currentTarget.href)
      hideSidebar()
      onClick?.()
    }}"
  >${children}</a>`
}

export function Navigate(href) {
  router.navigate(href)
  window.scrollTo(0, 0)
}

Root()(document.getElementById("app"))
