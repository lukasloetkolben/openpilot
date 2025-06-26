[![openpilot on the comma 3X](https://i.imgur.com/Zwv9SaN.png)](https://comma.ai/shop/comma-3x)

What is openpilot?
------

[openpilot](http://github.com/commaai/openpilot) is an open source driver assistance system. Currently, openpilot performs the functions of Adaptive Cruise Control (ACC), Automated Lane Centering (ALC), Forward Collision Warning (FCW), and Lane Departure Warning (LDW) for a growing variety of [supported car makes, models, and model years](docs/CARS.md). In addition, while openpilot is engaged, a camera-based Driver Monitoring (DM) feature alerts distracted and asleep drivers. See more about [the vehicle integration](docs/INTEGRATION.md) and [limitations](docs/LIMITATIONS.md).

<table>
  <tr>
    <td><a href="https://youtu.be/NmBfgOanCyk" title="Video By Greer Viau"><img src="https://github.com/commaai/openpilot/assets/8762862/2f7112ae-f748-4f39-b617-fabd689c3772"></a></td>
    <td><a href="https://youtu.be/VHKyqZ7t8Gw" title="Video By Logan LeGrand"><img src="https://github.com/commaai/openpilot/assets/8762862/92351544-2833-40d7-9e0b-7ef7ae37ec4c"></a></td>
    <td><a href="https://youtu.be/SUIZYzxtMQs" title="A drive to Taco Bell"><img src="https://github.com/commaai/openpilot/assets/8762862/05ceefc5-2628-439c-a9b2-89ce77dc6f63"></a></td>
  </tr>
</table>

What is FrogPilot? 🐸
------

FrogPilot is a fully open-sourced fork of openpilot, featuring clear and concise commits striving to be a resource for the openpilot developer community. It thrives on contributions from both users and developers, focusing on a collaborative, community-led approach to deliver an advanced openpilot experience for everyone!

------
FrogPilot was last updated on:

**April 12th, 2025**

------

# Features

## 🔊 Alerts & Sounds
<details>
<summary>Alert Volume Controller</summary>

  **Description:** Individually adjust the volume of each alert category (Engage/Disengage, Prompt, Warning, etc.).

  **How to Enable:** *Settings → Visuals → Alert Volume Controller*
</details>

<details>
<summary>Green Light Alert</summary>

  **Description:** Plays a chime when the traffic light ahead turns green.

  **How to Enable:** *Settings → Visuals → Custom Alerts*
</details>

<details>
<summary>Lead Departing Alert</summary>

  **Description:** Notifies you when the stopped lead car pulls away.

  **How to Enable:** *Settings → Visuals → Custom Alerts*
</details>

<details>
<summary>Speed Limit Change Alert</summary>

  **Description:** Chimes whenever the posted speed limit changes.

  **How to Enable:** *Settings → Visuals → Custom Alerts*
</details>

---

## 🛠 Device & System
<details>
<summary>Automatic Updates</summary>

  **Description:** Downloads and installs new FrogPilot versions automatically when off-road.

  **How to Enable:** *Settings → Software → Auto-Update*
</details>

<details>
<summary>Battery Monitor (Auto-Shutdown)</summary>

  **Description:** Shuts down the device if 12 V battery voltage drops below a set threshold.

  **How to Enable:** *Settings → Device → Battery Shutdown*
</details>

<details>
<summary>Delete Driving Data</summary>

  **Description:** Erases all stored logs and video footage with one tap.

  **How to Enable:** *Settings → Device → Delete Driving Data*
</details>

<details>
<summary>Device Shutdown Timer</summary>

  **Description:** Powers down the device X minutes after the car turns off.

  **How to Enable:** *Settings → Device → Off-road Shutdown Timer*
</details>

<details>
<summary>Disable Logging</summary>

  **Description:** Stops local drive logging for privacy and reduced storage wear.

  **How to Enable:** *Settings → Device → Disable Logging*
</details>

<details>
<summary>Disable Onroad Uploads</summary>

  **Description:** Prevents data uploads while driving; uploads only when parked on Wi-Fi.

  **How to Enable:** *Settings → Network → Uploads Only Off-road*
</details>

<details>
<summary>Disable Uploads</summary>

  **Description:** Blocks all cloud uploads for maximum privacy.

  **How to Enable:** *Settings → Network → Disable Uploads*
</details>

<details>
<summary>Enable Offline Mode</summary>

  **Description:** Lets FrogPilot run indefinitely with zero internet or Comma account.

  **How to Enable:** *Settings → Device → Offline Indefinitely*
</details>

<details>
<summary>Enable SSH Access</summary>

  **Description:** Turns on the SSH server for remote shell access.

  **How to Enable:** *Settings → Network → Enable SSH*
</details>

<details>
<summary>Enable Tethering (Hotspot)</summary>

  **Description:** Starts the device’s Wi-Fi hotspot (Off, On-road Only, or Always).

  **How to Enable:** *Settings → Network → Tethering*
</details>

<details>
<summary>Flash Panda (Reflash Panda Firmware)</summary>

  **Description:** One-tap recovery/upgrade of the Panda CAN-interface firmware.

  **How to Enable:** *Settings → Device → Flash Panda*
</details>

<details>
<summary>FrogPilot Backups</summary>

  **Description:** Backup/restore the entire FrogPilot software state.

  **How to Enable:** *Settings → Device → Backups → Create/Restore*
</details>

<details>
<summary>Reset Toggles to Default</summary>

  **Description:** Restores every toggle to factory defaults.

  **How to Enable:** *Settings → Device → Reset Toggles*
</details>

<details>
<summary>Toggle Backups</summary>

  **Description:** Save or restore only your toggle configuration profiles.

  **How to Enable:** *Settings → Device → Backups → Backup/Restore Toggles*
</details>

---

## 🚗 Driving Features
<details>
<summary>Acceleration Profile</summary>

  **Description:** Choose Sport (quick) or Eco (gentle) acceleration behavior.

  **How to Enable:** *Settings → Controls → Longitudinal Tuning → Acceleration Profile*
</details>

<details>
<summary>Always On Lateral</summary>

  **Description:** Keeps lane-centering active without cruise; optional pause on brake/signal/low-speed.

  **How to Enable:** *Settings → Controls → Always On Lateral*
</details>

<details>
<summary>Conditional Experimental Mode</summary>

  **Description:** Auto-switches to Experimental Mode for sharp turns, intersections, slow traffic, etc.

  **How to Enable:** *Settings → Controls → Conditional Experimental Mode*
</details>

<details>
<summary>Custom Driving Personalities</summary>

  **Description:** Fine-tune following distance and responsiveness for Relaxed, Standard, Aggressive, and Traffic profiles.

  **How to Enable:** *Settings → Controls → Driving Personalities → Customize*
</details>

<details>
<summary>Deceleration Profile</summary>

  **Description:** Sport or Eco style braking when slowing down.

  **How to Enable:** *Settings → Controls → Longitudinal Tuning → Deceleration Profile*
</details>

<details>
<summary>Experimental Mode Shortcuts</summary>

  **Description:** Toggle Experimental Mode via double-tap screen, long-press distance, or double-click LKAS.

  **How to Enable:** *Settings → Controls → Experimental Mode Activation*
</details>

<details>
<summary>Force Auto Tune (Unsupported Cars)</summary>

  **Description:** Forces steering calibration on unofficial car models.

  **How to Enable:** *Settings → Controls → Force Auto Tuning*
</details>

<details>
<summary>Increase Acceleration Behind Lead</summary>

  **Description:** Accelerates faster when the lead car starts moving.

  **How to Enable:** *Settings → Controls → Longitudinal Tuning*
</details>

<details>
<summary>Increase Set Speed Increment</summary>

  **Description:** Change cruise-speed step size (e.g., 5 mph per tap).

  **How to Enable:** *Settings → Controls → Longitudinal Tuning*
</details>

<details>
<summary>Increase Stopping Distance Behind Lead</summary>

  **Description:** Stops farther back from lead vehicles at red lights/traffic.

  **How to Enable:** *Settings → Controls → Longitudinal Tuning*
</details>

<details>
<summary>Lane Change Timer</summary>

  **Description:** Set a delay before automatic lane changes execute.

  **How to Enable:** *Settings → Controls → Lane Change Customizations*
</details>

<details>
<summary>Lane Detection Threshold</summary>

  **Description:** Adjust how strong lane lines must be before lane change is allowed.

  **How to Enable:** *Settings → Controls → Lane Change Customizations*
</details>

<details>
<summary>Lead Detection Threshold</summary>

  **Description:** Tune how early/late the system reacts to a lead vehicle.

  **How to Enable:** *Settings → Controls → Longitudinal Tuning*
</details>

<details>
<summary>Map Turn Speed Controller</summary>

  **Description:** Slows for upcoming curves using map data; adjustable aggressiveness.

  **How to Enable:** *Settings → Controls → Map Turn Speed Control*
</details>

<details>
<summary>Minimum Lane Change Speed</summary>

  **Description:** Configure the minimum speed at which auto lane changes initiate.

  **How to Enable:** *Settings → Controls → Lane Change Customizations*
</details>

<details>
<summary>Model Selector</summary>

  **Description:** Download and switch between Present, Past, and Experimental driving models.

  **How to Enable:** *Settings → Controls → Model Selector*
</details>

<details>
<summary>Nudgeless Lane Change</summary>

  **Description:** Executes lane changes with signal only – no steering nudge required.

  **How to Enable:** *Settings → Controls → Lane Change Customizations*
</details>

<details>
<summary>One Lane Change Per Signal</summary>

  **Description:** Limits to one lane change per turn-signal toggle.

  **How to Enable:** *Settings → Controls → Lane Change Customizations*
</details>

<details>
<summary>Pause Below Speed (Lateral)</summary>

  **Description:** Temporarily pauses lane-centering below a chosen speed.

  **How to Enable:** *Settings → Controls → Always On Lateral*
</details>

<details>
<summary>Pause On Brake (Lateral)</summary>

  **Description:** Brake pedal pauses steering assist instead of fully disengaging it.

  **How to Enable:** *Settings → Controls → Always On Lateral*
</details>

<details>
<summary>Pause On Turn Signal (Lateral)</summary>

  **Description:** Pauses lane-centering while your turn signal is active.

  **How to Enable:** *Settings → Controls → Always On Lateral*
</details>

<details>
<summary>Smoother Braking</summary>

  **Description:** Gentler deceleration when approaching slower traffic.

  **How to Enable:** *Settings → Controls → Longitudinal Tuning*
</details>

<details>
<summary>Speed Limit Controller</summary>

  **Description:** Auto-sets cruise speed to the posted limit with configurable offsets.

  **How to Enable:** *Settings → Controls → Speed Limit Controller*
</details>

<details>
<summary>Steer Ratio</summary>

  **Description:** Override the steering ratio used for lateral control.

  **How to Enable:** *Settings → Controls → Lateral Tuning*
</details>

<details>
<summary>Traffic Mode</summary>

  **Description:** Hold distance button ~2.5 s to activate extra-smooth stop-and-go profile (UI turns red).

  **How to Enable:** *Settings → Controls → Longitudinal Tuning*  
  *(Activate on-road by distance-button hold)*
</details>

<details>
<summary>Turn Desires (Precise Turns)</summary>

  **Description:** Allows sharper steering at low speed using model “turn desires”.

  **How to Enable:** *Settings → Controls → Lateral Tuning*
</details>

<details>
<summary>Twilsonco's NNFF Steering</summary>

  **Description:** Uses a neural-network feed-forward steering controller (full or Lite).

  **How to Enable:** *Settings → Controls → Lateral Tuning*
</details>

<details>
<summary>Vision Turn Speed Controller</summary>

  **Description:** Slows for curves detected by the camera (no map required); adjustable sensitivity.

  **How to Enable:** *Settings → Controls → Vision Turn Speed Controller*
</details>

---

## 🗺️ Navigation Features
<details>
<summary>3D Buildings on Map</summary>

  **Description:** Renders 3-D building models on the navigation map in supported areas.

  **How to Enable:** *Settings → Navigation → Show 3D Buildings*
</details>

<details>
<summary>Custom Map Styles</summary>

  **Description:** Choose alternative color themes (dark, high-contrast, etc.) for the map.

  **How to Enable:** *Settings → Navigation → Map Style*
</details>

<details>
<summary>iOS Shortcuts Integration</summary>

  **Description:** Use Siri/iOS Shortcuts to send destinations to FrogPilot instantly.

  **How to Enable:** Install the provided iOS Shortcut → Trigger via Siri or tap
</details>

<details>
<summary>Navigate on OpenPilot (No Comma Prime)</summary>

  **Description:** Full turn-by-turn routing without a subscription.

  **How to Enable:** *Settings → Navigation* (Primeless Navigation enabled by default)
</details>

<details>
<summary>Offline Maps</summary>

  **Description:** Download regions for completely offline routing and map tiles.

  **How to Enable:** *Settings → Navigation → Download Offline Maps*
</details>

<details>
<summary>OpenStreetMap Integration</summary>

  **Description:** Uses OSM data for road names, speed limits, and routing (always on).

  **How to Enable:** Enabled by default (ensure offline maps downloaded for no-cell service)
</details>

---

## 🎨 User Interface
<details>
<summary>Big Map (Full-Screen Map)</summary>

  **Description:** Expands the navigation map to cover most or all of the screen.

  **How to Enable:** *Settings → Visuals → Map Display → Full-Screen Map*
</details>

<details>
<summary>Camera View</summary>

  **Description:** Select wide, normal, or auto camera perspective for the on-road feed.

  **How to Enable:** *Settings → Visuals → Camera View*
</details>

<details>
<summary>Color Theme</summary>

  **Description:** Swap the entire UI color palette (Frog, Tesla, holiday themes, etc.).

  **How to Enable:** *Settings → Visuals → Custom Themes → Color Theme*
</details>

<details>
<summary>Compass</summary>

  **Description:** Shows a live compass heading on the on-road screen.

  **How to Enable:** *Settings → Visuals → Custom On-road UI* → Compass
</details>

<details>
<summary>Developer UI (Border Metrics)</summary>

  **Description:** Uses the screen border to show metrics like blind-spot or steering torque.

  **How to Enable:** *Settings → Visuals → Developer UI → Border Metrics*
</details>

<details>
<summary>Developer UI (FPS Counter)</summary>

  **Description:** Displays real-time UI frames-per-second.

  **How to Enable:** *Settings → Visuals → Developer UI → FPS Counter*
</details>

<details>
<summary>Developer UI (Lateral Metrics)</summary>

  **Description:** Shows live steering-control numbers/graphs.

  **How to Enable:** *Settings → Visuals → Developer UI → Lateral Metrics*
</details>

<details>
<summary>Developer UI (Longitudinal Metrics)</summary>

  **Description:** Displays acceleration/braking metrics on screen.

  **How to Enable:** *Settings → Visuals → Developer UI → Longitudinal Metrics*
</details>

<details>
<summary>Developer UI (Numerical Temperature Gauge)</summary>

  **Description:** Replaces “OK/HIGH” with exact °C/°F device temps.

  **How to Enable:** *Settings → Visuals → Developer UI → Numerical Temp Gauge*
</details>

<details>
<summary>Developer UI (Sidebar Stats)</summary>

  **Description:** Adds CPU, GPU, memory, IP, and storage stats to the sidebar (tap to cycle views).

  **How to Enable:** *Settings → Visuals → Developer UI → Sidebar Stats*
</details>

<details>
<summary>Developer UI (Use SI Units)</summary>

  **Description:** Switches Developer UI metrics to metric/SI units.

  **How to Enable:** *Settings → Visuals → Developer UI → Use SI Units*
</details>

<details>
<summary>Driver Camera on Reverse</summary>

  **Description:** Shows the driver-facing camera feed when you shift into reverse.

  **How to Enable:** *Settings → Visuals → Quality of Life* → Driver Camera on Reverse
</details>

<details>
<summary>Driving Statistics on Home</summary>

  **Description:** Displays lifetime/weekly drive stats on the launcher home screen.

  **How to Enable:** *Settings → Device* → Drive Stats on Home Screen
</details>

<details>
<summary>Fleet Manager</summary>

  **Description:** Browse, replay, and manage your recorded drives directly on the device.

  **How to Access:** Home screen → Fleet Manager (or *Settings → Device → Fleet Manager*)
</details>

<details>
<summary>Hide Speed</summary>

  **Description:** Removes the speedometer from the UI (optional tap-to-show).

  **How to Enable:** *Settings → Visuals → Quality of Life* → Hide Speed
</details>

<details>
<summary>Hide UI Elements</summary>

  **Description:** Granularly hide lane lines, icons, sidebar gauges, etc.

  **How to Enable:** *Settings → Visuals → Screen Management* → Hide UI Elements
</details>

<details>
<summary>Icon Pack</summary>

  **Description:** Replace default icons with themed packs (Tesla icons, custom sets, etc.).

  **How to Enable:** *Settings → Visuals → Custom Themes → Icon Pack*
</details>

<details>
<summary>Lane Line Width</summary>

  **Description:** Adjust the thickness of lane lines drawn on the UI.

  **How to Enable:** *Settings → Visuals → Model UI → Lane Line Width*
</details>

<details>
<summary>Lead Marker (Hide Lead Indicator)</summary>

  **Description:** Hides the lead-vehicle icon for a cleaner view.

  **How to Enable:** *Settings → Visuals → Model UI → Hide Lead Vehicle Marker*
</details>

<details>
<summary>Map Style</summary>

  **Description:** Change the map’s color theme to dark, retro, high-contrast, etc.

  **How to Enable:** *Settings → Visuals → Quality of Life* → Map Style
</details>

<details>
<summary>Path Edge Width</summary>

  **Description:** Make the colored edge highlights of the path thicker/thinner.

  **How to Enable:** *Settings → Visuals → Model UI → Path Edges Width*
</details>

<details>
<summary>Path Width</summary>

  **Description:** Adjust the main driving path line’s thickness.

  **How to Enable:** *Settings → Visuals → Model UI → Path Width*
</details>

<details>
<summary>Pedals Indicator</summary>

  **Description:** On-screen icons light up when gas or brake pedals are pressed (manual or system).

  **How to Enable:** *Settings → Visuals → Custom On-road UI* → Pedal Indicators
</details>

<details>
<summary>Random Events</summary>

  **Description:** Enables occasional fun Easter-egg graphics/sounds.

  **How to Enable:** *Settings → Visuals → Custom Themes → Random Events*
</details>

<details>
<summary>Road Edges Width</summary>

  **Description:** Adjust road-edge line thickness in the visualization.

  **How to Enable:** *Settings → Visuals → Model UI → Road Edge Line Width*
</details>

<details>
<summary>Road Names Display</summary>

  **Description:** Shows the current road/street name on screen.

  **How to Enable:** *Settings → Visuals → Custom On-road UI* → Show Road Name
</details>

<details>
<summary>Screen Brightness (Off-road)</summary>

  **Description:** Set screen brightness when parked or in menus.

  **How to Enable:** *Settings → Device → Screen → Brightness (Off-road)*
</details>

<details>
<summary>Screen Brightness (On-road)</summary>

  **Description:** Set screen brightness while driving.

  **How to Enable:** *Settings → Device → Screen → Brightness (On-road)*
</details>

<details>
<summary>Screen Recorder</summary>

  **Description:** Records the on-screen UI during drives.

  **How to Enable:** *Settings → Device → Screen → Enable Screen Recorder*
</details>

<details>
<summary>Screen Timeout (Off-road)</summary>

  **Description:** Auto-turns off the screen after X minutes when off-road.

  **How to Enable:** *Settings → Device → Screen → Screen Timeout (Off-road)*
</details>

<details>
<summary>Screen Timeout (On-road)</summary>

  **Description:** Auto-turns off the screen after X minutes while driving (use with Standby Mode).

  **How to Enable:** *Settings → Device → Screen → Screen Timeout (On-road)*
</details>

<details>
<summary>Standby Mode</summary>

  **Description:** Lets the screen sleep but wakes instantly for alerts/engagement changes.

  **How to Enable:** *Settings → Device → Screen → Standby Mode*
</details>

<details>
<summary>Steering Wheel Icon (Custom)</summary>

  **Description:** Swap steering-wheel graphic; icon rotates with your wheel in real time.

  **How to Enable:** *Settings → Visuals → Custom On-road UI* → Custom Steering Icon
</details>

<details>
<summary>Stopped Timer</summary>

  **Description:** Shows how long you’ve been stopped at a light or in traffic.

  **How to Enable:** *Settings → Visuals → Quality of Life* → Stopped Timer
</details>

<details>
<summary>Turn Signal Animation</summary>

  **Description:** Adds animated blinker graphics (theme dependent).

  **How to Enable:** *Settings → Visuals → Custom Themes → Animated Turn Signals*
</details>

<details>
<summary>Wheel Speed in UI</summary>

  **Description:** Uses wheel-sensor speed (more accurate) for the UI and logic.

  **How to Enable:** *Settings → Visuals → Quality of Life* → Use Wheel Speed
</details>

---

# 🧰 How to Install

Easiest way to install FrogPilot is via this URL at the installation screen:

```
frogpilot.download
```

DO NOT install the **"FrogPilot-Development"** branch. I'm constantly breaking things on there, so unless you don't want to use openpilot, NEVER install it!

![](https://i.imgur.com/swr0kqJ.png)

---

# 🐞 Bug reports / Feature Requests

If you encounter any issues or bugs while using FrogPilot, or if you have any suggestions for new features or improvements, please don't hesitate to post about it on the Discord! I'm always looking for ways to improve the fork and provide a better experience for everyone!

To report a bug or request a new feature, make a post in the #bug-reports or #feature-requests channel respectively on the FrogPilot Discord. Please provide as much detail as possible about the issue you're experiencing or the feature you'd like to see added. Photos, videos, log files, or other relevant information are very helpful!

I will do my best to respond to bug reports and feature requests in a timely manner, but please understand that I may not be able to address every request immediately. Your feedback and suggestions are valuable, and I appreciate your help in making FrogPilot the best it can be!

---

# 📱 Discord

[Join the FrogPilot Community Discord!](https://discord.gg/frogpilot)

---

# 📋 Credits

* [AlexandreSato](https://github.com/AlexandreSato)
* [Crwusiz](https://github.com/crwusiz)
* [DragonPilot](https://github.com/dragonpilot-community)
* [ErichMoraga](https://github.com/ErichMoraga)
* [Garrettpall](https://github.com/garrettpall)
* [Mike8643](https://github.com/mike8643)
* [Neokii](https://github.com/Neokii)
* [OPGM](https://github.com/opgm)
* [OPKR](https://github.com/openpilotkr)
* [Pfeiferj](https://github.com/pfeiferj)
* [ServerDummy](https://github.com/ServerDummy)
* [Twilsonco](https://github.com/twilsonco)

---

## Star History

[![Star History Chart](https://api.star-history.com/svg?repos=FrogAi/FrogPilot&type=Timeline)](https://www.star-history.com/#FrogAi/FrogPilot&Timeline)

---

# ⚖️ Licensing

openpilot is released under the MIT license. Some parts of the software are released under other licenses as specified.

Any user of this software shall indemnify and hold harmless Comma.ai, Inc. and its directors, officers, employees, agents, stockholders, affiliates, subcontractors and customers from and against all allegations, claims, actions, suits, demands, damages, liabilities, obligations, losses, settlements, judgments, costs and expenses (including without limitation attorneys’ fees and costs) which arise out of, relate to or result from any use of this software by user.

**THIS IS ALPHA QUALITY SOFTWARE FOR RESEARCH PURPOSES ONLY. THIS IS NOT A PRODUCT.
YOU ARE RESPONSIBLE FOR COMPLYING WITH LOCAL LAWS AND REGULATIONS.
NO WARRANTY EXPRESSED OR IMPLIED.**

---

<img src="https://d1qb2nb5cznatu.cloudfront.net/startups/i/1061157-bc7e9bf3b246ece7322e6ffe653f6af8-medium_jpg.jpg?buster=1458363130" width="75"></img> <img src="https://cdn-images-1.medium.com/max/1600/1*C87EjxGeMPrkTuVRVWVg4w.png" width="225"></img>
