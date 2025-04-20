#!/usr/bin/env python3
from datetime import datetime, timedelta, timezone
from flask import Flask, Response, jsonify, render_template, request, send_file, send_from_directory
from io import BytesIO
from pathlib import Path

import base64
import json
import os
import re
import requests
import secrets
import signal
import subprocess
import time

from cereal import car
from openpilot.common.realtime import DT_HW
from openpilot.selfdrive.car.toyota.carcontroller import LOCK_CMD, UNLOCK_CMD
from openpilot.system.hardware import HARDWARE, PC
from openpilot.system.hardware.hw import Paths
from openpilot.system.version import get_build_metadata
from panda import Panda

from openpilot.frogpilot.common.frogpilot_utilities import delete_file, run_cmd
from openpilot.frogpilot.common.frogpilot_variables import ERROR_LOGS_PATH, EXCLUDED_KEYS, frogpilot_default_params, params, params_cache, update_frogpilot_toggles
from openpilot.frogpilot.system.the_pond import helpers, utilities

FOOTAGE_PATHS = [
  Paths.log_root(HD=True, raw=True),
  Paths.log_root(konik=True, raw=True),
  Paths.log_root(raw=True),
]

KEYS = {
  "amap1": ("amap1", "", "AMapKey1", "Amap key #1", 39),
  "amap2": ("amap2", "", "AMapKey2", "Amap key #2", 39),
  "public": ("public", "pk.", "MapboxPublicKey", "Public key", 80),
  "secret": ("secret", "sk.", "MapboxSecretKey", "Secret key", 80),
}

TMUX_LOGS_PATH = "/data/tmux_logs/"

XOR_KEY = "s8#pL3*Xj!aZ@dWq"

def setup(app):
  @app.errorhandler(404)
  def not_found(_):
    return render_template("index.html")

  @app.route("/", methods=["GET"])
  def index():
    return render_template("index.html")

  @app.route("/api/doors_available", methods=["GET"])
  def doors_available():
    with car.CarParams.from_bytes(params.get("CarParamsPersistent")) as cp_reader:
      CP = cp_reader.as_builder()

    return jsonify({"result": HARDWARE.get_device_type() != "tici" and CP.carName == "toyota"})

  @app.route("/api/doors/lock", methods=["POST"])
  def lock_doors():
    for _ in range(3):
      with Panda() as panda:
        panda.set_safety_mode(panda.SAFETY_TOYOTA)
        panda.can_send(0x750, LOCK_CMD, 0)
        panda.send_heartbeat()
    return { "message": "Doors locked!" }

  @app.route("/api/doors/unlock", methods=["POST"])
  def unlock_doors():
    for _ in range(3):
      with Panda() as panda:
        panda.set_safety_mode(panda.SAFETY_TOYOTA)
        panda.can_send(0x750, UNLOCK_CMD, 0)
        panda.send_heartbeat()
    return { "message": "Doors unlocked!" }

  @app.route("/api/error_logs", methods=["GET"])
  def get_error_logs():
    if request.accept_mimetypes["text/html"]:
      return render_template("v2/error-logs.jinja", active="error_logs")

    if request.accept_mimetypes["application/json"]:
      return helpers.list_file(ERROR_LOGS_PATH), 200

  @app.route("/api/error_logs/<filename>", methods=["DELETE"])
  def delete_error_log(filename):
    delete_file(os.path.join(ERROR_LOGS_PATH, filename))
    return {"message": "Error log deleted!"}

  @app.route("/api/error_logs/<filename>", methods=["GET"])
  def get_error_log(filename):
    with open(os.path.join(ERROR_LOGS_PATH, filename)) as file:
      return file.read(), 200, {"Content-Type": "text/plain; charset=utf-8"}

  @app.route("/api/navigation", methods=["DELETE"])
  def clear_navigation():
    params.remove("NavDestination")
    return {"message": "Destination cleared"}

  @app.route("/api/navigation", methods=["GET"])
  def navigation():
    last_position = json.loads(
      params.get("LastGPSPosition", encoding="utf8") or
      "{\"latitude\": 51.276824158421331, \"longitude\": 30.221928335547232, \"altitude\": 111.0}"
    )

    return {
      "amap1Key": params_cache.get("AMapKey1", encoding="utf8") or "",
      "amap2Key": params_cache.get("AMapKey2", encoding="utf8") or "",
      "destination": params.get("NavDestination", encoding="utf8") or "",
      "isMetric": params.get_bool("IsMetric"),
      "lastPosition": {
        "latitude": str(last_position["latitude"]),
        "longitude": str(last_position["longitude"])
      },
      "mapboxPublic": params_cache.get("MapboxPublicKey", encoding="utf8") or "",
      "mapboxSecret": params_cache.get("MapboxSecretKey", encoding="utf8") or "",
      "previousDestinations": params.get("ApiCache_NavDestinations", encoding="utf8") or "",
    }

  @app.route("/api/navigation", methods=["POST"])
  def set_navigation():
    params.remove("NavDestination")

    time.sleep(1)

    params.put("NavDestination", json.dumps(request.json))
    return {"message": "Destination set"}

  @app.route("/api/navigation/favorite", methods=["DELETE"])
  def remove_favorite_destination():
    to_remove = request.json

    existing = json.loads(params.get("FavoriteDestinations", encoding="utf8") or "[]")
    existing = [
      favorite for favorite in existing
      if not (
        favorite.get("name") == to_remove.get("name") and
        favorite.get("latitude") == to_remove.get("latitude") and
        favorite.get("longitude") == to_remove.get("longitude") and
        favorite.get("routeId") == to_remove.get("routeId")
      )
    ]
    params.put("FavoriteDestinations", json.dumps(existing))
    return jsonify(message="Destination removed from favorites!")

  @app.route("/api/navigation/favorite", methods=["GET"])
  def list_favorite_destinations():
    favorites = json.loads(params.get("FavoriteDestinations", encoding="utf8") or "[]")
    return jsonify(favorites=favorites)

  @app.route("/api/navigation/favorite", methods=["POST"])
  def add_favorite_destination():
    existing = json.loads(params.get("FavoriteDestinations", encoding="utf8") or "[]")

    favorite_to_add = request.json
    if favorite_to_add not in existing:
      existing.append(favorite_to_add)

    params.put("FavoriteDestinations", json.dumps(existing))
    return {"message": "Destination added to favorites!"}

  @app.route("/api/navigation_key", methods=["DELETE"])
  def delete_navigation_key():
    meta = KEYS.get(request.args.get("type"))
    params_cache.remove(meta[2])
    return jsonify(message=f"{meta[3]} deleted successfully!")

  @app.route("/api/navigation_key", methods=["POST"])
  def set_navigation_keys():
    data = request.get_json() or {}

    saved = []
    for meta in KEYS.values():
      raw = (data.get(meta[0]) or "").strip()
      if not raw:
        continue

      full = raw if raw.startswith(meta[1]) else meta[1] + raw
      if len(full) < meta[4]:
        return jsonify(error=f"{meta[3]} is invalid or too short..."), 400

      params_cache.put(meta[2], full)

      saved.append(meta[3])

    if not saved:
      return jsonify(error="Nothing to update"), 400

    return jsonify(message=f"{', '.join(saved)} saved successfully!")

  @app.route("/api/params", methods=["GET"])
  def get_param():
    return params.get(request.args.get("key")) or "", 200

  @app.route("/api/routes", methods=["GET"])
  def list_routes():
    routes = []
    for footage_path in FOOTAGE_PATHS:
      for name in helpers.get_routes_names(footage_path):
        path = f"{footage_path}{name}--0"
        qcamera = f"{path}/qcamera.ts"

        utilities.video_to_gif(qcamera, f"{path}/preview.gif")
        utilities.video_to_png(qcamera, f"{path}/preview.png")

        routes.append({
          "name": name,
          "gif": f"/thumbnails/{name}--0/preview.gif",
          "png": f"/thumbnails/{name}--0/preview.png",
        })
    return routes, 200

  @app.route("/api/routes/<name>", methods=["GET"])
  def get_route(name):
    for footage_path in FOOTAGE_PATHS:
      base_path = f"{footage_path}{name}--0"
      if os.path.exists(base_path):
        segments = helpers.get_segments_in_route(name, footage_path)
        segment_urls = [f"/video/{segment}" for segment in segments]

        if not segment_urls:
          break

        last_segment_path = f"{footage_path}{name}--{len(segment_urls)-1}/qcamera.ts"
        last_duration = utilities.get_video_duration(last_segment_path)
        total_duration = round(last_duration + ((len(segment_urls) - 1) * 60))

        return {
          "name": name,
          "segment_urls": segment_urls,
          "total_duration": total_duration,
          "date": datetime.strptime(name, "%Y-%m-%d--%H-%M-%S"),
          "available_cameras": utilities.get_available_cameras(base_path),
        }, 200

    return {"error": "Route not found"}, 404

  @app.route("/api/speed_limits", methods=["GET"])
  def speed_limits():
    data = json.loads(params.get("SpeedLimitsFiltered") or "[]")
    current_time = (datetime.now(timezone.utc) - timedelta(days=6, hours=23)).isoformat()
    data = [{**e, "last_vetted": current_time} for e in data]
    params.put("SpeedLimitsFiltered", json.dumps(data))

    buffer = BytesIO(json.dumps(data, indent=2).encode())
    buffer.seek(0)
    return send_file(buffer, as_attachment=True, download_name="speed_limits.json", mimetype="application/json")

  @app.route("/api/stats", methods=["GET"])
  def get_stats():
    build_metadata = get_build_metadata()

    short_branch = build_metadata.channel
    if short_branch == "FrogPilot-Development":
      env = "Development"
    elif build_metadata.release_channel:
      env = "Release"
    elif short_branch == "FrogPilot-Testing":
      env = "Testing"
    elif build_metadata.tested_channel:
      env = "Staging"
    else:
      env = short_branch

    return {
      "diskUsage": utilities.get_disk_usage(),
      "driveStats": utilities.get_drive_stats(),
      "firehoseStats": {
        "segments": requests.get(
          f"https://api.comma.ai/v1/devices/{params.get('DongleId', encoding='utf8')}/firehose_stats",
          timeout=10
        ).json().get("firehose", 0)
      },
      "softwareInfo": {
        "branchName": build_metadata.channel,
        "buildEnvironment": env,
        "commitHash": build_metadata.openpilot.git_commit,
        "forkMaintainer": utilities.get_repo_owner(build_metadata.openpilot.git_normalized_origin),
        "updateAvailable": "Yes" if params.get_bool("UpdaterFetchAvailable") else "No",
        "versionDate": utilities.format_git_date(build_metadata.openpilot.git_commit_date),
      },
    }

  @app.route("/api/tailscale/installed", methods=["GET"])
  def tailscale_installed():
    base = "/data/tailscale"
    tailscale_binary = f"{base}/tailscale"
    tailscaled_binary = f"{base}/tailscaled"
    systemd_unit = "/etc/systemd/system/tailscaled.service"

    if os.path.exists(tailscale_binary) and os.path.exists(tailscaled_binary) and os.path.exists(systemd_unit):
      return jsonify({"installed": True})

    result = subprocess.run(["which", "tailscale"], capture_output=True, text=True)
    if result.returncode == 0:
      return jsonify({"installed": True})

    return jsonify({"installed": False})

  @app.route("/api/tailscale/setup", methods=["POST"])
  def tailscale_setup():
    arch = "arm64"
    base = "/data/tailscale"

    result = subprocess.run(
      "curl -s https://pkgs.tailscale.com/stable/ | grep -oP 'tailscale_\\K[0-9]+\\.[0-9]+\\.[0-9]+' | sort -V | tail -1",
      shell=True, capture_output=True, text=True
    )
    version = result.stdout.strip() or "1.84.0"

    bin_dir = f"{base}/tailscale_{version}_{arch}"
    state = f"{base}/state"
    socket = f"{base}/tailscaled.sock"
    tgz_path = f"{base}/tailscale.tgz"
    tgz_url = f"https://pkgs.tailscale.com/stable/tailscale_{version}_{arch}.tgz"

    os.makedirs(state, exist_ok=True)

    run_cmd(["curl", "-fsSL", tgz_url, "-o", tgz_path], "Downloaded Tailscale archive.", "Failed to download Tailscale archive.")
    run_cmd(["tar", "xzf", tgz_path, "-C", base], "Extracted Tailscale archive.", "Failed to extract Tailscale archive.")
    run_cmd(["cp", f"{bin_dir}/tailscale", f"{base}/tailscale"], "Copied tailscale binary.", "Failed to copy tailscale binary.")
    run_cmd(["cp", f"{bin_dir}/tailscaled", f"{base}/tailscaled"], "Copied tailscaled binary.", "Failed to copy tailscaled binary.")
    run_cmd(["chmod", "+x", f"{base}/tailscale", f"{base}/tailscaled"], "Made binaries executable.", "Failed to chmod binaries.")

    systemd_unit = f"""[Unit]
    Description=Tailscale node agent
    After=network.target

    [Service]
    ExecStart={base}/tailscaled \\
      --tun=userspace-networking \\
      --socks5-server=localhost:1055 \\
      --state={state}/tailscaled.state \\
      --socket={socket} \\
      --statedir={state}
    Restart=on-failure
    RestartSec=5

    [Install]
    WantedBy=multi-user.target
    """
    unit_tmp = f"{base}/tailscaled.service"
    with open(unit_tmp, "w") as f:
      f.write(systemd_unit)

    run_cmd(["sudo", "mount", "-o", "remount,rw", "/"], "Remounted / as read-write.", "Failed to remount / as read-write.")
    run_cmd(["sudo", "install", "-m", "644", unit_tmp, "/etc/systemd/system/tailscaled.service"], "Installed systemd unit.", "Failed to install systemd unit.")
    run_cmd(["sudo", "systemctl", "daemon-reload"], "Reloaded systemd daemon.", "Failed to reload systemd daemon.")
    run_cmd(["sudo", "systemctl", "enable", "/etc/systemd/system/tailscaled.service"], "Enabled tailscaled service.", "Failed to enable tailscaled service.")
    run_cmd(["sudo", "systemctl", "restart", "tailscaled"], "Started tailscaled service.", "Failed to start tailscaled service.")

    proc = subprocess.Popen(
      ["sudo", f"{base}/tailscale", "--socket", socket, "up", "--hostname", f"{HARDWARE.get_device_type()}-the-pond"],
      stdout=subprocess.PIPE,
      stderr=subprocess.STDOUT,
      text=True,
      preexec_fn=os.setsid
    )

    auth_url = None
    for line in proc.stdout:
      match = re.search(r"https://login\.tailscale\.com/\S+", line)
      if match and not auth_url:
        auth_url = match.group(0)
        run_cmd(["sudo", "kill", "-TERM", f"-{proc.pid}"], "Sent SIGTERM to Tailscale setup process.", "Failed to send SIGTERM to Tailscale setup process.")
        proc.wait(timeout=5)
        break

    return jsonify({
      "message": "Tailscale setup started. Please authenticate in your browser.",
      "auth_url": auth_url
    }), 200

  @app.route("/api/tailscale/uninstall", methods=["POST"])
  def tailscale_uninstall():
    base = "/data/tailscale"
    state = f"{base}/state"
    unit_path = "/etc/systemd/system/tailscaled.service"
    local_unit = f"{base}/tailscaled.service"

    run_cmd(["sudo", "systemctl", "stop", "tailscaled"], "Stopped tailscaled.", "Failed to stop tailscaled.")
    run_cmd(["sudo", "systemctl", "disable", "tailscaled"], "Disabled tailscaled.", "Failed to disable tailscaled.")
    run_cmd(["sudo", "mount", "-o", "remount,rw", "/"], "Remounted / as read-write.", "Failed to remount /.")

    if os.path.exists(unit_path):
      run_cmd(["sudo", "rm", unit_path], "Removed systemd unit file.", "Failed to remove systemd unit file.")
      run_cmd(["sudo", "systemctl", "daemon-reload"], "Reloaded systemd daemon.", "Failed to reload systemd.")

    delete_file(local_unit)

    for filename in ["tailscale", "tailscaled", "tailscale.tgz"]:
      delete_file(os.path.join(base, filename))

    for item in os.listdir(base):
      if item.startswith("tailscale_"):
        item_path = os.path.join(base, item)
        if os.path.isdir(item_path):
          run_cmd(["sudo", "rm", "-rf", item_path], f"Removed {item_path}.", f"Failed to remove {item_path}.")

    if os.path.exists(state):
      run_cmd(["sudo", "rm", "-rf", state], "Removed tailscale state dir.", "Failed to remove tailscale state dir.")

    if os.path.exists(base):
      run_cmd(["sudo", "rm", "-rf", base], "Removed tailscale dir.", "Failed to remove tailscale dir.")

    return jsonify({"message": "Tailscale uninstalled!"}), 200

  @app.route("/api/tmux_log/capture", methods=['POST'])
  def capture_tmux_log_route():
    os.makedirs(TMUX_LOGS_PATH, exist_ok=True)

    timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    log_filename = f"tmux_log_{timestamp}.json"
    log_path = os.path.join(TMUX_LOGS_PATH, log_filename)

    run_cmd(["tmux", "capture-pane", "-J", "-S", "-"], "Captured tmux pane.", "Failed to capture tmux pane.")
    result = subprocess.run(["tmux", "show-buffer"], capture_output=True, text=True, check=True)

    with open(log_path, "w", encoding="utf-8") as log_file:
      log_file.write(result.stdout)

    run_cmd(["tmux", "delete-buffer"], "Deleted tmux buffer.", "Failed to delete tmux buffer.")

    return jsonify({"message": "Captured console log successfully!", "log_file": log_filename}), 200

  @app.route("/api/tmux_log/delete/<filename>", methods=["DELETE"])
  def delete_tmux_log(filename):
    file_path = Path(TMUX_LOGS_PATH) / filename
    if file_path.exists():
      delete_file(file_path)
      return jsonify({"message": f"{filename} deleted"}), 200

    return jsonify({"error": "File not found"}), 404

  @app.route("/api/tmux_log/delete_all", methods=["DELETE"])
  def delete_all_tmux_logs():
    log_path = Path(TMUX_LOGS_PATH)

    if log_path.exists():
      delete_file(log_path)

    log_path.mkdir(parents=True, exist_ok=True)

    return jsonify({"message": "All tmux logs deleted and folder recreated"}), 200

  @app.route("/api/tmux_log/download/<filename>", methods=["GET"])
  def download_tmux_log(filename):
    return send_from_directory(TMUX_LOGS_PATH, filename, as_attachment=True)

  @app.route("/api/tmux_log/list", methods=["GET"])
  def list_tmux_logs():
    os.makedirs(TMUX_LOGS_PATH, exist_ok=True)
    files = sorted(Path(TMUX_LOGS_PATH).glob("tmux_log_*.json"), reverse=True)
    return jsonify([file.name for file in files])

  @app.route("/api/tmux_log/live", methods=["GET"])
  def stream_tmux_log():
    if subprocess.run(["tmux", "has-session", "-t", "comma"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL).returncode != 0:
      run_cmd(["tmux", "new-session", "-d", "-s", "comma", "-x", "240", "-y", "70", "bash"], "Started tmux session", "Failed to start tmux session")
    else:
      run_cmd(["tmux", "resize-window", "-t", "comma:0", "-x", "240", "-y", "70"], "Resized tmux window", "Failed to resize tmux window")

    def generate():
      while True:
        output = subprocess.check_output(["tmux", "capture-pane", "-t", "comma:0", "-p", "-S", "-1000"], text=True)
        yield "data: " + "\n".join(reversed(output.splitlines())).replace("\n", "\ndata: ") + "\n\n"

        time.sleep(0.1)

    return Response(generate(), mimetype="text/event-stream")

  def xor_encrypt_decrypt(data: str, key: str) -> str:
    return ''.join(chr(ord(c) ^ ord(key[i % len(key)])) for i, c in enumerate(data))

  def encode_parameters(params_dict):
    serialized_data = json.dumps(params_dict)
    obfuscated_data = xor_encrypt_decrypt(serialized_data, XOR_KEY)
    encoded_data = base64.b64encode(obfuscated_data.encode("utf-8")).decode("utf-8")
    return encoded_data

  def decode_parameters(encoded_string):
    obfuscated_data = base64.b64decode(encoded_string.encode("utf-8")).decode("utf-8")
    decrypted_data = xor_encrypt_decrypt(obfuscated_data, XOR_KEY)
    return json.loads(decrypted_data)

  @app.route("/api/toggles/backup", methods=["POST"])
  def backup_toggle_values():
    toggle_values = {
      key: (params.get(key).decode('utf-8') if isinstance(params.get(key), bytes) else params.get(key) or "0")
      for key, _, _ in frogpilot_default_params
      if key not in EXCLUDED_KEYS
    }

    encoded = encode_parameters(toggle_values)
    wrapped = json.dumps({ "data": encoded }, indent=2)

    buffer = BytesIO(wrapped.encode("utf-8"))
    buffer.seek(0)
    return send_file(buffer, as_attachment=True, download_name="toggle_backup.json", mimetype="application/json")

  @app.route("/api/toggles/restore", methods=["POST"])
  def restore_toggle_values():
    request_data = request.get_json()

    if not request_data or "data" not in request_data:
      return jsonify({ "success": False, "message": "Missing 'data' in request." }), 400

    allowed_keys = {key for key, _, _ in frogpilot_default_params if key not in EXCLUDED_KEYS}

    toggle_values = decode_parameters(request_data['data'])
    for key, value in toggle_values.items():
      if key in allowed_keys:
        params.put(key, value)

    update_frogpilot_toggles()
    return jsonify({ "success": True, "message": "Toggles restored!" })

  @app.route("/api/toggles/reset_default", methods=["POST"])
  def reset_toggle_values():
    params.put_bool("DoToggleReset", True)
    HARDWARE.reboot()

  @app.route("/api/toggles/reset_stock", methods=["POST"])
  def reset_toggle_values_to_stock():
    params.put_bool("DoToggleResetStock", True)
    HARDWARE.reboot()

  @app.route("/mapbox-help/<path:filename>", methods=["GET"])
  def serve_mapbox_help(filename):
    return send_from_directory("/data/openpilot/frogpilot/navigation/navigation_training", filename)

  @app.route("/playground", methods=["GET"])
  def playground():
    return render_template("playground.html")

  @app.route("/thumbnails/<path:file_path>", methods=["GET"])
  def get_thumbnail(file_path):
    for footage_path in FOOTAGE_PATHS:
      try:
        return send_from_directory(footage_path, file_path, as_attachment=True)
      except FileNotFoundError:
        continue
    return {"error": "Thumbnail not found"}, 404

  @app.route("/video/<path>", methods=["GET"])
  def get_video(path):
    camera = request.args.get("camera")
    filename = {"driver": "dcamera.hevc", "wide": "ecamera.hevc"}.get(camera, "qcamera.ts")

    for footage_path in FOOTAGE_PATHS:
      filepath = f"{footage_path}{path}/{filename}"
      if os.path.exists(filepath):
        process = helpers.ffmpeg_mp4_wrap_process_builder(filepath)
        return Response(process.stdout.read(), status=200, mimetype="video/mp4")

    return {"error": "Video not found"}, 404


def main():
  app = Flask(__name__, static_folder="assets", static_url_path="/assets")
  setup(app)

  debug = PC or __package__ == "the_pond"
  port = 8084 if debug else 8083

  if debug:
    print("\"The Pond\" is not running on a comma device, enabling debug mode")

  app.secret_key = secrets.token_hex(32)
  app.run(host="0.0.0.0", port=port, debug=debug)

if __name__ == "__main__":
  main()
