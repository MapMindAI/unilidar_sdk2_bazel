#!/usr/bin/env python3
import json
import os
import re
import shlex
import subprocess
import sys
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse


REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_COMPOSE_NAME = os.environ.get("UNILIDAR_COMPOSE_NAME", "unilidar_collection")
DEFAULT_CONTAINER_NAME = os.environ.get("UNILIDAR_CONTAINER_NAME", "UniLidarSdk")
DEFAULT_HOST = os.environ.get("UNILIDAR_WEB_HOST", "0.0.0.0")
DEFAULT_PORT = int(os.environ.get("UNILIDAR_WEB_PORT", "8080"))
START_SCRIPT = Path(
    os.environ.get(
        "UNILIDAR_START_SCRIPT",
        REPO_ROOT / "docker_compose" / "unilidar_mapping" / "arm64_start_unilidar.sh",
    )
)
STOP_SCRIPT = Path(
    os.environ.get(
        "UNILIDAR_STOP_SCRIPT",
        REPO_ROOT / "docker_compose" / "unilidar_mapping" / "arm64_stop_unilidar.sh",
    )
)
COPY_SCRIPT = Path(
    os.environ.get(
        "UNILIDAR_COPY_SCRIPT",
        REPO_ROOT / "docker_compose" / "unilidar_mapping" / "copy_to_drive.sh",
    )
)
ROS2_SETUP_OVERRIDE = os.environ.get("UNILIDAR_ROS2_SETUP", "")
ROS2_DISTRO_OVERRIDE = os.environ.get("UNILIDAR_ROS2_DISTRO", "")
ROS2_DISTRO_CANDIDATES = (
    "jazzy",
    "iron",
    "humble",
    "galactic",
    "foxy",
    "eloquent",
    "dashing",
    "rolling",
)
COMPOSE_PARAM_NAMES = ("alpha_bais_bias", "range_fix_a0", "range_fix_a1")

PARAM_LINE_RE = re.compile(
    r"(?P<prefix>--alpha_bais_bias=)(?P<alpha_bais_bias>\S+)"
    r"(?P<mid1>\s+--range_fix_a0=)(?P<range_fix_a0>\S+)"
    r"(?P<mid2>\s+(?:--)?range_fix_a1=)(?P<range_fix_a1>\S+)"
)


INDEX_HTML = """<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>UniLidar Remote Control</title>
  <style>
    :root {
      --bg: #0d1117;
      --panel: #161b22;
      --panel-2: #1f2937;
      --text: #e6edf3;
      --muted: #9da7b3;
      --accent: #3fb950;
      --danger: #f85149;
      --border: #30363d;
    }

    * { box-sizing: border-box; }
    body {
      margin: 0;
      padding: 24px;
      font-family: "Segoe UI", system-ui, sans-serif;
      background: linear-gradient(180deg, #0b1220 0%, var(--bg) 100%);
      color: var(--text);
    }
    .layout {
      max-width: 1080px;
      margin: 0 auto;
    }
    .card {
      background: rgba(22, 27, 34, 0.92);
      border: 1px solid var(--border);
      border-radius: 16px;
      padding: 20px;
      box-shadow: 0 20px 60px rgba(0, 0, 0, 0.25);
      backdrop-filter: blur(8px);
    }
    h1 {
      margin: 0 0 8px;
      font-size: 32px;
    }
    p {
      color: var(--muted);
      margin-top: 0;
    }
    .toolbar {
      display: flex;
      gap: 12px;
      flex-wrap: wrap;
      margin: 20px 0;
    }
    button {
      border: 0;
      border-radius: 999px;
      padding: 12px 18px;
      font-size: 15px;
      font-weight: 600;
      color: white;
      cursor: pointer;
      transition: transform 120ms ease, box-shadow 120ms ease, filter 120ms ease;
      box-shadow: 0 10px 24px rgba(0, 0, 0, 0.2);
    }
    button:hover {
      transform: translateY(-1px);
      filter: brightness(1.05);
    }
    button:active {
      transform: translateY(1px) scale(0.98);
      box-shadow: 0 4px 12px rgba(0, 0, 0, 0.18);
    }
    button:disabled {
      opacity: 0.55;
      cursor: not-allowed;
      transform: none;
      box-shadow: none;
      filter: none;
    }
    .start { background: var(--accent); color: #051b11; }
    .stop { background: var(--danger); }
    .copy { background: #58a6ff; color: #03111f; }
    .ghost {
      background: transparent;
      color: var(--text);
      border: 1px solid var(--border);
      box-shadow: none;
    }
    .status-grid {
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(220px, 1fr));
      gap: 12px;
      margin-bottom: 20px;
    }
    .status-box {
      background: var(--panel-2);
      border: 1px solid var(--border);
      border-radius: 12px;
      padding: 14px;
    }
    .status-box .label {
      display: block;
      color: var(--muted);
      font-size: 13px;
      margin-bottom: 8px;
    }
    .status-box .value {
      font-size: 16px;
      font-weight: 600;
      word-break: break-word;
    }
    .ok { color: #7ee787; }
    .bad { color: #ff7b72; }
    .logs {
      min-height: 420px;
      max-height: 68vh;
      overflow: auto;
      white-space: pre-wrap;
      background: #0b0f14;
      border: 1px solid var(--border);
      border-radius: 12px;
      padding: 16px;
      margin: 0;
      font-family: "SFMono-Regular", Consolas, monospace;
      font-size: 13px;
      line-height: 1.45;
    }
    .message {
      margin: 0 0 16px;
      min-height: 22px;
      color: var(--muted);
    }
    .param-grid {
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(220px, 1fr));
      gap: 12px;
      margin: 16px 0 8px;
    }
    .field {
      display: flex;
      flex-direction: column;
      gap: 8px;
    }
    .field label {
      font-size: 13px;
      color: var(--muted);
    }
    .field input {
      width: 100%;
      border: 1px solid var(--border);
      border-radius: 10px;
      background: #0b0f14;
      color: var(--text);
      padding: 12px 14px;
      font-size: 15px;
      outline: none;
    }
    .field input:focus {
      border-color: #58a6ff;
      box-shadow: 0 0 0 3px rgba(88, 166, 255, 0.18);
    }
    .panel-title {
      margin: 0 0 6px;
      font-size: 18px;
    }
    .panel-note {
      margin: 6px 0 0;
      font-size: 13px;
      color: var(--muted);
    }
  </style>
</head>
<body>
  <div class="layout">
    <div class="card">
      <h1>UniLidar Remote Control</h1>
      <p>Start or stop the compose stack and watch the live debug output from <code>UniLidarSdk</code>.</p>

      <div class="status-box" style="margin-bottom: 20px;">
        <h2 class="panel-title">Calibration Parameters</h2>
        <p class="panel-note">These values are written back into the compose file. Restart the stack after saving to apply them.</p>
        <div class="param-grid">
          <div class="field">
            <label for="alphaBaisBias">alpha_bais_bias</label>
            <input id="alphaBaisBias" type="number" step="any" inputmode="decimal">
          </div>
          <div class="field">
            <label for="rangeFixA0">range_fix_a0</label>
            <input id="rangeFixA0" type="number" step="any" inputmode="decimal">
          </div>
          <div class="field">
            <label for="rangeFixA1">range_fix_a1</label>
            <input id="rangeFixA1" type="number" step="any" inputmode="decimal">
          </div>
        </div>
        <div class="toolbar" style="margin-bottom: 0;">
          <button class="ghost" id="defaultParamsBtn">Load Defaults</button>
          <button class="ghost" id="zeroParamsBtn">All Zeros</button>
          <button class="copy" id="saveParamsBtn">Save Parameters</button>
        </div>
      </div>

      <div class="status-grid">
        <div class="status-box">
          <span class="label">Container Status</span>
          <div class="value" id="runningStatus">Unknown</div>
        </div>
        <div class="status-box">
          <span class="label">Container Name</span>
          <div class="value" id="containerName"></div>
        </div>
        <div class="status-box">
          <span class="label">Compose File</span>
          <div class="value" id="composeFile"></div>
        </div>
        <div class="status-box">
          <span class="label">ROS 2 Distro</span>
          <div class="value" id="ros2Distro">Unknown</div>
        </div>
      </div>

      <div class="toolbar">
        <button class="start" id="startBtn">Start UniLidar</button>
        <button class="stop" id="stopBtn">Stop UniLidar</button>
        <button class="ghost" id="refreshBtn">Refresh Logs</button>
      </div>

      <div class="message" id="message"></div>

      <pre class="logs" id="logs">Loading logs...</pre>

      <div class="status-box" style="margin-top: 20px;">
        <span class="label">Copy Controls</span>
        <div class="toolbar" style="margin: 12px 0 0;">
          <button class="copy" id="copyBtn">Copy to Drive</button>
          <button class="ghost" id="detectRosBtn">Detect ROS 2</button>
          <button class="ghost" id="topicsBtn">List Topics</button>
        </div>
      </div>

      <div class="status-box" style="margin-top: 20px;">
        <span class="label">ROS 2 Finder</span>
        <pre class="logs" id="ros2Info" style="min-height: 140px; max-height: 220px; margin-top: 0;">ROS 2 not detected yet.</pre>
      </div>

      <div class="status-box" style="margin-top: 20px;">
        <span class="label">Copy Result Log</span>
        <pre class="logs" id="copyLogs" style="min-height: 180px; max-height: 260px; margin-top: 0;">No copy has run yet.</pre>
      </div>

      <div class="status-box" style="margin-top: 20px;">
        <span class="label">ROS 2 Topic List</span>
        <pre class="logs" id="topicLogs" style="min-height: 180px; max-height: 260px; margin-top: 0;">No topic list has run yet.</pre>
      </div>
    </div>
  </div>

  <script>
    const startBtn = document.getElementById("startBtn");
    const stopBtn = document.getElementById("stopBtn");
    const copyBtn = document.getElementById("copyBtn");
    const detectRosBtn = document.getElementById("detectRosBtn");
    const topicsBtn = document.getElementById("topicsBtn");
    const refreshBtn = document.getElementById("refreshBtn");
    const defaultParamsBtn = document.getElementById("defaultParamsBtn");
    const zeroParamsBtn = document.getElementById("zeroParamsBtn");
    const saveParamsBtn = document.getElementById("saveParamsBtn");
    const runningStatus = document.getElementById("runningStatus");
    const containerName = document.getElementById("containerName");
    const composeFile = document.getElementById("composeFile");
    const ros2Distro = document.getElementById("ros2Distro");
    const ros2Info = document.getElementById("ros2Info");
    const copyLogs = document.getElementById("copyLogs");
    const topicLogs = document.getElementById("topicLogs");
    const logs = document.getElementById("logs");
    const alphaBaisBias = document.getElementById("alphaBaisBias");
    const rangeFixA0 = document.getElementById("rangeFixA0");
    const rangeFixA1 = document.getElementById("rangeFixA1");
    const message = document.getElementById("message");
    let actionInFlight = false;
    const defaultCalibrationParams = {
      alpha_bais_bias: "-0.014",
      range_fix_a0: "-0.0095",
      range_fix_a1: "-0.007",
    };

    async function fetchJson(url, options) {
      const response = await fetch(url, options);
      const data = await response.json();
      if (!response.ok) {
        throw new Error(data.error || "Request failed");
      }
      return data;
    }

    function setMessage(text, isError = false) {
      message.textContent = text;
      message.className = "message " + (isError ? "bad" : "");
    }

    function setActionState(busy) {
      actionInFlight = busy;
      startBtn.disabled = busy;
      stopBtn.disabled = busy;
      copyBtn.disabled = busy;
      detectRosBtn.disabled = busy;
      topicsBtn.disabled = busy;
      refreshBtn.disabled = busy;
      saveParamsBtn.disabled = busy;
    }

    async function refreshStatus() {
      try {
        const data = await fetchJson("/api/status");
        const running = Boolean(data.running);
        runningStatus.textContent = running ? "Running" : "Stopped";
        runningStatus.className = "value " + (running ? "ok" : "bad");
        containerName.textContent = data.container_name || "-";
        composeFile.textContent = data.compose_file || "-";
        ros2Distro.textContent = data.ros2_distro || data.ros2_error || "Unknown";
      } catch (error) {
        setMessage(error.message, true);
      }
    }

    function setParameterInputs(params) {
      alphaBaisBias.value = params.alpha_bais_bias ?? "";
      rangeFixA0.value = params.range_fix_a0 ?? "";
      rangeFixA1.value = params.range_fix_a1 ?? "";
    }

    function getParameterInputs() {
      return {
        alpha_bais_bias: alphaBaisBias.value,
        range_fix_a0: rangeFixA0.value,
        range_fix_a1: rangeFixA1.value,
      };
    }

    function setPresetAndSave(params) {
      setParameterInputs(params);
      return saveParameters(getParameterInputs());
    }

    async function refreshRos2Info() {
      try {
        const data = await fetchJson("/api/ros2");
        const lines = [
          "ROS 2 distro: " + (data.ros_distro || "Unknown"),
          "Setup path: " + (data.setup_path || "Unknown"),
        ];
        ros2Info.textContent = lines.join("\n");
        ros2Distro.textContent = data.ros_distro || "Unknown";
      } catch (error) {
        ros2Info.textContent = error.message;
        ros2Distro.textContent = "Unknown";
        setMessage(error.message, true);
      }
    }

    async function listTopics() {
      if (actionInFlight) return;
      setActionState(true);
      setMessage("Listing ROS 2 topics...");
      try {
        const data = await fetchJson("/api/topics", { method: "POST" });
        const output = [data.stdout, data.stderr].filter(Boolean).join("\n\n") || "No topics found.";
        topicLogs.textContent = output;
        topicLogs.scrollTop = topicLogs.scrollHeight;
        setMessage(data.stdout || "Topic list loaded.");
      } catch (error) {
        topicLogs.textContent = error.message;
        setMessage(error.message, true);
      } finally {
        setActionState(false);
        await refreshStatus();
        await refreshLogs();
      }
    }

    async function refreshParameters() {
      try {
        const data = await fetchJson("/api/params");
        setParameterInputs(data.params || {});
      } catch (error) {
        setMessage(error.message, true);
      }
    }

    async function refreshLogs() {
      try {
        const data = await fetchJson("/api/logs?tail=300");
        logs.textContent = data.logs || "No logs yet.";
        logs.scrollTop = logs.scrollHeight;
      } catch (error) {
        logs.textContent = error.message;
      }
    }

    async function runAction(path, outputTarget = null) {
      if (actionInFlight) return;
      setActionState(true);
      setMessage("Running " + path.replace("/api/", "") + "...");
      try {
        const data = await fetchJson(path, { method: "POST" });
        setMessage(data.stdout || "Command finished.");
        if (outputTarget) {
          outputTarget.textContent = [data.stdout, data.stderr].filter(Boolean).join("\\n\\n") || "No output.";
          outputTarget.scrollTop = outputTarget.scrollHeight;
        }
      } catch (error) {
        setMessage(error.message, true);
        if (outputTarget) {
          outputTarget.textContent = error.message;
        }
      } finally {
        setActionState(false);
        await refreshStatus();
        await refreshLogs();
      }
    }

    async function saveParameters(overrides = null) {
      if (actionInFlight) return;
      setActionState(true);
      setMessage("Saving parameters...");
      try {
        const payload = overrides || getParameterInputs();
        const data = await fetchJson("/api/params", {
          method: "POST",
          headers: {
            "Content-Type": "application/json",
          },
          body: JSON.stringify(payload),
        });
        setMessage(data.stdout || "Parameters saved.");
        setParameterInputs(data.params || payload);
      } catch (error) {
        setMessage(error.message, true);
      } finally {
        setActionState(false);
        await refreshStatus();
        await refreshLogs();
      }
    }

    startBtn.addEventListener("click", () => runAction("/api/start"));
    stopBtn.addEventListener("click", () => runAction("/api/stop"));
    copyBtn.addEventListener("click", () => runAction("/api/copy", copyLogs));
    detectRosBtn.addEventListener("click", refreshRos2Info);
    topicsBtn.addEventListener("click", listTopics);
    defaultParamsBtn.addEventListener("click", () => setPresetAndSave(defaultCalibrationParams));
    zeroParamsBtn.addEventListener("click", () => setPresetAndSave({
      alpha_bais_bias: "0",
      range_fix_a0: "0",
      range_fix_a1: "0",
    }));
    saveParamsBtn.addEventListener("click", saveParameters);
    refreshBtn.addEventListener("click", async () => {
      await refreshStatus();
      await refreshParameters();
      await refreshLogs();
    });

    refreshStatus();
    refreshRos2Info();
    refreshParameters();
    refreshLogs();
    setInterval(refreshStatus, 3000);
    setInterval(refreshLogs, 2000);
  </script>
</body>
</html>
"""


def run_command(command):
    process = subprocess.run(
        command,
        capture_output=True,
        text=True,
        cwd=str(REPO_ROOT),
    )
    return {
        "returncode": process.returncode,
        "stdout": process.stdout.strip(),
        "stderr": process.stderr.strip(),
    }


def compose_file_path(compose_name):
    return str(REPO_ROOT / "docker_compose" / "unilidar_mapping" / f"{compose_name}.compose.yml")


def _candidate_ros2_setup_paths():
    seen = set()

    def add(path):
        path = str(path)
        if path not in seen:
            seen.add(path)
            yield path

    if ROS2_SETUP_OVERRIDE:
        yield from add(ROS2_SETUP_OVERRIDE)

    if ROS2_DISTRO_OVERRIDE:
        yield from add(Path("/opt/ros") / ROS2_DISTRO_OVERRIDE / "setup.bash")

    for distro in ROS2_DISTRO_CANDIDATES:
        yield from add(Path("/opt/ros") / distro / "setup.bash")

    ros_root = Path("/opt/ros")
    if ros_root.is_dir():
        for child in sorted(ros_root.iterdir()):
            if child.is_dir():
                yield from add(child / "setup.bash")


def find_ros2_setup():
    for candidate in _candidate_ros2_setup_paths():
        candidate_path = Path(candidate)
        if not candidate_path.is_file():
            continue
        probe = run_command(
            [
                "bash",
                "-lc",
                f"source {shlex.quote(str(candidate_path))} >/dev/null 2>&1 && command -v ros2 >/dev/null 2>&1 && printf '%s' \"$ROS_DISTRO\"",
            ]
        )
        if probe["returncode"] == 0 and probe["stdout"]:
            return {
                "setup_path": str(candidate_path),
                "ros_distro": probe["stdout"],
            }
    raise FileNotFoundError(
        "Unable to find a usable ROS 2 setup.bash. "
        "Set UNILIDAR_ROS2_SETUP or UNILIDAR_ROS2_DISTRO if needed."
    )


def read_compose_parameters():
    compose_path = Path(compose_file_path(DEFAULT_COMPOSE_NAME))
    if not compose_path.is_file():
        raise FileNotFoundError(f"compose file not found: {compose_path}")

    content = compose_path.read_text(encoding="utf-8")
    match = PARAM_LINE_RE.search(content)
    if not match:
        raise ValueError("Could not find calibration parameters in compose file.")

    params = {name: match.group(name) for name in COMPOSE_PARAM_NAMES}
    return params


def write_compose_parameters(values):
    compose_path = Path(compose_file_path(DEFAULT_COMPOSE_NAME))
    if not compose_path.is_file():
        raise FileNotFoundError(f"compose file not found: {compose_path}")

    content = compose_path.read_text(encoding="utf-8")
    match = PARAM_LINE_RE.search(content)
    if not match:
        raise ValueError("Could not find calibration parameters in compose file.")

    def replace_value(name):
        raw_value = values[name]
        try:
            return format(float(raw_value), ".15g")
        except (TypeError, ValueError):
            raise ValueError(f"Invalid numeric value for {name}: {raw_value!r}")

    replacement = (
        f"--alpha_bais_bias={replace_value('alpha_bais_bias')}"
        f"{match.group('mid1')}{replace_value('range_fix_a0')}"
        f" --range_fix_a1={replace_value('range_fix_a1')}"
    )
    updated = content[: match.start()] + replacement + content[match.end() :]
    compose_path.write_text(updated, encoding="utf-8")
    return {name: replace_value(name) for name in COMPOSE_PARAM_NAMES}


def get_status():
    try:
        ros2_info = find_ros2_setup()
    except Exception as error:
        ros2_info = {"setup_path": "", "ros_distro": "", "error": str(error)}
    inspect = run_command(
        [
            "docker",
            "inspect",
            "-f",
            "{{.State.Running}}",
            DEFAULT_CONTAINER_NAME,
        ]
    )
    running = inspect["returncode"] == 0 and inspect["stdout"].lower() == "true"
    return {
        "running": running,
        "container_name": DEFAULT_CONTAINER_NAME,
        "compose_file": compose_file_path(DEFAULT_COMPOSE_NAME),
        "ros2_distro": ros2_info.get("ros_distro", ""),
        "ros2_setup_path": ros2_info.get("setup_path", ""),
        "ros2_error": ros2_info.get("error", ""),
        "docker_available": run_command(["docker", "version"])["returncode"] == 0,
        "inspect_error": inspect["stderr"] if inspect["returncode"] != 0 else "",
    }


def get_logs(tail):
    return run_command(
        [
            "docker",
            "logs",
            f"--tail={tail}",
            DEFAULT_CONTAINER_NAME,
        ]
    )


def combine_output(result):
    return "\n".join(part for part in [result["stdout"], result["stderr"]] if part)


def format_command_error(result, fallback):
    message = result["stderr"] or result["stdout"] or fallback
    return {
        "error": message,
        "returncode": result["returncode"],
        "stdout": result["stdout"],
        "stderr": result["stderr"],
    }


class UniLidarHandler(BaseHTTPRequestHandler):
    server_version = "UniLidarRemote/1.0"

    def log_message(self, fmt, *args):
        sys.stderr.write("%s - - [%s] %s\n" % (self.address_string(), self.log_date_time_string(), fmt % args))

    def _write_json(self, payload, status=HTTPStatus.OK):
        body = json.dumps(payload).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _write_html(self, body, status=HTTPStatus.OK):
        data = body.encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        parsed = urlparse(self.path)
        if parsed.path == "/":
            self._write_html(INDEX_HTML)
            return
        if parsed.path == "/api/status":
            self._write_json(get_status())
            return
        if parsed.path == "/api/ros2":
            try:
                self._write_json(find_ros2_setup())
            except Exception as error:
                self._write_json({"error": str(error)}, HTTPStatus.BAD_GATEWAY)
            return
        if parsed.path == "/api/logs":
            query = parse_qs(parsed.query)
            raw_tail = query.get("tail", ["300"])[0]
            try:
                tail = max(1, min(1000, int(raw_tail)))
            except ValueError:
                tail = 300
            result = get_logs(tail)
            status = get_status()
            if result["returncode"] != 0:
                missing_container = "No such container" in result["stderr"]
                message = (
                    f"{DEFAULT_CONTAINER_NAME} is not running yet."
                    if missing_container
                    else combine_output(result) or "Unable to read docker logs."
                )
                payload = {
                    "logs": message,
                    "running": status["running"],
                    "container_name": DEFAULT_CONTAINER_NAME,
                }
                self._write_json(payload)
                return
            payload = {
                "logs": combine_output(result),
                "running": status["running"],
                "container_name": DEFAULT_CONTAINER_NAME,
            }
            self._write_json(payload)
            return
        if parsed.path == "/api/params":
            try:
                params = read_compose_parameters()
                self._write_json({"params": params})
            except Exception as error:
                self._write_json({"error": str(error)}, HTTPStatus.BAD_GATEWAY)
            return
        self._write_json({"error": "Not found"}, HTTPStatus.NOT_FOUND)

    def do_POST(self):
        parsed = urlparse(self.path)
        if parsed.path == "/api/start":
            result = run_command([str(START_SCRIPT), DEFAULT_COMPOSE_NAME])
            if result["returncode"] == 0:
                self._write_json(result)
            else:
                self._write_json(
                    format_command_error(result, "Failed to start UniLidar."),
                    HTTPStatus.BAD_GATEWAY,
                )
            return
        if parsed.path == "/api/stop":
            result = run_command([str(STOP_SCRIPT), DEFAULT_COMPOSE_NAME])
            if result["returncode"] == 0:
                self._write_json(result)
            else:
                self._write_json(
                    format_command_error(result, "Failed to stop UniLidar."),
                    HTTPStatus.BAD_GATEWAY,
                )
            return
        if parsed.path == "/api/copy":
            result = run_command([str(COPY_SCRIPT)])
            if result["returncode"] == 0:
                self._write_json(result)
            else:
                self._write_json(
                    format_command_error(result, "Failed to copy data to drive."),
                    HTTPStatus.BAD_GATEWAY,
                )
            return
        if parsed.path == "/api/topics":
            try:
                ros2_info = find_ros2_setup()
            except Exception as error:
                self._write_json({"error": str(error)}, HTTPStatus.BAD_GATEWAY)
                return
            result = run_command(
                [
                    "bash",
                    "-lc",
                    f"source {shlex.quote(ros2_info['setup_path'])} >/dev/null 2>&1 && ros2 topic list",
                ]
            )
            if result["returncode"] == 0:
                self._write_json({**result, **ros2_info})
            else:
                self._write_json(
                    format_command_error(result, "Failed to list ROS 2 topics."),
                    HTTPStatus.BAD_GATEWAY,
                )
            return
        if parsed.path == "/api/params":
            try:
                raw_length = int(self.headers.get("Content-Length", "0"))
            except ValueError:
                raw_length = 0
            raw_body = self.rfile.read(max(0, raw_length)) if raw_length else b""
            try:
                payload = json.loads(raw_body.decode("utf-8") or "{}")
            except json.JSONDecodeError as error:
                self._write_json({"error": f"Invalid JSON payload: {error}"}, HTTPStatus.BAD_REQUEST)
                return
            try:
                params = write_compose_parameters(payload)
                self._write_json({"stdout": "Parameters saved.", "params": params})
            except Exception as error:
                self._write_json({"error": str(error)}, HTTPStatus.BAD_GATEWAY)
            return
        self._write_json({"error": "Not found"}, HTTPStatus.NOT_FOUND)


def main():
    if not START_SCRIPT.is_file():
        raise FileNotFoundError(f"start script not found: {START_SCRIPT}")
    if not STOP_SCRIPT.is_file():
        raise FileNotFoundError(f"stop script not found: {STOP_SCRIPT}")
    if not COPY_SCRIPT.is_file():
        raise FileNotFoundError(f"copy script not found: {COPY_SCRIPT}")

    server = ThreadingHTTPServer((DEFAULT_HOST, DEFAULT_PORT), UniLidarHandler)
    print(
        "Serving UniLidar web control on "
        f"http://{DEFAULT_HOST}:{DEFAULT_PORT} "
        f"(container={shlex.quote(DEFAULT_CONTAINER_NAME)}, compose={shlex.quote(DEFAULT_COMPOSE_NAME)})"
    )
    server.serve_forever()


if __name__ == "__main__":
    main()
