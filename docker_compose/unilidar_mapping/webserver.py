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
        REPO_ROOT / "tools" / "copy_to_drive.sh",
    )
)
CHECK_CPU_FREQ_SCRIPT = Path(
    os.environ.get(
        "UNILIDAR_CHECK_CPU_FREQ_SCRIPT",
        REPO_ROOT / "tools" / "check_current_cpu_freq.sh",
    )
)
SET_CPU_FREQ_MAX_SCRIPT = Path(
    os.environ.get(
        "UNILIDAR_SET_CPU_FREQ_MAX_SCRIPT",
        REPO_ROOT / "tools" / "set_cpu_freq_max.sh",
    )
)
COMPOSE_PARAM_NAMES = ("alpha_bais_bias", "range_fix_a0", "range_fix_a1")
RECORDER_BAG_SUFFIX_RE = re.compile(r'(?m)^(?P<prefix>\s*BAG_NAME_SUFFIX=")(?P<suffix>[^"]*)(?P<suffix_end>")\s*$')

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
    .toggle-active {
      background: #238636;
      color: white;
      border: 1px solid #2ea043;
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
      <p>Start or stop the compose stack and watch the live debug output from the collection containers.</p>

      <details class="status-box" style="margin-bottom: 20px;">
        <summary class="panel-title" style="cursor: pointer; list-style: none;">Calibration Parameters</summary>
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
          <button class="ghost" id="defaultParamsBtn" type="button">Load Defaults</button>
          <button class="ghost" id="zeroParamsBtn" type="button">All Zeros</button>
          <button class="copy" id="saveParamsBtn" type="button">Save Parameters</button>
        </div>
      </details>

      <details class="status-box" style="margin-bottom: 20px;">
        <summary class="panel-title" style="cursor: pointer; list-style: none;">Recorder Bag Name</summary>
        <p class="panel-note">Add an optional postfix to recorder bag names, for example <code>_postfix</code>.</p>
        <div class="field">
          <label for="bagNameSuffix">bag postfix</label>
          <input id="bagNameSuffix" type="text" spellcheck="false" placeholder="_postfix">
        </div>
        <div class="toolbar" style="margin-bottom: 0;">
          <button class="copy" id="saveBagSuffixBtn" type="button">Save Bag Postfix</button>
        </div>
      </details>

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
      </div>

      <div class="toolbar">
        <button class="start" id="startBtn">Start UniLidar</button>
        <button class="stop" id="stopBtn">Stop UniLidar</button>
        <button class="ghost" id="refreshBtn">Refresh Logs</button>
      </div>

      <div class="message" id="message"></div>

      <div class="status-box" style="margin-bottom: 20px;">
        <span class="label">Log Source</span>
        <div class="toolbar" style="margin: 12px 0 0;">
          <button class="ghost toggle-active" id="uniLogBtn">UniLidarSdk</button>
          <button class="ghost" id="recorderLogBtn">Recorder</button>
          <button class="ghost" id="rtkLogBtn">RtkPublisher</button>
        </div>
      </div>

      <pre class="logs" id="logs">Loading logs...</pre>

      <details class="status-box" style="margin-top: 20px;" open>
        <summary class="label" style="cursor: pointer; list-style: none;">Tools</summary>
        <div class="toolbar" style="margin: 12px 0 16px;">
          <button class="copy" id="copyBtn">Copy to Drive</button>
          <button class="ghost" id="topicsBtn">List Topics</button>
          <button class="ghost" id="checkCpuFreqBtn">Check CPU Freq</button>
          <button class="ghost" id="setCpuFreqMaxBtn">Set CPU Max</button>
        </div>
        <pre class="logs" id="toolLogs" style="min-height: 120px; max-height: 260px;">No tool has run yet.</pre>
      </details>
    </div>
  </div>

  <script>
    const startBtn = document.getElementById("startBtn");
    const stopBtn = document.getElementById("stopBtn");
    const copyBtn = document.getElementById("copyBtn");
    const topicsBtn = document.getElementById("topicsBtn");
    const refreshBtn = document.getElementById("refreshBtn");
    const uniLogBtn = document.getElementById("uniLogBtn");
    const recorderLogBtn = document.getElementById("recorderLogBtn");
    const rtkLogBtn = document.getElementById("rtkLogBtn");
    const defaultParamsBtn = document.getElementById("defaultParamsBtn");
    const zeroParamsBtn = document.getElementById("zeroParamsBtn");
    const saveParamsBtn = document.getElementById("saveParamsBtn");
    const bagNameSuffix = document.getElementById("bagNameSuffix");
    const saveBagSuffixBtn = document.getElementById("saveBagSuffixBtn");
    const runningStatus = document.getElementById("runningStatus");
    const containerName = document.getElementById("containerName");
    const composeFile = document.getElementById("composeFile");
    const checkCpuFreqBtn = document.getElementById("checkCpuFreqBtn");
    const setCpuFreqMaxBtn = document.getElementById("setCpuFreqMaxBtn");
    const toolLogs = document.getElementById("toolLogs");
    const logs = document.getElementById("logs");
    const alphaBaisBias = document.getElementById("alphaBaisBias");
    const rangeFixA0 = document.getElementById("rangeFixA0");
    const rangeFixA1 = document.getElementById("rangeFixA1");
    const message = document.getElementById("message");
    let actionInFlight = false;
    let logContainer = "UniLidarSdk";
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
      topicsBtn.disabled = busy;
      refreshBtn.disabled = busy;
      uniLogBtn.disabled = busy;
      recorderLogBtn.disabled = busy;
      rtkLogBtn.disabled = busy;
      saveParamsBtn.disabled = busy;
      saveBagSuffixBtn.disabled = busy;
      checkCpuFreqBtn.disabled = busy;
      setCpuFreqMaxBtn.disabled = busy;
    }

    function setLogContainer(name) {
      logContainer = name;
      uniLogBtn.className = "ghost " + (name === "UniLidarSdk" ? "toggle-active" : "");
      recorderLogBtn.className = "ghost " + (name === "Recorder" ? "toggle-active" : "");
      rtkLogBtn.className = "ghost " + (name === "RtkPublisher" ? "toggle-active" : "");
    }

    async function refreshStatus() {
      try {
        const data = await fetchJson("/api/status");
        const running = Boolean(data.running);
        runningStatus.textContent = running ? "Running" : "Stopped";
        runningStatus.className = "value " + (running ? "ok" : "bad");
        containerName.textContent = data.container_name || "-";
        composeFile.textContent = data.compose_file || "-";
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

    function setBagSuffix(value) {
      bagNameSuffix.value = value ?? "";
    }

    function setPresetAndSave(params) {
      setParameterInputs(params);
      return saveParameters(getParameterInputs());
    }

    async function listTopics() {
      if (actionInFlight) return;
      setActionState(true);
      setMessage("Listing ROS 2 topics...");
      try {
        const data = await fetchJson("/api/topics", { method: "POST" });
        const output = [data.stdout, data.stderr].filter(Boolean).join("\\n\\n") || "No topics found.";
        toolLogs.textContent = output;
        toolLogs.scrollTop = toolLogs.scrollHeight;
        setMessage(data.stdout || "Topic list loaded.");
      } catch (error) {
        toolLogs.textContent = error.message;
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

    async function refreshBagSuffix() {
      try {
        const data = await fetchJson("/api/bag_suffix");
        setBagSuffix(data.bag_name_suffix || "");
      } catch (error) {
        setMessage(error.message, true);
      }
    }

    async function refreshLogs() {
      try {
        const data = await fetchJson("/api/logs?tail=50&container=" + encodeURIComponent(logContainer));
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
        if (outputTarget) {
          outputTarget.textContent = [data.stdout, data.stderr].filter(Boolean).join("\\n\\n") || "No output.";
          outputTarget.scrollTop = outputTarget.scrollHeight;
        } else {
          setMessage(data.stdout || "Command finished.");
        }
      } catch (error) {
        if (outputTarget) {
          outputTarget.textContent = error.message;
        } else {
          setMessage(error.message, true);
        }
      } finally {
        setActionState(false);
        await refreshStatus();
        await refreshLogs();
      }
    }

    async function saveParameters(overrides = null) {
      if (actionInFlight) return;
      if (overrides && typeof overrides.preventDefault === "function") {
        overrides.preventDefault();
        overrides = null;
      }
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

    async function saveBagSuffix() {
      if (actionInFlight) return;
      setActionState(true);
      setMessage("Saving bag postfix...");
      try {
        const payload = {
          bag_name_suffix: bagNameSuffix.value,
        };
        const data = await fetchJson("/api/bag_suffix", {
          method: "POST",
          headers: {
            "Content-Type": "application/json",
          },
          body: JSON.stringify(payload),
        });
        setMessage(data.stdout || "Bag postfix saved.");
        setBagSuffix(data.bag_name_suffix || payload.bag_name_suffix);
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
    copyBtn.addEventListener("click", () => runAction("/api/copy", toolLogs));
    topicsBtn.addEventListener("click", listTopics);
    checkCpuFreqBtn.addEventListener("click", () => runAction("/api/cpu_freq", toolLogs));
    setCpuFreqMaxBtn.addEventListener("click", () => runAction("/api/cpu_freq_max", toolLogs));
    uniLogBtn.addEventListener("click", async () => {
      setLogContainer("UniLidarSdk");
      await refreshLogs();
    });
    recorderLogBtn.addEventListener("click", async () => {
      setLogContainer("Recorder");
      await refreshLogs();
    });
    rtkLogBtn.addEventListener("click", async () => {
      setLogContainer("RtkPublisher");
      await refreshLogs();
    });
    defaultParamsBtn.addEventListener("click", () => setPresetAndSave(defaultCalibrationParams));
    zeroParamsBtn.addEventListener("click", () => setPresetAndSave({
      alpha_bais_bias: "0",
      range_fix_a0: "0",
      range_fix_a1: "0",
    }));
    saveParamsBtn.addEventListener("click", () => saveParameters());
    saveBagSuffixBtn.addEventListener("click", saveBagSuffix);
    refreshBtn.addEventListener("click", async () => {
      await refreshStatus();
      await refreshParameters();
      await refreshBagSuffix();
      await refreshLogs();
    });

    refreshStatus();
    setLogContainer("UniLidarSdk");
    refreshParameters();
    refreshBagSuffix();
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


def read_bag_name_suffix():
    compose_path = Path(compose_file_path(DEFAULT_COMPOSE_NAME))
    if not compose_path.is_file():
        raise FileNotFoundError(f"compose file not found: {compose_path}")

    content = compose_path.read_text(encoding="utf-8")
    match = RECORDER_BAG_SUFFIX_RE.search(content)
    if not match:
        raise ValueError("Could not find bag postfix in compose file.")
    return match.group("suffix")


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


def write_bag_name_suffix(value):
    compose_path = Path(compose_file_path(DEFAULT_COMPOSE_NAME))
    if not compose_path.is_file():
        raise FileNotFoundError(f"compose file not found: {compose_path}")

    content = compose_path.read_text(encoding="utf-8")
    match = RECORDER_BAG_SUFFIX_RE.search(content)
    if not match:
        raise ValueError("Could not find bag postfix in compose file.")

    suffix = "" if value is None else str(value)
    if "\n" in suffix or "\r" in suffix:
        raise ValueError("bag postfix must be a single line.")
    escaped_suffix = (
        suffix.replace("\\", "\\\\")
        .replace('"', '\\"')
        .replace("$", "\\$")
        .replace("`", "\\`")
    )

    replacement = f'{match.group("prefix")}{escaped_suffix}{match.group("suffix_end")}'
    updated = content[: match.start()] + replacement + content[match.end() :]
    compose_path.write_text(updated, encoding="utf-8")
    return suffix


def get_status():
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
        "docker_available": run_command(["docker", "version"])["returncode"] == 0,
        "inspect_error": inspect["stderr"] if inspect["returncode"] != 0 else "",
    }


def get_logs(tail, container_name=DEFAULT_CONTAINER_NAME):
    return run_command(
        [
            "docker",
            "logs",
            f"--tail={tail}",
            container_name,
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
        if parsed.path == "/api/bag_suffix":
            try:
                self._write_json({"bag_name_suffix": read_bag_name_suffix()})
            except Exception as error:
                self._write_json({"error": str(error)}, HTTPStatus.BAD_GATEWAY)
            return
        if parsed.path == "/api/logs":
            query = parse_qs(parsed.query)
            raw_tail = query.get("tail", ["300"])[0]
            container_name = query.get("container", [DEFAULT_CONTAINER_NAME])[0] or DEFAULT_CONTAINER_NAME
            try:
                tail = max(1, min(1000, int(raw_tail)))
            except ValueError:
                tail = 300
            result = get_logs(tail, container_name)
            status = get_status()
            if result["returncode"] != 0:
                missing_container = "No such container" in result["stderr"]
                message = (
                    f"{container_name} is not running yet."
                    if missing_container
                    else combine_output(result) or "Unable to read docker logs."
                )
                payload = {
                    "logs": message,
                    "running": status["running"],
                    "container_name": container_name,
                }
                self._write_json(payload)
                return
            payload = {
                "logs": combine_output(result),
                "running": status["running"],
                "container_name": container_name,
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
        if parsed.path == "/api/cpu_freq":
            result = run_command([str(CHECK_CPU_FREQ_SCRIPT)])
            if result["returncode"] == 0:
                self._write_json(result)
            else:
                self._write_json(
                    format_command_error(result, "Failed to read CPU frequency."),
                    HTTPStatus.BAD_GATEWAY,
                )
            return
        if parsed.path == "/api/cpu_freq_max":
            result = run_command([str(SET_CPU_FREQ_MAX_SCRIPT)])
            if result["returncode"] == 0:
                self._write_json(result)
            else:
                self._write_json(
                    format_command_error(result, "Failed to set CPU frequency to max."),
                    HTTPStatus.BAD_GATEWAY,
                )
            return
        if parsed.path == "/api/topics":
            result = run_command(
                [
                    "docker",
                    "exec",
                    DEFAULT_CONTAINER_NAME,
                    "bash",
                    "-lc",
                    "source /opt/ros/humble/setup.bash && ros2 topic list",
                ]
            )
            if result["returncode"] == 0:
                self._write_json(result)
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
        if parsed.path == "/api/bag_suffix":
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
                bag_name_suffix = write_bag_name_suffix(payload.get("bag_name_suffix", ""))
                self._write_json({"stdout": "Bag postfix saved.", "bag_name_suffix": bag_name_suffix})
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
    if not CHECK_CPU_FREQ_SCRIPT.is_file():
        raise FileNotFoundError(f"check cpu freq script not found: {CHECK_CPU_FREQ_SCRIPT}")
    if not SET_CPU_FREQ_MAX_SCRIPT.is_file():
        raise FileNotFoundError(f"set cpu freq max script not found: {SET_CPU_FREQ_MAX_SCRIPT}")

    server = ThreadingHTTPServer((DEFAULT_HOST, DEFAULT_PORT), UniLidarHandler)
    print(
        "Serving UniLidar web control on "
        f"http://{DEFAULT_HOST}:{DEFAULT_PORT} "
        f"(container={shlex.quote(DEFAULT_CONTAINER_NAME)}, compose={shlex.quote(DEFAULT_COMPOSE_NAME)})"
    )
    server.serve_forever()


if __name__ == "__main__":
    main()
