#!/usr/bin/env python3
import json
import os
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
  </style>
</head>
<body>
  <div class="layout">
    <div class="card">
      <h1>UniLidar Remote Control</h1>
      <p>Start or stop the compose stack and watch the live debug output from <code>UniLidarSdk</code>.</p>

      <div class="toolbar">
        <button class="start" id="startBtn">Start UniLidar</button>
        <button class="stop" id="stopBtn">Stop UniLidar</button>
        <button class="copy" id="copyBtn">Copy to Drive</button>
        <button class="ghost" id="refreshBtn">Refresh Logs</button>
      </div>

      <div class="message" id="message"></div>

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

      <div class="status-box" style="margin-bottom: 20px;">
        <span class="label">Copy Result Log</span>
        <pre class="logs" id="copyLogs" style="min-height: 180px; max-height: 260px; margin-top: 0;">No copy has run yet.</pre>
      </div>

      <pre class="logs" id="logs">Loading logs...</pre>
    </div>
  </div>

  <script>
    const startBtn = document.getElementById("startBtn");
    const stopBtn = document.getElementById("stopBtn");
    const copyBtn = document.getElementById("copyBtn");
    const refreshBtn = document.getElementById("refreshBtn");
    const runningStatus = document.getElementById("runningStatus");
    const containerName = document.getElementById("containerName");
    const composeFile = document.getElementById("composeFile");
    const copyLogs = document.getElementById("copyLogs");
    const logs = document.getElementById("logs");
    const message = document.getElementById("message");
    let actionInFlight = false;

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
      refreshBtn.disabled = busy;
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

    startBtn.addEventListener("click", () => runAction("/api/start"));
    stopBtn.addEventListener("click", () => runAction("/api/stop"));
    copyBtn.addEventListener("click", () => runAction("/api/copy", copyLogs));
    refreshBtn.addEventListener("click", async () => {
      await refreshStatus();
      await refreshLogs();
    });

    refreshStatus();
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
