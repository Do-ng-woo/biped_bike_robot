"""Local hold-to-drive web controls for the hardware policy runner."""

from __future__ import annotations

import json
import threading
import time
from collections.abc import Callable
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer


_PAGE = b"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Biped Control</title>
<style>
  :root { color-scheme: dark; font-family: system-ui, sans-serif; }
  body { margin: 0; min-height: 100vh; display: grid; place-items: center;
    background: #111418; color: #f4f6f8; }
  main { width: min(420px, calc(100vw - 32px)); text-align: center; }
  h1 { font-size: 22px; margin: 0 0 8px; }
  #status { color: #9ca7b2; margin-bottom: 24px; }
  .controls { display: grid; grid-template-columns: repeat(3, 1fr); gap: 10px; }
  button { min-height: 84px; border: 1px solid #46515d; border-radius: 8px;
    background: #20262d; color: inherit; font-size: 18px; font-weight: 650;
    touch-action: none; user-select: none; }
  button:active, button.active { background: #1769aa; border-color: #4ca8ee; }
  #ccw { grid-column: 1; }
  #forward { grid-column: 2; }
  #cw { grid-column: 3; }
  #left { grid-column: 1; }
  #stop { grid-column: 2; background: #812c32; }
  #right { grid-column: 3; }
  p { color: #9ca7b2; font-size: 14px; line-height: 1.5; }
</style>
</head>
<body>
<main>
  <h1>Biped Control</h1>
  <div id="status">STOPPED</div>
  <div class="controls">
    <button id="ccw" data-command="ccw">&#8634;<br>CCW</button>
    <button id="forward" data-command="forward">&#8593;<br>Forward</button>
    <button id="cw" data-command="cw">&#8635;<br>CW</button>
    <button id="left" data-command="left">&#8592;<br>Crab left</button>
    <button id="stop" data-command="stop">Stop</button>
    <button id="right" data-command="right">&#8594;<br>Crab right</button>
  </div>
  <p>Hold: W/Up forward, A/Left and D/Right crab, Q/E turn. Release or Space to stop.</p>
</main>
<script>
const status = document.querySelector('#status');
let active = 'stop';
let heartbeat = null;

async function send(command) {
  active = command;
  status.textContent = command.toUpperCase();
  document.querySelectorAll('button').forEach(
    button => button.classList.toggle('active', button.dataset.command === command)
  );
  try { await fetch('/command/' + command, {method: 'POST', cache: 'no-store'}); }
  catch (_) { status.textContent = 'DISCONNECTED'; }
}

function start(command) {
  if (heartbeat !== null) clearInterval(heartbeat);
  send(command);
  if (command !== 'stop') heartbeat = setInterval(() => send(command), 100);
}

function stop() {
  if (heartbeat !== null) clearInterval(heartbeat);
  heartbeat = null;
  if (active !== 'stop') send('stop');
}

document.querySelectorAll('button').forEach(button => {
  const command = button.dataset.command;
  button.addEventListener('pointerdown', event => {
    event.preventDefault();
    button.setPointerCapture(event.pointerId);
    command === 'stop' ? stop() : start(command);
  });
  button.addEventListener('pointerup', stop);
  button.addEventListener('pointercancel', stop);
});

const keys = new Map([
  ['ArrowUp', 'forward'], ['w', 'forward'], ['W', 'forward'],
  ['ArrowLeft', 'left'], ['a', 'left'], ['A', 'left'],
  ['ArrowRight', 'right'], ['d', 'right'], ['D', 'right'],
  ['q', 'ccw'], ['Q', 'ccw'], ['e', 'cw'], ['E', 'cw'],
]);
window.addEventListener('keydown', event => {
  if (event.code === 'Space') { event.preventDefault(); stop(); return; }
  const command = keys.get(event.key);
  if (command && !event.repeat) { event.preventDefault(); start(command); }
});
window.addEventListener('keyup', event => {
  if (keys.has(event.key)) { event.preventDefault(); stop(); }
});
window.addEventListener('blur', stop);
document.addEventListener('visibilitychange', () => { if (document.hidden) stop(); });
send('stop');
</script>
</body>
</html>
"""


class PolicyControlServer:
  """Serve local controls and stop motion when command heartbeats expire."""

  def __init__(
    self,
    port: int,
    command_callback: Callable[[str], None],
    deadman_timeout: float = 0.35,
  ) -> None:
    self._command_callback = command_callback
    self._deadman_timeout = deadman_timeout
    self._last_motion_command = 0.0
    self._moving = False
    owner = self

    class Handler(BaseHTTPRequestHandler):
      def do_GET(self) -> None:  # noqa: N802
        if self.path != "/":
          self.send_error(404)
          return
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(_PAGE)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(_PAGE)

      def do_POST(self) -> None:  # noqa: N802
        command = self.path.removeprefix("/command/")
        if command not in {"stop", "forward", "left", "right", "ccw", "cw"}:
          self.send_error(400)
          return
        owner._receive(command)
        payload = json.dumps({"command": command}).encode()
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(payload)))
        self.end_headers()
        self.wfile.write(payload)

      def log_message(self, format: str, *args: object) -> None:
        del format, args

    self._server = ThreadingHTTPServer(("127.0.0.1", port), Handler)
    self._thread = threading.Thread(target=self._run, daemon=True)
    self._watchdog = threading.Thread(target=self._run_watchdog, daemon=True)
    self._stopping = threading.Event()
    self.port = self._server.server_port

  def _receive(self, command: str) -> None:
    if command == "stop":
      self._moving = False
    else:
      self._last_motion_command = time.monotonic()
      self._moving = True
    self._command_callback(command)

  def _run(self) -> None:
    self._server.serve_forever(poll_interval=0.1)

  def _run_watchdog(self) -> None:
    while not self._stopping.wait(0.05):
      if self._moving and time.monotonic() - self._last_motion_command > self._deadman_timeout:
        self._moving = False
        self._command_callback("stop")

  def start(self) -> None:
    self._thread.start()
    self._watchdog.start()

  def close(self) -> None:
    self._stopping.set()
    self._command_callback("stop")
    self._server.shutdown()
    self._server.server_close()
    self._thread.join(timeout=1.0)
    self._watchdog.join(timeout=1.0)
