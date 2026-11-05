#!/usr/bin/env python3
"""
BalanceBot Local Dev Server
Serves the dashboard at http://localhost:8080 with a simulated WebSocket
so you can preview and test without the ESP32 being connected.

Usage:
  python server.py

Then open:  http://localhost:8080
Library installer page:  http://localhost:8080/install
"""

import asyncio
import http.server
import json
import math
import os
import random
import socketserver
import threading
import time
import webbrowser
from pathlib import Path

try:
    import websockets
    WS_OK = True
except ImportError:
    WS_OK = False
    print("[WARN] websockets not installed. Simulated WS disabled.")
    print("       Run:  pip install websockets")

PORT_HTTP = 8080
PORT_WS   = 81
HTML_FILE = Path(__file__).parent / "dashboard.html"

# ─── Simulated robot physics ───────────────────────────────
class SimRobot:
    def __init__(self):
        self.angle   = 3.0
        self.omega   = 0.0
        self.Kp      = 12.0
        self.Ki      = 0.0
        self.Kd      = 0.4
        self.setpoint = 0.0
        self.integral = 0.0
        self.armed    = False
        self.output   = 0.0
        self.motor_pwm= 0
        self.loop_hz  = 198
        self.start_t  = time.time()

    def step(self, dt=0.005):
        acc_noise = self.angle + (random.random() - 0.5) * 1.5
        # Kalman (simplified: use acc_noise weighted toward angle)
        self.angle += self.omega * dt

        if self.armed:
            err = self.angle - self.setpoint
            self.integral += err * dt
            self.integral  = max(-30, min(30, self.integral))
            self.output    = self.Kp*err + self.Ki*self.integral + self.Kd*self.omega
            # Apply "motor force" back to pendulum (inverse)
            self.omega += (-9.8 * math.sin(math.radians(self.angle)) - self.output * 0.03) * dt
        else:
            self.output = 0
            self.omega += -9.8 * math.sin(math.radians(self.angle)) * dt

        self.omega *= 0.98  # damping
        self.motor_pwm = int(max(-255, min(255, self.output)))
        return acc_noise

    def telemetry(self, acc_noise):
        t_ms = int((time.time() - self.start_t) * 1000)
        return {
            "t":        t_ms,
            "angle":    round(self.angle, 2),
            "accAngle": round(acc_noise, 2),
            "gyro":     round(self.omega, 2),
            "output":   round(self.output, 1),
            "setpoint": round(self.setpoint, 2),
            "error":    round(self.angle - self.setpoint, 2),
            "integral": round(self.integral, 3),
            "Kp":       self.Kp,
            "Ki":       self.Ki,
            "Kd":       self.Kd,
            "armed":    self.armed,
            "motorPWM": self.motor_pwm,
            "trimRate": 0.0,
            "loopHz":   self.loop_hz,
        }

    def handle_cmd(self, msg):
        try:
            d = json.loads(msg)
            cmd = d.get("cmd", "")
            if cmd == "set":
                if "Kp" in d: self.Kp = float(d["Kp"])
                if "Ki" in d: self.Ki = float(d["Ki"])
                if "Kd" in d: self.Kd = float(d["Kd"])
                print(f"[SIM] PID → Kp={self.Kp} Ki={self.Ki} Kd={self.Kd}")
            elif cmd == "arm":
                self.setpoint  = self.angle
                self.integral  = 0
                self.armed     = True
                print("[SIM] ARMED")
            elif cmd == "disarm":
                self.armed = False
                self.output = 0
                print("[SIM] DISARMED")
            elif cmd == "setpoint":
                self.setpoint = float(d.get("value", 0))
            elif cmd == "reset_integral":
                self.integral = 0
                print("[SIM] Integral reset")
            elif cmd == "calibrate":
                self.angle  = 0
                self.omega  = 0
                self.armed  = False
                print("[SIM] Calibrated")
        except Exception as e:
            print(f"[SIM] CMD error: {e}")

sim = SimRobot()

# ─── WebSocket server (simulated telemetry) ────────────────
async def ws_handler(websocket, path=None):
    print(f"[WS] Client connected: {websocket.remote_address}")
    send_task = None

    async def sender():
        while True:
            acc = sim.step()
            msg = json.dumps(sim.telemetry(acc))
            try:
                await websocket.send(msg)
            except:
                break
            await asyncio.sleep(0.05)  # 20 Hz

    send_task = asyncio.ensure_future(sender())
    try:
        async for msg in websocket:
            sim.handle_cmd(msg)
    except:
        pass
    finally:
        if send_task:
            send_task.cancel()
        print("[WS] Client disconnected")

async def run_ws():
    if not WS_OK:
        return
    print(f"[WS] Simulated robot WebSocket on ws://localhost:{PORT_WS}")
    async with websockets.serve(ws_handler, "localhost", PORT_WS):
        await asyncio.Future()

# ─── HTTP server ───────────────────────────────────────────
INSTALL_HTML = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>BalanceBot — Library Installer</title>
<style>
  :root{--bg:#0d1117;--card:#161b22;--border:#30363d;--accent:#58a6ff;--green:#3fb950;--red:#f85149;--yellow:#d29922;--text:#c9d1d9;--muted:#8b949e;}
  *{box-sizing:border-box;margin:0;padding:0;}
  body{background:var(--bg);color:var(--text);font-family:'Segoe UI',system-ui,sans-serif;padding:20px;max-width:860px;margin:0 auto;}
  h1{font-size:22px;margin-bottom:4px;display:flex;align-items:center;gap:10px;}
  .sub{color:var(--muted);margin-bottom:24px;font-size:13px;}
  h2{font-size:15px;color:var(--accent);margin:24px 0 12px;display:flex;align-items:center;gap:8px;}
  .lib-card{background:var(--card);border:1px solid var(--border);border-radius:8px;padding:16px;margin-bottom:12px;display:grid;grid-template-columns:1fr auto;gap:12px;align-items:center;}
  .lib-name{font-size:15px;font-weight:700;}
  .lib-author{font-size:12px;color:var(--muted);margin-top:2px;}
  .lib-desc{font-size:12px;color:var(--muted);margin-top:6px;}
  .lib-ver{font-size:11px;background:#21262d;padding:2px 8px;border-radius:4px;color:var(--yellow);border:1px solid var(--border);}
  .btn{display:inline-block;padding:10px 18px;background:var(--accent);color:#0d1117;border-radius:6px;text-decoration:none;font-weight:700;font-size:13px;transition:opacity .15s;}
  .btn:hover{opacity:.85;}
  .btn-zip{background:#21262d;color:var(--text);border:1px solid var(--border);}
  .steps{background:var(--card);border:1px solid var(--border);border-radius:8px;padding:16px;margin-top:12px;}
  .step{display:flex;gap:12px;margin-bottom:14px;align-items:flex-start;}
  .step-num{background:var(--accent);color:#0d1117;width:24px;height:24px;border-radius:50%;display:flex;align-items:center;justify-content:center;font-weight:700;font-size:12px;flex-shrink:0;}
  .step-text{font-size:13px;line-height:1.6;}
  code{background:#21262d;padding:1px 6px;border-radius:3px;font-family:Consolas,monospace;font-size:12px;color:var(--yellow);}
  .tag-required{background:#3a1a1a;color:var(--red);font-size:10px;padding:2px 6px;border-radius:4px;border:1px solid var(--red);font-weight:700;}
  .tag-optional{background:#1a2a3a;color:var(--accent);font-size:10px;padding:2px 6px;border-radius:4px;border:1px solid var(--accent);}
  .wifi-box{background:#0f1f0f;border:1px solid var(--green);border-radius:8px;padding:14px;margin-top:16px;}
  .wifi-box h3{color:var(--green);margin-bottom:8px;}
  .wifi-row{display:flex;justify-content:space-between;padding:4px 0;border-bottom:1px solid #21262d;font-size:13px;}
  .wifi-row:last-child{border:none;}
  .wifi-val{font-weight:700;color:var(--text);}
  .back{display:inline-block;margin-bottom:16px;color:var(--accent);text-decoration:none;font-size:13px;}
  .back:hover{text-decoration:underline;}
</style>
</head>
<body>
<a href="/" class="back">← Back to Dashboard</a>
<h1>🤖 BalanceBot — Setup Guide</h1>
<p class="sub">Everything you need to get your self-balancing robot working</p>

<h2>📦 Required Arduino Libraries</h2>
<p style="font-size:12px;color:var(--muted);margin-bottom:12px;">Install via <strong>Arduino IDE → Tools → Manage Libraries</strong> (Ctrl+Shift+I)</p>

<div class="lib-card">
  <div>
    <div style="display:flex;align-items:center;gap:8px;margin-bottom:4px;">
      <span class="lib-name">WebSockets</span>
      <span class="tag-required">REQUIRED</span>
      <span class="lib-ver">v2.3.x</span>
    </div>
    <div class="lib-author">by Markus Sattler</div>
    <div class="lib-desc">WebSocket server on ESP32 for real-time bidirectional communication with the browser dashboard. Used for live telemetry (20Hz) and receiving PID commands.</div>
  </div>
  <div style="display:flex;flex-direction:column;gap:8px;text-align:center;">
    <a class="btn" href="https://github.com/Links2004/arduinoWebSockets/archive/refs/heads/master.zip" target="_blank">⬇ Download ZIP</a>
    <div style="font-size:11px;color:var(--muted)">or search "WebSockets"<br>in Library Manager</div>
  </div>
</div>

<div class="lib-card">
  <div>
    <div style="display:flex;align-items:center;gap:8px;margin-bottom:4px;">
      <span class="lib-name">ArduinoJson</span>
      <span class="tag-required">REQUIRED</span>
      <span class="lib-ver">v6.x</span>
    </div>
    <div class="lib-author">by Benoit Blanchon</div>
    <div class="lib-desc">JSON serialization for telemetry packets and command parsing. <strong style="color:var(--red)">Use v6 — NOT v7</strong>. The API changed in v7 and will break compilation.</div>
  </div>
  <div style="display:flex;flex-direction:column;gap:8px;text-align:center;">
    <a class="btn" href="https://github.com/bblanchon/ArduinoJson/releases/download/v6.21.5/ArduinoJson-v6.21.5.zip" target="_blank">⬇ Download v6.21.5</a>
    <div style="font-size:11px;color:var(--muted)">search "ArduinoJson"<br>select version 6.x</div>
  </div>
</div>

<div class="lib-card">
  <div>
    <div style="display:flex;align-items:center;gap:8px;margin-bottom:4px;">
      <span class="lib-name">ESP32 Arduino Core</span>
      <span class="tag-required">REQUIRED</span>
      <span class="lib-ver">v2.x or v3.x</span>
    </div>
    <div class="lib-author">by Espressif Systems</div>
    <div class="lib-desc">Board support for ESP32. Includes WiFi, WebServer, Wire, ledc PWM. Install via <strong>Boards Manager</strong>, not Library Manager.</div>
  </div>
  <div style="display:flex;flex-direction:column;gap:8px;text-align:center;">
    <a class="btn btn-zip" href="https://dl.espressif.com/dl/package_esp32_index.json" target="_blank">📋 Board URL</a>
    <div style="font-size:11px;color:var(--muted)">Add to Additional<br>Boards Manager URLs</div>
  </div>
</div>

<h2>🛠️ Arduino IDE Setup Steps</h2>
<div class="steps">
  <div class="step">
    <div class="step-num">1</div>
    <div class="step-text">
      <strong>Add ESP32 board support URL</strong><br>
      File → Preferences → Additional Boards Manager URLs:<br>
      <code>https://dl.espressif.com/dl/package_esp32_index.json</code>
    </div>
  </div>
  <div class="step">
    <div class="step-num">2</div>
    <div class="step-text">
      <strong>Install ESP32 boards</strong><br>
      Tools → Board → Boards Manager → search "esp32" → Install <strong>esp32 by Espressif Systems</strong>
    </div>
  </div>
  <div class="step">
    <div class="step-num">3</div>
    <div class="step-text">
      <strong>Install WebSockets library</strong><br>
      Tools → Manage Libraries → search <code>WebSockets</code> → install by <strong>Markus Sattler</strong> v2.3.x
    </div>
  </div>
  <div class="step">
    <div class="step-num">4</div>
    <div class="step-text">
      <strong>Install ArduinoJson library</strong><br>
      Tools → Manage Libraries → search <code>ArduinoJson</code> → install <strong>version 6.21.5</strong> (click version dropdown!)
    </div>
  </div>
  <div class="step">
    <div class="step-num">5</div>
    <div class="step-text">
      <strong>Open the sketch</strong><br>
      File → Open → <code>balance_bot/balance_bot.ino</code>
    </div>
  </div>
  <div class="step">
    <div class="step-num">6</div>
    <div class="step-text">
      <strong>Select board & port</strong><br>
      Tools → Board → ESP32 Arduino → <strong>ESP32 Dev Module</strong><br>
      Tools → Port → select your COM port
    </div>
  </div>
  <div class="step">
    <div class="step-num">7</div>
    <div class="step-text">
      <strong>Upload</strong><br>
      Click Upload ▶ (or Ctrl+U). Watch Serial Monitor at <code>115200</code> baud for calibration progress.
    </div>
  </div>
</div>

<h2>📶 WiFi Connection</h2>
<div class="wifi-box">
  <h3>🔗 Connect to your robot</h3>
  <div class="wifi-row"><span>WiFi SSID</span><span class="wifi-val">BalanceBot</span></div>
  <div class="wifi-row"><span>Password</span><span class="wifi-val">12345678</span></div>
  <div class="wifi-row"><span>Dashboard URL</span><span class="wifi-val"><a href="http://192.168.4.1" style="color:#58a6ff">http://192.168.4.1</a></span></div>
  <div class="wifi-row"><span>WebSocket</span><span class="wifi-val">ws://192.168.4.1:81</span></div>
  <div class="wifi-row"><span>Telemetry rate</span><span class="wifi-val">20 Hz</span></div>
  <div class="wifi-row"><span>Control loop</span><span class="wifi-val">200 Hz</span></div>
</div>

<h2>⚡ Quick Test (this local server)</h2>
<div class="steps">
  <div class="step">
    <div class="step-num">1</div>
    <div class="step-text">The dashboard at <a href="/" style="color:var(--accent)">http://localhost:8080</a> runs in <strong>DEMO MODE</strong> — simulated physics so you can preview all charts and the robot SVG without hardware.</div>
  </div>
  <div class="step">
    <div class="step-num">2</div>
    <div class="step-text">When you flash the ESP32 and connect to its WiFi, open <code>http://192.168.4.1</code> instead — the page will auto-connect via WebSocket and show real data.</div>
  </div>
</div>
</body>
</html>
"""

class Handler(http.server.SimpleHTTPRequestHandler):
    def do_GET(self):
        if self.path == "/" or self.path == "/index.html":
            content = HTML_FILE.read_bytes()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", len(content))
            self.end_headers()
            self.wfile.write(content)
        elif self.path == "/install":
            content = INSTALL_HTML.encode()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", len(content))
            self.end_headers()
            self.wfile.write(content)
        else:
            self.send_error(404)

    def log_message(self, fmt, *args):
        pass  # silence HTTP logs

def run_http():
    with socketserver.TCPServer(("", PORT_HTTP), Handler) as httpd:
        httpd.allow_reuse_address = True
        print(f"[HTTP] Dashboard  → http://localhost:{PORT_HTTP}")
        print(f"[HTTP] Installer  → http://localhost:{PORT_HTTP}/install")
        httpd.serve_forever()

def main():
    print("=" * 55)
    print("  [ROBOT]  BalanceBot Local Dev Server")
    print("=" * 55)

    # Start HTTP server in background thread
    t = threading.Thread(target=run_http, daemon=True)
    t.start()

    # Open browser
    time.sleep(0.5)
    webbrowser.open(f"http://localhost:{PORT_HTTP}")

    # Start WS server (runs event loop)
    if WS_OK:
        asyncio.run(run_ws())
    else:
        print("[!] WebSocket simulation disabled (install: pip install websockets)")
        print("    Dashboard will show demo animation only.")
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            pass

if __name__ == "__main__":
    main()
