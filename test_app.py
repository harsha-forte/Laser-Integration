"""
test_app.py — Hardware test interface
Tests: A6-RS servo motor (Method 17 homing), GPIO signal testing
Run: sudo python3 test_app.py
Open: http://<pi-ip>:5001
"""

from flask import Flask, jsonify, request, render_template_string
import threading
import time
import serial
import crcmod

try:
    import RPi.GPIO as GPIO
    GPIO_AVAILABLE = True
except (ImportError, RuntimeError):
    GPIO_AVAILABLE = False
    print("[WARN] RPi.GPIO not available — GPIO simulation mode")

app = Flask(__name__)

# ═════════════════════════════════════════════════════════════════════════════
# CONFIG
# ═════════════════════════════════════════════════════════════════════════════

SERIAL_PORT   = "/dev/ttyACM0"
BAUD_RATE     = 115200
DRIVE_ADDR    = 0x01
PULSES_PER_MM = 2500
MAX_TRAVEL_MM = 420.0

PIN_LASER_START  = 17
PIN_LASER_STOP   = 27
PIN_INPUT_4      = 4
PIN_INPUT_26     = 26

# ═════════════════════════════════════════════════════════════════════════════
# MODBUS
# ═════════════════════════════════════════════════════════════════════════════

_crc16 = crcmod.mkCrcFun(0x18005, rev=True, initCrc=0xFFFF, xorOut=0x0000)

def crc(data):
    v = _crc16(data)
    return bytes([v & 0xFF, (v >> 8) & 0xFF])

def w16(grp, off, val):
    v = val & 0xFFFF
    p = bytes([DRIVE_ADDR, 0x06, grp, off, (v >> 8) & 0xFF, v & 0xFF])
    return p + crc(p)

def w32(grp, off, val):
    v = val & 0xFFFFFFFF
    lo = v & 0xFFFF; hi = (v >> 16) & 0xFFFF
    p = bytes([DRIVE_ADDR, 0x10, grp, off, 0x00, 0x02, 0x04,
               (lo >> 8) & 0xFF, lo & 0xFF,
               (hi >> 8) & 0xFF, hi & 0xFF])
    return p + crc(p)

def r(grp, off, n=1):
    p = bytes([DRIVE_ADDR, 0x03, grp, off, 0x00, n])
    return p + crc(p)

# ═════════════════════════════════════════════════════════════════════════════
# SERIAL
# ═════════════════════════════════════════════════════════════════════════════

ser = None
ser_lock = threading.Lock()

def connect_serial():
    global ser
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE,
                            bytesize=8, parity='N', stopbits=1, timeout=0.5)
        log(f"Serial connected: {SERIAL_PORT}")
        return True
    except Exception as e:
        log(f"Serial failed: {e}")
        return False

def send(frame, read_n=8):
    if not ser or not ser.is_open:
        return None
    with ser_lock:
        try:
            ser.reset_input_buffer()
            ser.write(frame)
            time.sleep(0.02)
            return ser.read(read_n)
        except Exception as e:
            log(f"Serial error: {e}")
            return None

def read16(grp, off):
    resp = send(r(grp, off, 1), 7)
    if resp and len(resp) >= 7:
        v = (resp[3] << 8) | resp[4]
        return v - 65536 if v > 32767 else v
    return None

def read32(grp, off):
    resp = send(r(grp, off, 2), 9)
    if resp and len(resp) >= 9:
        lo = (resp[3] << 8) | resp[4]
        hi = (resp[5] << 8) | resp[6]
        v = (hi << 16) | lo
        return v - 0x100000000 if v > 0x7FFFFFFF else v
    return None

def write16(grp, off, val):
    resp = send(w16(grp, off, val))
    return resp is not None and len(resp) >= 6

def write32(grp, off, val):
    resp = send(w32(grp, off, val), 12)
    return resp is not None and len(resp) >= 6

# ═════════════════════════════════════════════════════════════════════════════
# GPIO
# ═════════════════════════════════════════════════════════════════════════════

def setup_gpio():
    if not GPIO_AVAILABLE:
        return
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(PIN_LASER_START, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(PIN_LASER_STOP,  GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(PIN_INPUT_4,  GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    GPIO.setup(PIN_INPUT_26, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    log("GPIO initialised")

def read_gpio_state():
    if not GPIO_AVAILABLE:
        return {"gpio4": False, "gpio26": False,
                "gpio17": False, "gpio27": False, "simulated": True}
    return {
        "gpio4":     GPIO.input(PIN_INPUT_4)     == GPIO.HIGH,
        "gpio26":    GPIO.input(PIN_INPUT_26)    == GPIO.HIGH,
        "gpio17":    GPIO.input(PIN_LASER_START) == GPIO.HIGH,
        "gpio27":    GPIO.input(PIN_LASER_STOP)  == GPIO.HIGH,
        "simulated": False
    }

def set_gpio_output(pin, state):
    name = {PIN_LASER_START: "GPIO17 (Laser START)",
            PIN_LASER_STOP:  "GPIO27 (Laser STOP)"}.get(pin, f"GPIO{pin}")
    if not GPIO_AVAILABLE:
        log(f"[SIM] {name} → {'HIGH' if state else 'LOW'}")
        return
    GPIO.output(pin, GPIO.HIGH if state else GPIO.LOW)
    log(f"{name} → {'HIGH (relay ON)' if state else 'LOW (relay OFF)'}")

def pulse_output(pin, duration_s=0.5):
    name = {PIN_LASER_START: "GPIO17",
            PIN_LASER_STOP:  "GPIO27"}.get(pin, f"GPIO{pin}")
    log(f"Pulse {name} HIGH for {duration_s}s")
    set_gpio_output(pin, True)
    time.sleep(duration_s)
    set_gpio_output(pin, False)
    log(f"Pulse {name} complete")

# ═════════════════════════════════════════════════════════════════════════════
# LOG
# ═════════════════════════════════════════════════════════════════════════════

_log = []
_log_lock = threading.Lock()

def log(msg):
    ts = time.strftime("%H:%M:%S")
    line = f"[{ts}] {msg}"
    print(line)
    with _log_lock:
        _log.append(line)
        if len(_log) > 300:
            _log[:] = _log[-300:]

def get_log():
    with _log_lock:
        return list(_log)

# ═════════════════════════════════════════════════════════════════════════════
# MOTOR CONTROL
# ═════════════════════════════════════════════════════════════════════════════

_motor_busy  = False
_stop_flag   = False
_motor_lock  = threading.Lock()

def servo_on():
    return write16(0x04, 0x11, 1)

def servo_off():
    return write16(0x04, 0x11, 0)

def emergency_stop():
    """Immediately disable servo — stops motor as fast as possible."""
    global _stop_flag
    _stop_flag = True
    servo_off()
    log("=== MOTOR STOPPED ===")

def get_drive_status():
    state = read16(0x41, 0x0A)
    speed = read16(0x40, 0x01)
    pos   = read32(0x40, 0x16)
    do    = read16(0x40, 0x05)
    pos_mm = round(pos / PULSES_PER_MM, 3) if pos is not None else None
    return {
        "connected":   ser is not None and ser.is_open,
        "state":       state,
        "state_text":  {0:"Not ready", 1:"Ready",
                        2:"Running",   3:"FAULT"}.get(state, "Unknown"),
        "speed_rpm":   speed,
        "position_mm": pos_mm,
        "do_bits":     do,
        "homing_done": (do & 0x10) == 0 if do is not None else None,
        "busy":        _motor_busy,
    }

# ── Homing — Method 17 ───────────────────────────────────────────────────────
# Method 17: motor moves in REVERSE (negative/DOWN) direction at high speed.
# When NL status changes from OFF→ON, that position is used as home (0mm).
# No Z pulse search. Your system: 0mm = bottom, positive = up.
# NL switch must be at the BOTTOM of travel.
#
# Two scenarios handled automatically by the drive:
#   - NL inactive on start → motor moves reverse (down) at high speed until NL triggers
#   - NL already active on start → motor moves forward (up) at low speed
#     until NL clears, then moves reverse again to find the NL edge

def do_home():
    global _motor_busy, _stop_flag
    with _motor_lock:
        _motor_busy = True
    _stop_flag = False
    try:
        log("=== HOMING START (Method 17 — NL reverse (DOWN)) ===")
        log("Motor will move DOWN (reverse) toward NL switch at bottom")
        log("NL switch triggers → that position = 0mm home")

        servo_off()
        time.sleep(0.3)

        if _stop_flag:
            log("Homing cancelled before start")
            return

        # Configure Method 17
        write16(0x10, 0x01, 17)       # C10.01 — method 17 (NL, reverse, no Z pulse)
        write16(0x10, 0x02, 50)       # C10.02 — fast speed 50 rpm
        write16(0x10, 0x03, 10)       # C10.03 — slow speed 10 rpm
        write32(0x10, 0x04, 1000)     # C10.04 — accel ms
        write32(0x10, 0x06, 1000)     # C10.06 — decel ms
        write32(0x10, 0x08, 200000)   # C10.08 — timeout 200s
        write32(0x10, 0x0B, -6250)    # C10.0B — offset -2.5mm (-2.5 × 2500 pulses)

        # Stage 1: servo ready check
        servo_on()
        time.sleep(0.3)

        if _stop_flag:
            log("Homing cancelled after servo on")
            servo_off(); return

        state = read16(0x41, 0x0A)
        if state not in (1, 2):
            log(f"Servo not ready (state={state}) — check drive")
            servo_off(); return

        # Stage 2: trigger homing
        ok = write16(0x10, 0x00, 1)
        if not ok:
            log("Failed to write homing trigger")
            servo_off(); return

        # Stage 3: confirm physical motion within 3s
        deadline = time.time() + 3.0
        moving = False
        while time.time() < deadline:
            if _stop_flag:
                log("Homing stopped by user")
                write16(0x10, 0x00, 0)
                servo_off(); return
            spd = read16(0x40, 0x01)
            if spd and abs(spd) > 2:
                moving = True; break
            time.sleep(0.1)

        if not moving:
            log("No motion detected — check wiring / NL switch / drive fault")
            write16(0x10, 0x00, 0)
            servo_off(); return

        log("Homing in progress — moving DOWN toward NL switch...")

        # Stage 4: wait for completion
        # Conditions: DO5 bit4 LOW + speed=0 + position stable (2 reads)
        deadline = time.time() + 200
        prev_pos = None
        stable   = 0
        while time.time() < deadline:
            if _stop_flag:
                log("Homing stopped by user")
                write16(0x10, 0x00, 0)
                servo_off(); return

            do  = read16(0x40, 0x05)
            spd = abs(read16(0x40, 0x01) or 0)
            pos = read32(0x40, 0x16)

            bit4_low = (do & 0x10) == 0 if do is not None else False

            if bit4_low and spd == 0:
                if pos == prev_pos:
                    stable += 1
                else:
                    stable = 0
                prev_pos = pos
                if stable >= 2:
                    break
            time.sleep(0.2)
        else:
            log("Homing timeout — NL switch not reached within 200s")
            write16(0x10, 0x00, 0)
            servo_off(); return

        # Complete
        write16(0x10, 0x00, 0)   # reset trigger
        servo_off()
        pos_mm = (read32(0x40, 0x16) or 0) / PULSES_PER_MM
        log(f"=== HOMING COMPLETE — position: {pos_mm:.2f}mm ===")
        log("Moving to position 0mm...")
        do_move(0.0, 100)
        log("=== AT POSITION 0mm ===")


    except Exception as e:
        log(f"Homing error: {e}")
        write16(0x10, 0x00, 0)
        servo_off()
    finally:
        with _motor_lock:
            _motor_busy = False

# ── Move ─────────────────────────────────────────────────────────────────────

def do_move(target_mm, speed_rpm=300):
    global _motor_busy, _stop_flag
    with _motor_lock:
        _motor_busy = True
    _stop_flag = False
    try:
        if not (0 <= target_mm <= MAX_TRAVEL_MM):
            log(f"Target {target_mm}mm out of range [0–{MAX_TRAVEL_MM}]"); return

        pulses = int(target_mm * PULSES_PER_MM)
        log(f"=== MOVE TO {target_mm:.2f}mm ({pulses} pulses, {speed_rpm} rpm) ===")

        servo_off(); time.sleep(0.2)

        if _stop_flag:
            log("Move cancelled"); return

        write16(0x00, 0x00, 0)
        write16(0x03, 0x00, 1)
        write16(0x11, 0x00, 3)   # PP mode
        write16(0x11, 0x01, 0)   # absolute
        write16(0x11, 0x02, 1)   # immediate (not cache)
        write16(0x11, 0x03, 1)
        write16(0x11, 0x04, 1)
        write16(0x04, 0x14, 19)  # DI6 = trigger

        servo_on(); time.sleep(0.2)

        write32(0x11, 0x06, pulses)
        write16(0x11, 0x08, speed_rpm)
        write32(0x11, 0x0A, 500)
        write32(0x11, 0x0C, 500)

        # Edge trigger
        write16(0x04, 0x15, 1)
        time.sleep(0.3)
        write16(0x04, 0x15, 0)

        time.sleep(0.5)
        deadline = time.time() + 30
        while time.time() < deadline:
            if _stop_flag:
                log("Move stopped by user")
                servo_off(); return
            spd = read16(0x40, 0x01)
            if spd is not None and abs(spd) == 0:
                break
            time.sleep(0.1)
        else:
            log("Move timeout"); servo_off(); return

        servo_off()
        pos_mm = (read32(0x40, 0x16) or 0) / PULSES_PER_MM
        log(f"=== MOVE COMPLETE — position: {pos_mm:.2f}mm ===")

    except Exception as e:
        log(f"Move error: {e}")
        servo_off()
    finally:
        with _motor_lock:
            _motor_busy = False

def do_jog(direction, distance_mm, speed_rpm=200):
    pos = read32(0x40, 0x16)
    if pos is None:
        log("Cannot jog — cannot read position"); return
    cur_mm = pos / PULSES_PER_MM
    target = cur_mm + distance_mm if direction == "up" else cur_mm - distance_mm
    target = max(0.0, min(MAX_TRAVEL_MM, target))
    log(f"Jog {direction} {distance_mm}mm → {target:.2f}mm")
    do_move(target, speed_rpm)

def run_in_thread(fn, *args):
    if _motor_busy:
        log("Motor busy — stop current operation first")
        return False
    t = threading.Thread(target=fn, args=args, daemon=True)
    t.start()
    return True

# ═════════════════════════════════════════════════════════════════════════════
# HTML
# ═════════════════════════════════════════════════════════════════════════════

HTML = """
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Hardware Test</title>
<style>
  *, *::before, *::after { box-sizing: border-box; margin: 0; padding: 0; }
  :root {
    --bg:#0f1117; --surface:#1a1d27; --border:#2a2d3a;
    --text:#e2e4ed; --muted:#7a7d8e;
    --blue:#4a8fff; --green:#3ecf6e; --amber:#f0a030;
    --red:#f04040; --r:10px;
  }
  body { background:var(--bg); color:var(--text);
         font-family:'Segoe UI',system-ui,sans-serif; font-size:14px; }
  header { background:var(--surface); border-bottom:1px solid var(--border);
           padding:14px 24px; display:flex; align-items:center; gap:12px; }
  header h1 { font-size:16px; font-weight:600; }
  .badge { padding:3px 10px; border-radius:20px; font-size:11px; font-weight:600; }
  .badge-ok  { background:#3ecf6e22; color:var(--green); border:1px solid #3ecf6e44; }
  .badge-err { background:#f0404022; color:var(--red);   border:1px solid #f0404044; }
  .badge-warn{ background:#f0a03022; color:var(--amber); border:1px solid #f0a03044; }
  .layout { display:grid; grid-template-columns:1fr 1fr; gap:20px;
            padding:20px; max-width:1100px; margin:0 auto; }
  .section { display:flex; flex-direction:column; gap:16px; }
  .card { background:var(--surface); border:1px solid var(--border);
          border-radius:var(--r); padding:16px; }
  .card-title { font-size:11px; font-weight:600; letter-spacing:.08em;
                color:var(--muted); text-transform:uppercase; margin-bottom:12px; }
  .btn { display:inline-flex; align-items:center; justify-content:center;
         gap:6px; padding:8px 16px; border-radius:8px; border:none;
         font-size:13px; font-weight:600; cursor:pointer;
         transition:opacity .15s, transform .1s; width:100%; margin-bottom:6px; }
  .btn:active { transform:scale(.97); }
  .btn:disabled { opacity:.4; cursor:not-allowed; }
  .btn-blue   { background:var(--blue);  color:#fff; }
  .btn-green  { background:var(--green); color:#0a1a10; }
  .btn-amber  { background:var(--amber); color:#1a0f00; }
  .btn-red    { background:var(--red);   color:#fff; }
  .btn-ghost  { background:transparent; color:var(--text);
                border:1px solid var(--border); }
  .btn-row { display:grid; grid-template-columns:1fr 1fr; gap:6px; }
  .btn-row .btn { margin-bottom:0; }
  /* STOP button — always full width, prominent */
  .btn-stop-all {
    background:var(--red); color:#fff; font-size:15px;
    padding:12px; border-radius:10px; border:none; width:100%;
    font-weight:700; cursor:pointer; margin-bottom:8px;
    transition:opacity .15s, transform .1s;
    letter-spacing:.04em;
  }
  .btn-stop-all:active { transform:scale(.97); }
  input[type=number] {
    width:100%; background:var(--bg); border:1px solid var(--border);
    border-radius:8px; color:var(--text); padding:7px 10px;
    font-size:13px; margin-bottom:8px; outline:none; }
  input:focus { border-color:var(--blue); }
  label { font-size:12px; color:var(--muted); display:block; margin-bottom:3px; }
  .stat-grid { display:grid; grid-template-columns:1fr 1fr; gap:8px; margin-bottom:12px; }
  .stat { background:var(--bg); border:1px solid var(--border);
          border-radius:8px; padding:10px 12px; }
  .stat-label { font-size:11px; color:var(--muted); margin-bottom:2px; }
  .stat-value { font-size:17px; font-weight:700; font-variant-numeric:tabular-nums; }
  .gpio-grid { display:grid; grid-template-columns:1fr 1fr; gap:8px; }
  .gpio-card { background:var(--bg); border:1px solid var(--border);
               border-radius:8px; padding:12px; }
  .gpio-label { font-size:11px; color:var(--muted); margin-bottom:4px; }
  .gpio-name  { font-size:13px; font-weight:600; margin-bottom:8px; }
  .gpio-state { display:flex; align-items:center; gap:8px; margin-bottom:8px; }
  .dot { width:12px; height:12px; border-radius:50%;
         background:var(--border); transition:background .3s; flex-shrink:0; }
  .dot.high { background:var(--green); box-shadow:0 0 6px #3ecf6e44; }
  .dot.low  { background:var(--red);   box-shadow:0 0 6px #f0404044; }
  .state-text { font-size:12px; font-weight:600; }
  .log-box { background:var(--bg); border:1px solid var(--border);
             border-radius:8px; padding:10px; font-family:monospace;
             font-size:11px; color:#9ca3af; height:240px; overflow-y:auto; }
  .log-line { margin-bottom:2px; }
  .log-ok   { color:var(--green); }
  .log-err  { color:var(--red); }
  .log-warn { color:var(--amber); }
  .jog-dist-row { display:flex; gap:6px; flex-wrap:wrap; margin-bottom:8px; }
  .jog-chip { padding:5px 10px; border-radius:6px; border:1px solid var(--border);
              background:var(--bg); color:var(--muted); cursor:pointer;
              font-size:12px; font-weight:600; transition:all .15s; }
  .jog-chip.active { border-color:var(--blue); color:var(--blue); background:#4a8fff11; }
  .info-box { background:#4a8fff11; border:1px solid #4a8fff33; border-radius:8px;
              padding:10px 12px; font-size:12px; color:#a8c4ff; margin-bottom:10px;
              line-height:1.6; }
  .divider { border:none; border-top:1px solid var(--border); margin:8px 0; }
</style>
</head>
<body>

<header>
  <h1>🔧 Hardware Test</h1>
  <span id="conn-badge" class="badge badge-err">Serial: disconnected</span>
  <span id="gpio-badge" class="badge badge-err" style="margin-left:4px">GPIO: —</span>
  <span id="busy-badge" class="badge" style="margin-left:4px;display:none">⚙ Motor busy</span>
</header>

<div class="layout">

  <!-- ══ LEFT — Motor ══════════════════════════════════════════════════════ -->
  <div class="section">

    <!-- STOP button — always visible at top -->
    <div class="card" style="border-color:#f0404066">
      <button class="btn-stop-all" onclick="stopMotor()">⏹ STOP MOTOR</button>
      <div style="font-size:11px;color:var(--muted);text-align:center">
        Immediately disables servo — stops homing, move, or jog
      </div>
    </div>

    <div class="card">
      <div class="card-title">Drive status</div>
      <div class="stat-grid">
        <div class="stat">
          <div class="stat-label">State</div>
          <div class="stat-value" id="m-state">—</div>
        </div>
        <div class="stat">
          <div class="stat-label">Position</div>
          <div class="stat-value" id="m-pos">—</div>
        </div>
        <div class="stat">
          <div class="stat-label">Speed</div>
          <div class="stat-value" id="m-speed">—</div>
        </div>
        <div class="stat">
          <div class="stat-label">DO bits</div>
          <div class="stat-value" id="m-do">—</div>
        </div>
      </div>
      <div class="btn-row">
        <button class="btn btn-ghost" onclick="reconnect()">↻ Reconnect</button>
        <button class="btn btn-ghost" onclick="clearFault()">⚠ Clear fault</button>
      </div>
    </div>

    <div class="card">
      <div class="card-title">Homing — Method 17</div>
      <div class="info-box">
        Motor moves <strong>reverse (DOWN)</strong> until NL switch triggers →
        that point becomes <strong>0mm home</strong>.<br>
        NL switch must be at the <strong>bottom</strong> of travel.
      </div>
      <button class="btn btn-amber" id="btn-home" onclick="doHome()">⌂ Start homing</button>
      <button class="btn btn-red"   id="btn-stop-home" onclick="stopMotor()">✕ Stop homing</button>
    </div>

    <div class="card">
      <div class="card-title">Move to position</div>
      <label>Target (mm)</label>
      <input type="number" id="move-mm" min="0" max="420" step="1" value="50">
      <label>Speed (rpm)</label>
      <input type="number" id="move-rpm" min="10" max="3000" step="10" value="300">
      <button class="btn btn-blue"  id="btn-move" onclick="doMove()">→ Move to position</button>
      <button class="btn btn-ghost" onclick="stopMotor()">✕ Stop move</button>
    </div>

    <div class="card">
      <div class="card-title">Jog</div>
      <div class="jog-dist-row" id="jog-chips">
        <span class="jog-chip" onclick="selectJog(0.1)">0.1mm</span>
        <span class="jog-chip" onclick="selectJog(0.5)">0.5mm</span>
        <span class="jog-chip active" onclick="selectJog(1)">1mm</span>
        <span class="jog-chip" onclick="selectJog(5)">5mm</span>
        <span class="jog-chip" onclick="selectJog(10)">10mm</span>
        <span class="jog-chip" onclick="selectJog(50)">50mm</span>
      </div>
      <div class="btn-row">
        <button class="btn btn-ghost" onclick="doJog('up')">▲ Up</button>
        <button class="btn btn-ghost" onclick="doJog('down')">▼ Down</button>
      </div>
    </div>

  </div>

  <!-- ══ RIGHT — GPIO + Log ════════════════════════════════════════════════ -->
  <div class="section">

    <div class="card">
      <div class="card-title">GPIO inputs (from laser)</div>
      <div class="gpio-grid">
        <div class="gpio-card">
          <div class="gpio-label">INPUT</div>
          <div class="gpio-name">GPIO 4</div>
          <div class="gpio-state">
            <span class="dot" id="dot-4"></span>
            <span class="state-text" id="txt-4">—</span>
          </div>
          <div style="font-size:11px;color:var(--muted)">Signal from laser (TBD)</div>
        </div>
        <div class="gpio-card">
          <div class="gpio-label">INPUT</div>
          <div class="gpio-name">GPIO 26</div>
          <div class="gpio-state">
            <span class="dot" id="dot-26"></span>
            <span class="state-text" id="txt-26">—</span>
          </div>
          <div style="font-size:11px;color:var(--muted)">Signal from laser (TBD)</div>
        </div>
      </div>
    </div>

    <div class="card">
      <div class="card-title">GPIO outputs (to laser) — 3V relay, active HIGH</div>
      <div class="gpio-grid">
        <div class="gpio-card">
          <div class="gpio-label">OUTPUT</div>
          <div class="gpio-name">GPIO 17 — Laser START</div>
          <div class="gpio-state">
            <span class="dot" id="dot-17"></span>
            <span class="state-text" id="txt-17">—</span>
          </div>
          <div class="btn-row" style="margin-top:8px">
            <button class="btn btn-green" style="font-size:12px;padding:6px"
                    onclick="setOutput(17,true)">HIGH</button>
            <button class="btn btn-ghost" style="font-size:12px;padding:6px"
                    onclick="setOutput(17,false)">LOW</button>
          </div>
          <button class="btn btn-amber" style="margin-top:6px;font-size:12px;padding:6px"
                  onclick="pulseOutput(17)">Pulse 0.5s</button>
        </div>
        <div class="gpio-card">
          <div class="gpio-label">OUTPUT</div>
          <div class="gpio-name">GPIO 27 — Laser STOP</div>
          <div class="gpio-state">
            <span class="dot" id="dot-27"></span>
            <span class="state-text" id="txt-27">—</span>
          </div>
          <div class="btn-row" style="margin-top:8px">
            <button class="btn btn-green" style="font-size:12px;padding:6px"
                    onclick="setOutput(27,true)">HIGH</button>
            <button class="btn btn-ghost" style="font-size:12px;padding:6px"
                    onclick="setOutput(27,false)">LOW</button>
          </div>
          <button class="btn btn-amber" style="margin-top:6px;font-size:12px;padding:6px"
                  onclick="pulseOutput(27)">Pulse 0.5s</button>
        </div>
      </div>
    </div>

    <div class="card" style="flex:1">
      <div class="card-title">Log</div>
      <div class="log-box" id="log-box"></div>
      <button class="btn btn-ghost" style="margin-top:8px;font-size:12px"
              onclick="clearLog()">Clear log</button>
    </div>

  </div>
</div>

<script>
let jogDist = 1.0;

function selectJog(v) {
  jogDist = v;
  document.querySelectorAll('.jog-chip').forEach(c => {
    c.classList.toggle('active', parseFloat(c.textContent) === v);
  });
}

function poll() {
  fetch('/api/drive_status').then(r=>r.json()).then(updateDrive).catch(()=>{});
  fetch('/api/gpio_state').then(r=>r.json()).then(updateGPIO).catch(()=>{});
  fetch('/api/log').then(r=>r.json()).then(d=>updateLog(d.log)).catch(()=>{});
}
setInterval(poll, 600);
poll();

function updateDrive(d) {
  const cb = document.getElementById('conn-badge');
  cb.className = 'badge ' + (d.connected ? 'badge-ok' : 'badge-err');
  cb.textContent = d.connected ? 'Serial: connected' : 'Serial: disconnected';

  const bb = document.getElementById('busy-badge');
  bb.style.display = d.busy ? 'inline-block' : 'none';
  bb.className = 'badge badge-warn';

  const stateColors = {
    'Ready':    'var(--green)',
    'Running':  'var(--blue)',
    'FAULT':    'var(--red)',
    'Not ready':'var(--muted)'
  };
  const stEl = document.getElementById('m-state');
  stEl.textContent = d.state_text || '—';
  stEl.style.color = stateColors[d.state_text] || 'var(--text)';

  document.getElementById('m-pos').textContent =
    d.position_mm !== null ? d.position_mm + ' mm' : '—';
  document.getElementById('m-speed').textContent =
    d.speed_rpm !== null ? d.speed_rpm + ' rpm' : '—';
  document.getElementById('m-do').textContent =
    d.do_bits !== null ? '0x' + d.do_bits.toString(16).toUpperCase() : '—';

  const busy = d.busy;
  document.getElementById('btn-home').disabled = busy;
  document.getElementById('btn-move').disabled = busy;
}

function updateGPIO(d) {
  const gb = document.getElementById('gpio-badge');
  gb.className = 'badge ' + (d.simulated ? 'badge-err' : 'badge-ok');
  gb.textContent = d.simulated ? 'GPIO: simulated' : 'GPIO: active';
  setDot('dot-4',  'txt-4',  d.gpio4);
  setDot('dot-26', 'txt-26', d.gpio26);
  setDot('dot-17', 'txt-17', d.gpio17);
  setDot('dot-27', 'txt-27', d.gpio27);
}

function setDot(dotId, txtId, high) {
  document.getElementById(dotId).className = 'dot ' + (high ? 'high' : 'low');
  const t = document.getElementById(txtId);
  t.textContent = high ? 'HIGH' : 'LOW';
  t.style.color = high ? 'var(--green)' : 'var(--red)';
}

function updateLog(lines) {
  const box = document.getElementById('log-box');
  const atBottom = box.scrollHeight - box.scrollTop <= box.clientHeight + 20;
  box.innerHTML = lines.map(l => {
    const c = l.includes('error')||l.includes('Error')||l.includes('FAULT')||l.includes('STOP') ? 'log-err'
            : l.includes('COMPLETE')||l.includes('complete')||l.includes('HOMING COMPLETE') ? 'log-ok'
            : l.includes('timeout')||l.includes('WARN')||l.includes('cancelled') ? 'log-warn' : '';
    return `<div class="log-line ${c}">${esc(l)}</div>`;
  }).join('');
  if (atBottom) box.scrollTop = box.scrollHeight;
}

function clearLog() { fetch('/api/log/clear', {method:'POST'}); }

// Motor actions
function doHome()  { fetch('/api/home',      {method:'POST'}); }
function stopMotor() { fetch('/api/stop',    {method:'POST'}); }
function clearFault(){ fetch('/api/clear_fault',{method:'POST'}); }
function reconnect() { fetch('/api/reconnect',{method:'POST'}); }

function doMove() {
  const mm  = parseFloat(document.getElementById('move-mm').value);
  const rpm = parseInt(document.getElementById('move-rpm').value);
  fetch('/api/move', {
    method:'POST', headers:{'Content-Type':'application/json'},
    body: JSON.stringify({mm, rpm})
  });
}

function doJog(dir) {
  fetch('/api/jog', {
    method:'POST', headers:{'Content-Type':'application/json'},
    body: JSON.stringify({direction: dir, distance_mm: jogDist})
  });
}

// GPIO actions
function setOutput(pin, state) {
  fetch('/api/gpio/set', {
    method:'POST', headers:{'Content-Type':'application/json'},
    body: JSON.stringify({pin, state})
  });
}
function pulseOutput(pin) {
  fetch('/api/gpio/pulse', {
    method:'POST', headers:{'Content-Type':'application/json'},
    body: JSON.stringify({pin, duration: 0.5})
  });
}

function esc(s) {
  return String(s).replace(/&/g,'&amp;').replace(/</g,'&lt;').replace(/>/g,'&gt;');
}
</script>
</body>
</html>
"""

# ═════════════════════════════════════════════════════════════════════════════
# ROUTES
# ═════════════════════════════════════════════════════════════════════════════

@app.route("/")
def index():
    return render_template_string(HTML)

@app.route("/api/drive_status")
def api_drive_status():
    return jsonify(get_drive_status())

@app.route("/api/gpio_state")
def api_gpio_state():
    return jsonify(read_gpio_state())

@app.route("/api/log")
def api_log():
    return jsonify({"log": get_log()})

@app.route("/api/log/clear", methods=["POST"])
def api_log_clear():
    with _log_lock:
        _log.clear()
    return jsonify({"ok": True})

@app.route("/api/reconnect", methods=["POST"])
def api_reconnect():
    return jsonify({"ok": connect_serial()})

@app.route("/api/home", methods=["POST"])
def api_home():
    ok = run_in_thread(do_home)
    return jsonify({"ok": ok})

@app.route("/api/stop", methods=["POST"])
def api_stop():
    emergency_stop()
    return jsonify({"ok": True})

@app.route("/api/clear_fault", methods=["POST"])
def api_clear_fault():
    write16(0x31, 0x00, 1)
    log("Fault cleared")
    return jsonify({"ok": True})

@app.route("/api/move", methods=["POST"])
def api_move():
    d = request.get_json() or {}
    ok = run_in_thread(do_move, float(d.get("mm", 0)), int(d.get("rpm", 300)))
    return jsonify({"ok": ok})

@app.route("/api/jog", methods=["POST"])
def api_jog():
    d = request.get_json() or {}
    ok = run_in_thread(do_jog, d.get("direction", "up"),
                       float(d.get("distance_mm", 1.0)))
    return jsonify({"ok": ok})

@app.route("/api/gpio/set", methods=["POST"])
def api_gpio_set():
    d = request.get_json() or {}
    pin   = int(d.get("pin", 0))
    state = bool(d.get("state", False))
    if pin not in (PIN_LASER_START, PIN_LASER_STOP):
        return jsonify({"ok": False, "error": "Invalid pin"})
    set_gpio_output(pin, state)
    return jsonify({"ok": True})

@app.route("/api/gpio/pulse", methods=["POST"])
def api_gpio_pulse():
    d = request.get_json() or {}
    pin = int(d.get("pin", 0))
    dur = float(d.get("duration", 0.5))
    if pin not in (PIN_LASER_START, PIN_LASER_STOP):
        return jsonify({"ok": False, "error": "Invalid pin"})
    threading.Thread(target=pulse_output, args=(pin, dur), daemon=True).start()
    return jsonify({"ok": True})

# ═════════════════════════════════════════════════════════════════════════════

if __name__ == "__main__":
    log("=== Hardware Test App Starting ===")
    connect_serial()
    setup_gpio()
    app.run(host="0.0.0.0", port=5001, debug=False, threaded=True)