"""
laser_ctrl.py — Laser marking automation engine
Hardware: Raspberry Pi + A6-RS servo (RS485/Modbus RTU) + GPIO
"""

import time
import threading
import json
import os
import glob
import serial
import crcmod

try:
    import RPi.GPIO as GPIO
    GPIO_AVAILABLE = True
except (ImportError, RuntimeError):
    GPIO_AVAILABLE = False
    print("[WARN] RPi.GPIO not available — simulation mode")

# ═════════════════════════════════════════════════════════════════════════════
# CONSTANTS
# ═════════════════════════════════════════════════════════════════════════════

SERIAL_PORT    = "/dev/ttyACM0"
BAUD_RATE      = 115200
DRIVE_ADDR     = 0x01
# U40.16 and PP target positions are in drive COMMAND UNITS, not encoder pulses.
# Verify this value from C00.02 and the mechanical travel per motor revolution:
#   COMMAND_UNITS_PER_MM = C00.02 / mm_per_motor_revolution
COMMAND_UNITS_PER_MM = 2500
MAX_TRAVEL_MM        = 420.0
PART_HEIGHT_MIN_MM    = 60.0
PART_HEIGHT_MAX_MM    = 400.0

PARTS_DIR      = "parts"
SETTINGS_FILE  = "settings.json"

# GPIO pins (BCM)
PIN_LASER_START        = 17   # OUT - Pi software Mark -> EzCad2 GIN15
PIN_LASER_REMARK       = 27   # OUT - reserved remarking output
PIN_FOOT_PEDAL         = 22   # IN  - operator foot pedal acknowledgement to Pi
PIN_INTERMEDIATE_DONE  = 26   # IN  - EzCad2 OUT4 pulse: one intermediate side finished
PIN_PART_COMPLETE      = 4    # IN  - EzCad2 OUT5 pulse: complete part finished

# Legacy aliases used only by the existing Automatic-mode code.
PIN_MARKING_STAT = PIN_INTERMEDIATE_DONE
PIN_MARKING_DONE = PIN_PART_COMPLETE

# ═════════════════════════════════════════════════════════════════════════════
# DEFAULT SETTINGS
# ═════════════════════════════════════════════════════════════════════════════

DEFAULT_SETTINGS = {
    "homing": {
        "speed_fast":  50,       # rpm
        "speed_slow":  10,       # rpm
        "timeout":     200,      # seconds
        "offset_mm":   -2.5,     # mm
    },
    "laser": {
        "pulse_ms":    100,      # ms — laser trigger pulse duration
    },
    "absolute_position": {
        # This becomes True only after a successful machine homing.
        # It is intentionally persistent so normal power cycles do not
        # require homing again when the battery-backed encoder retains data.
        "reference_established": False,
    }
}

# ═════════════════════════════════════════════════════════════════════════════
# STATE MACHINE
# ═════════════════════════════════════════════════════════════════════════════

class State:
    IDLE             = "IDLE"
    HOMING           = "HOMING"
    READY            = "READY"
    MOVING_Z         = "MOVING_Z"
    WAITING_PLACE    = "WAITING_PLACE"
    LASER_FIRING     = "LASER_FIRING"
    WAITING_LASER    = "WAITING_LASER"
    WAITING_ROBOT    = "WAITING_ROBOT"
    PAUSED           = "PAUSED"
    ERROR            = "ERROR"
    PROGRAM_SELECTED = "PROGRAM_SELECTED"
    INITIALIZING     = "INITIALIZING"
    READY_TO_MARK    = "READY_TO_MARK"
    PROGRAM_STOPPED  = "PROGRAM_STOPPED"
    RESETTING        = "RESETTING"

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
# DRIVE
# ═════════════════════════════════════════════════════════════════════════════

class A6Drive:
    def __init__(self):
        self.ser   = None
        self._lock = threading.Lock()

    def connect(self):
        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE,
                                     bytesize=8, parity='N', stopbits=1,
                                     timeout=0.5)
            return True
        except Exception as e:
            print(f"[DRIVE] Connect failed: {e}")
            return False

    def disconnect(self):
        if self.ser and self.ser.is_open:
            self.ser.close()

    def _send(self, frame, read_n=8):
        if not self.ser or not self.ser.is_open:
            return None
        with self._lock:
            try:
                self.ser.reset_input_buffer()
                self.ser.write(frame)
                time.sleep(0.02)
                return self.ser.read(read_n)
            except Exception as e:
                print(f"[DRIVE] Serial error: {e}")
                return None

    def write16(self, grp, off, val):
        r = self._send(w16(grp, off, val))
        return r is not None and len(r) >= 6

    def write32(self, grp, off, val):
        r = self._send(w32(grp, off, val), 12)
        return r is not None and len(r) >= 6

    def read16(self, grp, off):
        resp = self._send(r(grp, off, 1), 7)
        if resp and len(resp) >= 7:
            v = (resp[3] << 8) | resp[4]
            return v - 65536 if v > 32767 else v
        return None

    def read32(self, grp, off):
        resp = self._send(r(grp, off, 2), 9)
        if resp and len(resp) >= 9:
            lo = (resp[3] << 8) | resp[4]
            hi = (resp[5] << 8) | resp[6]
            v  = (hi << 16) | lo
            return v - 0x100000000 if v > 0x7FFFFFFF else v
        return None

    def servo_on(self):  return self.write16(0x04, 0x11, 1)
    def servo_off(self): return self.write16(0x04, 0x11, 0)

    def get_speed(self):
        v = self.read16(0x40, 0x01)
        return v if v is not None else 0

    def get_position_raw(self):
        """Return U40.16 absolute position feedback in command units."""
        return self.read32(0x40, 0x16)

    def get_position_mm(self):
        p = self.get_position_raw()
        return round(p / COMMAND_UNITS_PER_MM, 3) if p is not None else None

    def get_encoder_position_raw(self):
        """Return U40.18 absolute position feedback in encoder units."""
        return self.read32(0x40, 0x18)

    def get_state(self):
        return self.read16(0x41, 0x0A)

    def get_do_bits(self):
        return self.read16(0x40, 0x05)

    def jog(self, direction, distance_mm, speed_rpm=200):
        """Jog up or down by distance_mm from current position."""
        cur = self.get_position_mm()
        if cur is None:
            return False
        target = cur + distance_mm if direction == "up" else cur - distance_mm
        target = max(0.0, min(MAX_TRAVEL_MM, target))
        return self.move_to(target, speed_rpm)

    def connected(self):
        return self.ser is not None and self.ser.is_open

    def clear_fault(self):
        self.write16(0x31, 0x00, 1)
        time.sleep(0.2)

    def move_to(self, target_mm, speed_rpm=300, stop_flag=None, log_cb=None):
        """PP mode absolute move."""
        def log(m):
            if log_cb: log_cb(m)

        if not (0.0 <= target_mm <= MAX_TRAVEL_MM):
            log(f"Target {target_mm}mm out of range"); return False

        command_units = int(target_mm * COMMAND_UNITS_PER_MM)
        log(f"Moving to {target_mm:.2f}mm...")

        self.servo_off(); time.sleep(0.05)

        self.write16(0x00, 0x00, 0)
        self.write16(0x03, 0x00, 1)
        self.write16(0x11, 0x00, 3)
        self.write16(0x11, 0x01, 0)
        self.write16(0x11, 0x02, 1)
        self.write16(0x11, 0x03, 1)
        self.write16(0x11, 0x04, 1)
        self.write16(0x04, 0x14, 19)

        self.servo_on(); time.sleep(0.05)

        self.write32(0x11, 0x06, command_units)
        self.write16(0x11, 0x08, speed_rpm)
        self.write32(0x11, 0x0A, 200)
        self.write32(0x11, 0x0C, 200)

        self.write16(0x04, 0x15, 1)
        time.sleep(0.05)
        self.write16(0x04, 0x15, 0)

        time.sleep(0.1)
        deadline = time.time() + 30
        while time.time() < deadline:
            if stop_flag and stop_flag():
                log("Move stopped"); self.servo_off(); return False
            if abs(self.get_speed()) == 0:
                break
            time.sleep(0.1)
        else:
            log("Move timeout"); self.servo_off(); return False

        self.servo_off()
        log(f"Arrived at {self.get_position_mm():.2f}mm")
        return True

    def home(self, speed_fast, speed_slow, timeout_s, offset_mm,
             stop_flag=None, log_cb=None):
        """Method 17 homing — moves DOWN to NL switch."""
        def log(m):
            if log_cb: log_cb(m)

        offset_units = int(offset_mm * COMMAND_UNITS_PER_MM)
        log("=== HOMING START (Method 17 — moving DOWN) ===")

        self.servo_off(); time.sleep(0.3)

        self.write16(0x10, 0x01, 17)
        self.write16(0x10, 0x02, speed_fast)
        self.write16(0x10, 0x03, speed_slow)
        self.write32(0x10, 0x04, 1000)
        self.write32(0x10, 0x06, 1000)
        self.write32(0x10, 0x08, int(timeout_s * 1000))
        self.write32(0x10, 0x0B, offset_units)

        self.servo_on(); time.sleep(0.3)

        if stop_flag and stop_flag():
            log("Homing cancelled"); self.servo_off(); return False

        state = self.get_state()
        if state not in (1, 2):
            log(f"Servo not ready (state={state})"); self.servo_off(); return False

        self.write16(0x10, 0x00, 1)

        # Confirm motion
        deadline = time.time() + 3
        moving = False
        while time.time() < deadline:
            if stop_flag and stop_flag(): break
            if abs(self.get_speed()) > 2:
                moving = True; break
            time.sleep(0.1)

        if not moving:
            log("No motion — check NL switch wiring")
            self.write16(0x10, 0x00, 0); self.servo_off(); return False

        log("Moving DOWN to NL switch...")

        # Wait for completion
        deadline = time.time() + timeout_s
        prev_pos = None; stable = 0
        while time.time() < deadline:
            if stop_flag and stop_flag():
                log("Homing stopped by user")
                self.write16(0x10, 0x00, 0); self.servo_off(); return False
            do  = self.get_do_bits()
            spd = abs(self.get_speed())
            pos = self.read32(0x40, 0x16)
            if do is not None and (do & 0x10) == 0 and spd == 0:
                if pos == prev_pos: stable += 1
                else: stable = 0
                prev_pos = pos
                if stable >= 2: break
            time.sleep(0.2)
        else:
            log(f"Homing timeout after {timeout_s}s")
            self.write16(0x10, 0x00, 0); self.servo_off(); return False

        self.write16(0x10, 0x00, 0)
        self.servo_off()
        log(f"Homing complete — position: {self.get_position_mm():.2f}mm")

        # Move to 0mm
        log("Moving to position 0mm...")
        ok = self.move_to(0.0, 100, stop_flag=stop_flag, log_cb=log_cb)
        if ok:
            log("=== AT POSITION 0mm — READY ===")
        return ok

# ═════════════════════════════════════════════════════════════════════════════
# GPIO
# ═════════════════════════════════════════════════════════════════════════════

class GPIOManager:
    def __init__(self):
        self._sim = not GPIO_AVAILABLE
        if not self._sim:
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(PIN_LASER_START,       GPIO.OUT, initial=GPIO.LOW)
            GPIO.setup(PIN_LASER_REMARK,      GPIO.OUT, initial=GPIO.LOW)
            GPIO.setup(PIN_FOOT_PEDAL,        GPIO.IN,  pull_up_down=GPIO.PUD_DOWN)
            GPIO.setup(PIN_INTERMEDIATE_DONE, GPIO.IN,  pull_up_down=GPIO.PUD_DOWN)
            GPIO.setup(PIN_PART_COMPLETE,     GPIO.IN,  pull_up_down=GPIO.PUD_DOWN)

    def pulse_laser_start(self, pulse_ms=100):
        """Software MARK: pulse GPIO17 to EzCad2 GIN15."""
        dur = pulse_ms / 1000.0
        if not self._sim:
            GPIO.output(PIN_LASER_START, GPIO.HIGH)
            time.sleep(dur)
            GPIO.output(PIN_LASER_START, GPIO.LOW)
        else:
            print(f"[GPIO SIM] Laser start pulse {pulse_ms}ms")
            time.sleep(dur)

    def foot_pedal(self):
        """GPIO22 HIGH means the physical foot pedal has been pressed."""
        if self._sim:
            return False
        return GPIO.input(PIN_FOOT_PEDAL) == GPIO.HIGH

    def intermediate_finish(self):
        """GPIO26 / EzCad2 OUT4 pulse: an intermediate side has finished."""
        if self._sim:
            return False
        return GPIO.input(PIN_INTERMEDIATE_DONE) == GPIO.HIGH

    def part_complete(self):
        """GPIO4 / EzCad2 OUT5 pulse: the complete part has finished."""
        if self._sim:
            return False
        return GPIO.input(PIN_PART_COMPLETE) == GPIO.HIGH

    # Legacy names retained so the not-yet-redesigned Automatic mode continues
    # to import/run exactly as before.  Manual mode does NOT use these methods.
    def marking_active(self):
        return self.intermediate_finish()

    def marking_done(self):
        return self.part_complete()

    def read_all(self):
        if self._sim:
            return {
                "gpio4": False, "gpio22": False, "gpio26": False,
                "gpio17": False, "gpio27": False, "simulated": True,
            }
        return {
            "gpio4":  GPIO.input(PIN_PART_COMPLETE)     == GPIO.HIGH,
            "gpio22": GPIO.input(PIN_FOOT_PEDAL)        == GPIO.HIGH,
            "gpio26": GPIO.input(PIN_INTERMEDIATE_DONE) == GPIO.HIGH,
            "gpio17": GPIO.input(PIN_LASER_START)       == GPIO.HIGH,
            "gpio27": GPIO.input(PIN_LASER_REMARK)      == GPIO.HIGH,
            "simulated": False,
        }

    def cleanup(self):
        if not self._sim:
            GPIO.cleanup()

# ═════════════════════════════════════════════════════════════════════════════
# SETTINGS
# ═════════════════════════════════════════════════════════════════════════════

class SettingsManager:
    def __init__(self):
        self._file = SETTINGS_FILE
        self._data = self._load()

    def _load(self):
        if os.path.exists(self._file):
            try:
                with open(self._file) as f:
                    saved = json.load(f)
                # Merge with defaults to handle missing keys
                merged = json.loads(json.dumps(DEFAULT_SETTINGS))
                for section, vals in saved.items():
                    if section in merged:
                        merged[section].update(vals)
                return merged
            except Exception:
                pass
        return json.loads(json.dumps(DEFAULT_SETTINGS))

    def save(self):
        with open(self._file, 'w') as f:
            json.dump(self._data, f, indent=2)

    def get(self, section, key):
        return self._data.get(section, {}).get(key)

    def set(self, section, key, value):
        if section not in self._data:
            self._data[section] = {}
        self._data[section][key] = value
        self.save()

    def get_section(self, section):
        return dict(self._data.get(section, {}))

    def update_section(self, section, data):
        if section not in self._data:
            self._data[section] = {}
        self._data[section].update(data)
        self.save()

# ═════════════════════════════════════════════════════════════════════════════
# PARTS
# ═════════════════════════════════════════════════════════════════════════════

class PartManager:
    def __init__(self):
        os.makedirs(PARTS_DIR, exist_ok=True)

    def list_parts(self):
        files = glob.glob(os.path.join(PARTS_DIR, "*.json"))
        return sorted([os.path.splitext(os.path.basename(f))[0] for f in files])

    def load(self, name):
        path = os.path.join(PARTS_DIR, f"{name}.json")
        if not os.path.exists(path):
            return None
        with open(path) as f:
            return json.load(f)

    def validate(self, data):
        """Every part must explicitly define side count and one Z height per side."""
        if not data:
            return False, "Part not found"
        name = str(data.get("name", "")).strip()
        if not name:
            return False, "Part name is required"

        sides = data.get("sides")
        try:
            sides = int(sides)
        except (TypeError, ValueError):
            return False, "Number of sides is required"
        if sides < 1:
            return False, "Part must have at least one side"

        positions = data.get("positions")
        if not isinstance(positions, list):
            return False, "A Z height is required for every side"
        if len(positions) != sides:
            return False, f"Part configuration incomplete: {sides} sides require {sides} height entries"

        for idx, p in enumerate(positions, start=1):
            if not isinstance(p, dict) or p.get("mm") in (None, ""):
                return False, f"Height for side {idx} is required"
            try:
                mm = float(p.get("mm"))
            except (TypeError, ValueError):
                return False, f"Height for side {idx} is invalid"
            if not (PART_HEIGHT_MIN_MM <= mm <= PART_HEIGHT_MAX_MM):
                return False, (
                    f"Side {idx} height {mm}mm is out of range "
                    f"[{PART_HEIGHT_MIN_MM:g}-{PART_HEIGHT_MAX_MM:g} mm]"
                )
        return True, "Valid"

    def save(self, data):
        ok, msg = self.validate(data)
        if not ok:
            return False, msg
        clean = {
            "name": str(data["name"]).strip(),
            "sides": int(data["sides"]),
            "positions": [],
        }
        for idx, p in enumerate(data["positions"], start=1):
            clean["positions"].append({
                "mm": float(p["mm"]),
                "label": str(p.get("label") or f"Side {idx}"),
            })
        path = os.path.join(PARTS_DIR, f"{clean['name']}.json")
        with open(path, 'w') as f:
            json.dump(clean, f, indent=2)
        return True, "Saved"

    def delete(self, name):
        path = os.path.join(PARTS_DIR, f"{name}.json")
        if os.path.exists(path):
            os.remove(path)
            return True
        return False

# ═════════════════════════════════════════════════════════════════════════════
# LASER CONTROLLER
# ═════════════════════════════════════════════════════════════════════════════

class LaserController:
    """Main controller.

    Manual mode uses an OUT4/OUT5 side protocol:
      - Program Start moves to side 1 Z and arms MARK / foot pedal.
      - UI MARK pulses GPIO17; foot pedal starts EzCad2 itself and GPIO22 only
        tells this controller that a cycle has started.
      - OUT4 advances to the next side and the Pi triggers that next side.
      - OUT5 completes the part, resets the OUT4 count, and prepares side 1
        again for the next physical part.
    """

    def __init__(self):
        self.drive    = A6Drive()
        self.gpio     = GPIOManager()
        self.settings = SettingsManager()
        self.parts    = PartManager()

        self.state          = State.IDLE
        self.mode           = "manual"
        self.current_part   = None
        self.current_step   = 0
        self.total_steps    = 0
        self.cycle_count    = 0
        self.total_cycles   = 0
        self.auto_advance   = False  # legacy Automatic/manual compatibility
        self.log_lines      = []

        # Absolute position cache.
        self.z_position                 = None
        self.z_position_raw             = None
        self.encoder_position_raw       = None
        self.encoder_position_available = False
        self.speed_rpm                    = 0
        self._telemetry_last_ok           = 0.0
        self._telemetry_stop              = False
        self._telemetry_thread            = None
        abs_cfg = self.settings.get_section("absolute_position")
        self.reference_established = bool(abs_cfg.get("reference_established", False))

        self._stop_flag      = False
        self._pause_flag     = False
        self._thread         = None
        self._lock           = threading.Lock()

        # Manual laser-program state.
        self.manual_selected_part_name = None
        self.manual_program_started    = False
        self.manual_stopped            = False
        self.manual_mark_enabled       = False
        self.manual_initial_ready      = False
        self.manual_awaiting_result    = False
        self.manual_side_index         = 0       # 0-based side currently marking / next initial side
        self.manual_out4_count         = 0       # intermediate-finish pulses in current part
        self.manual_parts_completed    = 0
        self.manual_pending_next_index = None
        self.manual_pending_return     = False
        self.manual_program_state      = "NO_PART"
        self.manual_error              = None
        self._manual_motion_stop       = False
        self._manual_operation_lock    = threading.Lock()
        self._gpio_monitor_stop        = False
        self._gpio_monitor_thread      = None

    # ── Log / status ────────────────────────────────────────────────────────

    def log(self, msg):
        ts = time.strftime("%H:%M:%S")
        line = f"[{ts}] {msg}"
        print(line)
        with self._lock:
            self.log_lines.append(line)
            if len(self.log_lines) > 300:
                self.log_lines = self.log_lines[-300:]

    def get_logs(self):
        with self._lock:
            return list(self.log_lines)

    def set_state(self, s):
        self.state = s
        self.log(f"State -> {s}")

    def _refresh_absolute_position(self):
        raw = self.drive.get_position_raw()
        if raw is not None:
            self.z_position_raw = raw
            self.z_position = round(raw / COMMAND_UNITS_PER_MM, 3)
            self.encoder_position_available = True
        return raw

    def position_ready(self):
        return bool(
            self.reference_established
            and self.encoder_position_available
            and self.drive.connected()
        )

    def get_status(self):
        # IMPORTANT: do not perform Modbus transactions in a Flask request.
        # Telemetry is refreshed in one background thread so a slow/unresponsive
        # drive cannot make browser status requests pile up and freeze the HMI.
        part = self.current_part
        return {
            "state": self.state,
            "mode": self.mode,
            "part": part["name"] if part else self.manual_selected_part_name,
            "step": self.manual_side_index + 1 if self.manual_program_started and part else 0,
            "total_steps": int(part.get("sides", 0)) if part else 0,
            "cycle_count": self.cycle_count,
            "total_cycles": self.total_cycles,
            "z_position": self.z_position,
            "z_position_raw": self.z_position_raw,
            "speed_rpm": self.speed_rpm,
            "connected": self.drive.connected(),
            "auto_advance": self.auto_advance,
            "encoder_position_available": self.encoder_position_available,
            "reference_established": self.reference_established,
            "position_ready": self.position_ready(),
            "homed": self.reference_established,

            # Manual-program status for GUI interlocks.
            "manual_selected_part": self.manual_selected_part_name,
            "manual_program_started": self.manual_program_started,
            "manual_program_state": self.manual_program_state,
            "manual_stopped": self.manual_stopped,
            "manual_mark_enabled": self.manual_mark_enabled,
            "manual_initial_ready": self.manual_initial_ready,
            "manual_awaiting_result": self.manual_awaiting_result,
            "manual_side_index": self.manual_side_index,
            "manual_side_number": self.manual_side_index + 1 if part else 0,
            "manual_total_sides": int(part.get("sides", 0)) if part else 0,
            "manual_out4_count": self.manual_out4_count,
            "manual_parts_completed": self.manual_parts_completed,
            "manual_error": self.manual_error,
        }

    # ── Telemetry cache ───────────────────────────────────────────────────

    def _start_telemetry(self):
        if self._telemetry_thread and self._telemetry_thread.is_alive():
            return
        self._telemetry_stop = False
        self._telemetry_thread = threading.Thread(
            target=self._telemetry_loop, name="drive-telemetry", daemon=True
        )
        self._telemetry_thread.start()

    def _telemetry_loop(self):
        while not self._telemetry_stop:
            if self.drive.connected():
                raw = self.drive.get_position_raw()
                if raw is not None:
                    self.z_position_raw = raw
                    self.z_position = round(raw / COMMAND_UNITS_PER_MM, 3)
                    self.encoder_position_available = True
                    self._telemetry_last_ok = time.monotonic()
                spd = self.drive.get_speed()
                if spd is not None:
                    self.speed_rpm = spd
            time.sleep(0.35)

    # ── Startup / absolute reference ────────────────────────────────────────

    def startup(self):
        self.log("=== Laser Controller Starting ===")
        if self.drive.connect():
            self.log(f"Drive connected on {SERIAL_PORT}")
            raw = None
            for _ in range(5):
                raw = self._refresh_absolute_position()
                if raw is not None:
                    break
                time.sleep(0.20)
            if raw is not None:
                self.log(f"Absolute position read: {raw} command units ({self.z_position:.3f} mm)")
                if self.reference_established:
                    self.log("Machine reference valid - homing not required after this power cycle")
                else:
                    self.log("Absolute encoder readable, but machine reference is not established - home once")
            else:
                self.encoder_position_available = False
                self.log("Could not read U40.16 absolute position - do not trust machine position")
            # From here on, position/speed are refreshed in one background
            # telemetry thread; Flask status requests only read cached values.
            self._start_telemetry()
            self.set_state(State.READY)
            self._start_gpio_monitor()
        else:
            self.log("Drive connection failed")
            self.set_state(State.ERROR)

    def start_homing(self):
        if self.manual_program_started:
            self.log("Cannot home while Laser Program is started - Reset/finish program first")
            return False
        if self.state not in (State.READY, State.ERROR, State.IDLE, State.PROGRAM_SELECTED):
            self.log("Cannot home - system busy")
            return False
        self._stop_flag = False
        threading.Thread(target=self._do_home, daemon=True).start()
        return True

    def stop_homing(self):
        self._stop_flag = True
        self.log("Homing stop requested")

    def _do_home(self):
        self.set_state(State.HOMING)
        h = self.settings.get_section("homing")
        ok = self.drive.home(
            speed_fast=h["speed_fast"], speed_slow=h["speed_slow"],
            timeout_s=h["timeout"], offset_mm=h["offset_mm"],
            stop_flag=lambda: self._stop_flag, log_cb=self.log,
        )
        if ok:
            self._refresh_absolute_position()
            self.reference_established = True
            self.settings.update_section("absolute_position", {"reference_established": True})
            self.log("Machine Z reference established and saved")
        self.set_state(State.READY if ok else State.ERROR)

    def invalidate_position_reference(self):
        self.reference_established = False
        self.settings.update_section("absolute_position", {"reference_established": False})
        self.log("Machine position reference invalidated - homing required")

    # ── GPIO edge monitor ──────────────────────────────────────────────────

    def _start_gpio_monitor(self):
        if self._gpio_monitor_thread and self._gpio_monitor_thread.is_alive():
            return
        self._gpio_monitor_stop = False
        self._gpio_monitor_thread = threading.Thread(target=self._gpio_monitor_loop, daemon=True)
        self._gpio_monitor_thread.start()

    def _gpio_monitor_loop(self):
        prev_pedal = self.gpio.foot_pedal()
        prev_out4 = self.gpio.intermediate_finish()
        prev_out5 = self.gpio.part_complete()
        last_pedal_edge = 0.0
        while not self._gpio_monitor_stop:
            try:
                pedal = self.gpio.foot_pedal()
                out4 = self.gpio.intermediate_finish()
                out5 = self.gpio.part_complete()
                now = time.monotonic()

                if pedal and not prev_pedal and now - last_pedal_edge > 0.15:
                    last_pedal_edge = now
                    self._on_foot_pedal()
                if out4 and not prev_out4:
                    self._on_out4()
                if out5 and not prev_out5:
                    self._on_out5()

                prev_pedal, prev_out4, prev_out5 = pedal, out4, out5
            except Exception as exc:
                self.log(f"GPIO monitor error: {exc}")
            time.sleep(0.02)

    def _on_foot_pedal(self):
        # Pedal is a valid Mark action ONLY when the manual program is armed at
        # side-1 initial position.  It already starts EzCad2 electrically, so
        # this path must NOT pulse GPIO17 again.
        if not self.manual_mark_enabled or not self.manual_initial_ready:
            self.log("Foot pedal ignored - Laser Program is not ready at initial position")
            return
        self.log("Foot pedal detected on GPIO22 - treating as MARK (no GPIO17 pulse)")
        self.manual_mark(source="pedal")

    # ── Part selection / validation ─────────────────────────────────────────

    def select_manual_part(self, part_name):
        part_name = str(part_name or "").strip()

        # Do not let a dropdown change disturb an active side/motion.
        if (part_name != (self.manual_selected_part_name or "") and
                (self.manual_awaiting_result or self.manual_program_state in {
                    "INITIALIZING", "MOVING_NEXT", "RETURNING_INITIAL", "RESETTING"
                })):
            return False, "Cannot change part while the manual cycle is active"

        if not part_name:
            self._clear_manual_program("NO_PART")
            return True, None

        part = self.parts.load(part_name)
        ok, msg = self.parts.validate(part)
        if not ok:
            self.manual_selected_part_name = part_name
            self.manual_program_started = False
            self.manual_mark_enabled = False
            self.manual_initial_ready = False
            self.manual_program_state = "CONFIG_ERROR"
            self.manual_error = msg
            self.log(f"Part '{part_name}' configuration invalid: {msg}")
            return False, msg

        if part_name != self.manual_selected_part_name:
            self.manual_selected_part_name = part_name
            self.current_part = None
            self.manual_program_started = False
            self.manual_mark_enabled = False
            self.manual_initial_ready = False
            self.manual_stopped = False
            self.manual_side_index = 0
            self.manual_out4_count = 0
            self.manual_pending_next_index = None
            self.manual_pending_return = False
            self.manual_program_state = "PART_SELECTED"
            self.manual_error = None
            self.state = State.PROGRAM_SELECTED
            self.log(f"Part selected: {part_name} - Laser Program Start required")
        return True, None

    def _clear_manual_program(self, program_state="NO_PART"):
        self.manual_selected_part_name = None
        self.current_part = None
        self.manual_program_started = False
        self.manual_stopped = False
        self.manual_mark_enabled = False
        self.manual_initial_ready = False
        self.manual_awaiting_result = False
        self.manual_side_index = 0
        self.manual_out4_count = 0
        self.manual_pending_next_index = None
        self.manual_pending_return = False
        self.manual_program_state = program_state
        self.manual_error = None

    # ── Manual Laser Program Start / Mark ──────────────────────────────────

    def start_manual_program(self, part_name):
        if not self.position_ready():
            return False, "Trusted absolute machine position unavailable - home/check drive first"
        part = self.parts.load(part_name)
        ok, msg = self.parts.validate(part)
        if not ok:
            self.manual_error = msg
            self.manual_program_state = "CONFIG_ERROR"
            return False, msg
        if self.manual_awaiting_result:
            return False, "Laser cycle is already active"

        self.manual_selected_part_name = part_name
        self.current_part = part
        self.total_steps = int(part["sides"])
        self.current_step = 0
        self.manual_program_started = True
        self.manual_stopped = False
        self.manual_mark_enabled = False
        self.manual_initial_ready = False
        self.manual_awaiting_result = False
        self.manual_side_index = 0
        self.manual_out4_count = 0
        self.manual_pending_next_index = None
        self.manual_pending_return = False
        self.manual_program_state = "INITIALIZING"
        self.manual_error = None
        self._manual_motion_stop = False
        self.set_state(State.INITIALIZING)
        threading.Thread(target=self._move_to_initial, daemon=True).start()
        return True, None

    def _move_to_initial(self):
        if not self.current_part:
            return
        target = float(self.current_part["positions"][0]["mm"])
        with self._manual_operation_lock:
            if self.manual_stopped:
                self.manual_program_state = "PROGRAM_STOPPED"
                return
            self.manual_program_state = "INITIALIZING"
            self.set_state(State.MOVING_Z)
            cur = self.drive.get_position_mm()
            ok = True
            if cur is None or abs(cur - target) > 0.05:
                self.log(f"Laser Program Start - moving Z to side 1 initial height {target:.3f} mm")
                ok = self.drive.move_to(
                    target, 300,
                    stop_flag=lambda: self._manual_motion_stop or self.manual_stopped,
                    log_cb=self.log,
                )
            if not ok:
                if self.manual_stopped or self._manual_motion_stop:
                    self.manual_program_state = "PROGRAM_STOPPED"
                    self.set_state(State.PROGRAM_STOPPED)
                else:
                    self.manual_error = "Failed to reach initial side height"
                    self.manual_program_state = "ERROR"
                    self.set_state(State.ERROR)
                return
            self._refresh_absolute_position()
            self.manual_side_index = 0
            self.current_step = 1
            self.manual_initial_ready = True
            self.manual_mark_enabled = not self.manual_stopped
            if self.manual_stopped:
                self.manual_program_state = "PROGRAM_STOPPED"
                self.set_state(State.PROGRAM_STOPPED)
            else:
                self.manual_program_state = "READY_TO_MARK"
                self.set_state(State.READY_TO_MARK)
                self.log("Initial position reached - MARK button and GPIO22 foot pedal are enabled")

    def manual_mark(self, source="gui"):
        if not self.manual_program_started or not self.current_part:
            return False, "Press Laser Program Start first"
        if self.manual_stopped:
            return False, "Laser Program is stopped - press Resume"
        if not self.manual_mark_enabled or not self.manual_initial_ready:
            return False, "MARK is locked until Z is at the initial side position"
        if self.manual_awaiting_result:
            return False, "Already waiting for EzCad2 OUT4/OUT5"

        self.manual_mark_enabled = False  # prevent a double-start while marking
        self.manual_awaiting_result = True
        self.manual_side_index = 0
        self.current_step = 1
        self.manual_out4_count = 0
        self.manual_program_state = "WAITING_SIDE_RESULT"
        self.set_state(State.WAITING_LASER)

        if source == "pedal":
            # IMPORTANT: physical pedal already starts EzCad2. GPIO22 is only
            # acknowledgement to this program so it starts waiting for OUT4/OUT5.
            self.log("Side 1 started by foot pedal - awaiting OUT4/OUT5")
        else:
            pulse_ms = self.settings.get("laser", "pulse_ms") or 100
            self.log(f"Side 1 started by GUI MARK - GPIO17 pulse {pulse_ms} ms")
            threading.Thread(target=self.gpio.pulse_laser_start, args=(pulse_ms,), daemon=True).start()
        return True, None

    def _trigger_side(self, index):
        """Move to side index if necessary, then start it via GPIO17."""
        if not self.current_part:
            return
        positions = self.current_part["positions"]
        if index < 0 or index >= len(positions):
            self._manual_fault(f"Requested side {index + 1} is outside recipe")
            return

        with self._manual_operation_lock:
            if self.manual_stopped:
                self.manual_pending_next_index = index
                self.manual_program_state = "PROGRAM_STOPPED"
                self.set_state(State.PROGRAM_STOPPED)
                return

            self.manual_program_state = "MOVING_NEXT"
            self.manual_mark_enabled = False
            self.manual_initial_ready = False
            self.manual_pending_next_index = index
            prev_index = max(0, index - 1)
            prev_mm = float(positions[prev_index]["mm"])
            target = float(positions[index]["mm"])

            if abs(target - prev_mm) > 0.001:
                self.set_state(State.MOVING_Z)
                self.log(f"Side {index + 1} height differs: {prev_mm:.3f} -> {target:.3f} mm")
                ok = self.drive.move_to(
                    target, 300,
                    stop_flag=lambda: self._manual_motion_stop or self.manual_stopped,
                    log_cb=self.log,
                )
                if not ok:
                    if self.manual_stopped or self._manual_motion_stop:
                        self.manual_program_state = "PROGRAM_STOPPED"
                        self.set_state(State.PROGRAM_STOPPED)
                        return
                    self._manual_fault(f"Failed to move Z for side {index + 1}")
                    return
            else:
                self.log(f"Side {index + 1} uses same Z height {target:.3f} mm - no motor move")

            if self.manual_stopped:
                self.manual_pending_next_index = index
                self.manual_program_state = "PROGRAM_STOPPED"
                self.set_state(State.PROGRAM_STOPPED)
                return

            self.manual_side_index = index
            self.current_step = index + 1
            self.manual_pending_next_index = None
            self.manual_awaiting_result = True
            self.manual_program_state = "WAITING_SIDE_RESULT"
            self.set_state(State.LASER_FIRING)
            pulse_ms = self.settings.get("laser", "pulse_ms") or 100
            self.log(f"Starting side {index + 1}/{len(positions)} - GPIO17 pulse {pulse_ms} ms")
            self.gpio.pulse_laser_start(pulse_ms)
            self.set_state(State.WAITING_LASER)

    # ── OUT4 / OUT5 manual protocol ────────────────────────────────────────

    def _on_out4(self):
        if not self.manual_program_started or not self.current_part:
            self.log("OUT4 ignored - no manual Laser Program started")
            return
        if not self.manual_awaiting_result:
            self.log("OUT4 ignored - program was not awaiting a side result")
            return

        total = int(self.current_part["sides"])
        current = self.manual_side_index
        if current >= total - 1:
            self._manual_fault("Unexpected OUT4 after the configured last side; OUT5 was expected")
            return

        self.manual_awaiting_result = False
        self.manual_out4_count += 1
        next_index = current + 1
        self.manual_side_index = next_index
        self.current_step = next_index + 1
        self.log(
            f"OUT4 intermediate finish received - count {self.manual_out4_count}/{max(total - 1, 0)}; "
            f"next side is {next_index + 1}/{total}"
        )

        if self.manual_stopped:
            self.manual_pending_next_index = next_index
            self.manual_program_state = "PROGRAM_STOPPED"
            self.set_state(State.PROGRAM_STOPPED)
            self.log("Program is stopped - next side stored and will continue on Resume")
            return

        threading.Thread(target=self._trigger_side, args=(next_index,), daemon=True).start()

    def _on_out5(self):
        if not self.manual_program_started or not self.current_part:
            self.log("OUT5 ignored - no manual Laser Program started")
            return
        if not self.manual_awaiting_result:
            self.log("OUT5 ignored - program was not awaiting part completion")
            return

        total = int(self.current_part["sides"])
        if self.manual_side_index != total - 1:
            self._manual_fault(
                f"OUT5 arrived at side {self.manual_side_index + 1}, but recipe has {total} sides"
            )
            return

        self.manual_awaiting_result = False
        self.manual_parts_completed += 1
        self.cycle_count = self.manual_parts_completed
        self.log(f"OUT5 PART COMPLETE - part count {self.manual_parts_completed}")

        # Reset per-part side tracking. The Laser Program remains started for
        # the same selected recipe, exactly as requested.
        self.manual_out4_count = 0
        self.manual_side_index = 0
        self.current_step = 1
        self.manual_mark_enabled = False
        self.manual_initial_ready = False
        self.manual_pending_next_index = None

        if self.manual_stopped:
            self.manual_pending_return = True
            self.manual_program_state = "PROGRAM_STOPPED"
            self.set_state(State.PROGRAM_STOPPED)
            self.log("Part complete while stopped - return to side 1 deferred until Resume")
            return

        self.manual_pending_return = True
        threading.Thread(target=self._return_to_initial_after_part, daemon=True).start()

    def _return_to_initial_after_part(self):
        """Prepare the same recipe for the next physical part without requiring Start again."""
        if not self.current_part:
            return
        target = float(self.current_part["positions"][0]["mm"])
        with self._manual_operation_lock:
            if self.manual_stopped:
                self.manual_pending_return = True
                return
            self.manual_program_state = "RETURNING_INITIAL"
            cur = self.drive.get_position_mm()
            if cur is None or abs(cur - target) > 0.05:
                self.set_state(State.MOVING_Z)
                self.log(f"Preparing next part - returning Z to side 1 height {target:.3f} mm")
                ok = self.drive.move_to(
                    target, 300,
                    stop_flag=lambda: self._manual_motion_stop or self.manual_stopped,
                    log_cb=self.log,
                )
                if not ok:
                    if self.manual_stopped or self._manual_motion_stop:
                        self.manual_pending_return = True
                        self.manual_program_state = "PROGRAM_STOPPED"
                        self.set_state(State.PROGRAM_STOPPED)
                        return
                    self._manual_fault("Failed to return Z to side 1 after part completion")
                    return

            self.manual_pending_return = False
            self.manual_side_index = 0
            self.current_step = 1
            self.manual_initial_ready = True
            self.manual_mark_enabled = True
            self.manual_program_state = "READY_TO_MARK"
            self.set_state(State.READY_TO_MARK)
            self.log("Next part ready - MARK and foot pedal enabled; Laser Program remains started")

    def _manual_fault(self, message):
        self.manual_error = message
        self.manual_mark_enabled = False
        self.manual_initial_ready = False
        self.manual_awaiting_result = False
        self.manual_program_state = "ERROR"
        self.set_state(State.ERROR)
        self.log(f"MANUAL PROGRAM ERROR: {message}")

    # ── Manual Stop / Resume / Reset ───────────────────────────────────────

    def stop_manual_program(self):
        if not self.manual_program_started:
            return False, "Laser Program has not been started"
        self.manual_stopped = True
        self.manual_mark_enabled = False
        self._manual_motion_stop = True
        self.drive.servo_off()
        self.manual_program_state = "PROGRAM_STOPPED"
        self.set_state(State.PROGRAM_STOPPED)
        if self.manual_awaiting_result:
            self.log(
                "Laser Program STOPPED while awaiting EzCad2. Z will not move and no next side will be triggered. "
                "Current EzCad2 marking cannot be aborted because no EzCad2 STOP output is wired."
            )
        else:
            self.log("Laser Program STOPPED - Z remains at its current position")
        return True, None

    def resume_manual_program(self):
        if not self.manual_program_started:
            return False, "Laser Program has not been started"
        if not self.manual_stopped:
            return False, "Laser Program is not stopped"
        self.manual_stopped = False
        self._manual_motion_stop = False
        self.manual_error = None
        self.log("Laser Program RESUME")

        if self.manual_awaiting_result:
            self.manual_program_state = "WAITING_SIDE_RESULT"
            self.set_state(State.WAITING_LASER)
            self.log("Resumed at current side - waiting for the existing EzCad2 OUT4/OUT5 result; not retriggering")
            return True, None

        if self.manual_pending_next_index is not None:
            idx = self.manual_pending_next_index
            threading.Thread(target=self._trigger_side, args=(idx,), daemon=True).start()
            return True, None

        if self.manual_pending_return:
            threading.Thread(target=self._return_to_initial_after_part, daemon=True).start()
            return True, None

        if not self.manual_initial_ready:
            threading.Thread(target=self._move_to_initial, daemon=True).start()
            return True, None

        self.manual_mark_enabled = True
        self.manual_program_state = "READY_TO_MARK"
        self.set_state(State.READY_TO_MARK)
        return True, None

    def reset_manual_program(self, confirm_abort=False):
        if not self.manual_program_started or not self.current_part:
            return False, "Laser Program has not been started"

        # If EzCad2 failed and never returned OUT4/OUT5, recovery is allowed,
        # but only after the operator has first stopped our sequence and then
        # explicitly confirmed that EzCad2 itself is no longer marking.
        if self.manual_awaiting_result:
            if not self.manual_stopped:
                return False, (
                    "EzCad2 result is still outstanding. Press Laser Program STOP first, "
                    "make sure EzCad2 marking has stopped, then press RESET."
                )
            if not confirm_abort:
                return False, (
                    "Reset confirmation required: confirm that EzCad2 marking has stopped "
                    "before the Z axis returns to side 1."
                )
            self.log(
                "FORCED MANUAL RESET - operator confirmed EzCad2 has stopped; "
                "discarding the outstanding OUT4/OUT5 result"
            )

        # Cancel any pending/unfinished side result and rebuild the manual
        # cycle from side 1. Late OUT4/OUT5 edges are ignored because
        # manual_awaiting_result is cleared before the axis is allowed to move.
        self.manual_awaiting_result = False
        self.manual_stopped = False
        self._manual_motion_stop = False
        self.manual_mark_enabled = False
        self.manual_initial_ready = False
        self.manual_side_index = 0
        self.current_step = 1
        self.manual_out4_count = 0
        self.manual_pending_next_index = None
        self.manual_pending_return = False
        self.manual_program_state = "RESETTING"
        self.manual_error = None
        self.set_state(State.RESETTING)
        self.log("Laser Program RESET - side count cleared; moving to side 1 height")
        threading.Thread(target=self._move_to_initial, daemon=True).start()
        return True, None

    # ── Jog ────────────────────────────────────────────────────────────────

    def jog(self, direction, distance_mm, speed_rpm=200):
        if self.manual_program_started:
            self.log("Jog blocked - stop/end the manual Laser Program before service jog")
            return False
        if not self.position_ready():
            self.log("Jog blocked - trusted absolute machine position unavailable")
            return False
        return self.drive.jog(direction, distance_mm, speed_rpm)

    # ── Automatic mode (legacy; not changed for the new manual side protocol) ─

    def start_auto(self, part_name, total_cycles):
        if not self.position_ready():
            self.log("Auto start blocked - trusted absolute machine position unavailable")
            return False
        part = self.parts.load(part_name)
        if not part:
            self.log(f"Part '{part_name}' not found")
            return False
        if self.state != State.READY:
            self.log(f"Cannot start - state is {self.state}")
            return False
        self.current_part = part
        self.total_steps = len(part.get("positions", []))
        self.total_cycles = total_cycles
        self.cycle_count = 0
        self._stop_flag = False
        self._pause_flag = False
        self._thread = threading.Thread(target=self._auto_cycle, daemon=True)
        self._thread.start()
        return True

    def _auto_cycle(self):
        positions = self.current_part["positions"]
        self.log(f"=== AUTO START: {self.current_part['name']} x {self.total_cycles} cycles ===")
        for cycle in range(self.total_cycles):
            if self._stop_flag:
                break
            self.cycle_count = cycle + 1
            self.log(f"--- Cycle {self.cycle_count}/{self.total_cycles} ---")
            while self._pause_flag and not self._stop_flag:
                time.sleep(0.2)
            if self._stop_flag:
                break
            self.set_state(State.WAITING_ROBOT)
            self.log("[OPC UA] Signal robot: place part (placeholder)")
            time.sleep(1.0)
            self.log("[OPC UA] Robot confirmed: part placed")
            for idx, pos_cfg in enumerate(positions):
                if self._stop_flag:
                    break
                self.current_step = idx + 1
                target = pos_cfg["mm"]
                label = pos_cfg.get("label", f"Step {idx + 1}")
                self.set_state(State.MOVING_Z)
                ok = self.drive.move_to(target, 300, stop_flag=lambda: self._stop_flag, log_cb=self.log)
                if not ok:
                    self.set_state(State.ERROR)
                    self._stop_flag = True
                    break
                self.set_state(State.LASER_FIRING)
                self.log(f"Laser trigger -> {label}")
                self.gpio.pulse_laser_start(self.settings.get("laser", "pulse_ms") or 100)
                self.set_state(State.WAITING_LASER)
                ok = self._wait_laser_done()
                if not ok:
                    self.log("Laser error - stopping")
                    self.set_state(State.ERROR)
                    self._stop_flag = True
                    break
                self.log(f"{label} complete")
            if self._stop_flag:
                break
            self.set_state(State.WAITING_ROBOT)
            self.log("[OPC UA] Signal robot: pick part (placeholder)")
            time.sleep(1.0)
            self.log("[OPC UA] Robot confirmed: part picked")
            self.log(f"Cycle {self.cycle_count} complete")
        if self.state != State.ERROR:
            self.set_state(State.READY)
        self.log(f"=== AUTO DONE - {self.cycle_count} cycles completed ===")

    def _wait_laser_done(self, timeout_s=120):
        """Legacy Automatic-mode wait logic. Manual mode uses edge events above."""
        if not GPIO_AVAILABLE:
            time.sleep(2.0)
            return True
        deadline = time.time() + 5
        started = False
        while time.time() < deadline:
            if self._stop_flag:
                return False
            if self.gpio.marking_active():
                started = True
                break
            time.sleep(0.05)
        if not started:
            self.log("Laser did not produce OUT4 within start timeout")
            return False
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            if self._stop_flag:
                return False
            if not self.gpio.marking_active():
                break
            time.sleep(0.05)
        return True

    # ── Legacy/global controls ─────────────────────────────────────────────

    def stop(self):
        # Header emergency/global STOP. Manual program gets the same safe stop
        # behavior; automatic mode uses the legacy global stop flag.
        if self.manual_program_started:
            self.stop_manual_program()
        self._stop_flag = True
        self.drive.servo_off()
        self.log("GLOBAL STOP")

    def pause_resume(self):
        self._pause_flag = not self._pause_flag
        self.log("Paused" if self._pause_flag else "Resumed")
        if self._pause_flag:
            self.set_state(State.PAUSED)

    def set_mode(self, mode):
        self.mode = mode
        self.log(f"Mode -> {mode}")

    def set_auto_advance(self, val):
        self.auto_advance = bool(val)

    def shutdown(self):
        self._gpio_monitor_stop = True
        self._telemetry_stop = True
        self._stop_flag = True
        self._manual_motion_stop = True
        self.drive.servo_off()
        self.drive.disconnect()
        self.gpio.cleanup()
        self.log("Shutdown complete")


# Singleton
controller = LaserController()
