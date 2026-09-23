"""
laser_ctrl.py — Laser marking automation engine
Hardware: Raspberry Pi + 2x A6-RS servo (CN6 USB / Modbus RTU) + GPIO

Axes
  transfer : horizontal, left-right, 5 mm/rev, 0-390 mm, /dev/transfer_axis
             home = NL switch release point (0 mm), no move after homing
  laser    : vertical, up-down, 4 mm/rev, 0-420 mm, /dev/laser_axis
             home = NL switch release point -2.5 mm, then moves 2.5 mm up to 0 mm

Drive layer (per axis)
  - every Modbus frame verified (CRC, echo, exception reply); FC10 reply = 8 bytes
  - comm failure is never treated as "arrived"; unplugged drive is detected and reconnected
  - PP configuration verified/written once when the drive connects
  - servo enable waits for U41.0A = 2 plus C05.13 delay before commanding
  - arrival = position within tolerance AND speed 0 on two consecutive reads
  - servo is ON only while an axis is moving (per-axis HOLD setting)
  - one motion lock per axis: jog / homing / move on the same axis cannot interleave
  - machine reference only trusted across power cycles when C00.07 = absolute mode

Manual laser program (two axes):
  START  -> transfer to loading position, laser to side 1 height
  MARK / foot pedal -> transfer to laser position, verify, pulse GPIO17
  OUT4   -> laser to the next side height, verify, pulse again
  OUT5   -> delay, then back to the loading position
Every program move goes through the group collision rule.
The Automatic cycle is still disabled (AUTO_ENABLED = False).
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

BAUD_RATE      = 115200
DRIVE_ADDR     = 0x01          # both drives: address 1, each on its own USB port

MANUAL_ENABLED   = True        # Manual laser program (two axes)
AUTO_ENABLED     = False       # Automatic cycle - not redesigned for two axes yet
POS_VERIFY_TOL_MM = 0.05       # both axes must be this close before the laser fires
SIGNAL_DEBOUNCE_S = 0.020      # an input must be stable this long before it counts
SIGNAL_LOCKOUT_S  = 0.400      # ignore further edges of the same signal for this long
SPEED_STOPPED_RPM = 5          # |U40.01| below this counts as standing still
VERIFY_RETRY_S    = 2.0        # keep re-checking this long before failing

# Per-axis configuration. Units: command units/mm = C00.02 / lead.
AXIS_CONFIG = {
    "transfer": {
        "name":            "Transfer axis",
        "port":            "/dev/transfer_axis",
        "mm_per_rev":      5.0,
        "pulses_per_rev":  10000,        # expected C00.02
        "max_travel_mm":   390.0,
        "home_offset_mm":  0.0,          # home coordinate at the NL release point
        "home_then_zero":  False,        # no move after homing
        "orientation":     "horizontal",
        "home_side":       "left",       # end of travel where the NL/home switch is
        "hold_after_move": False,        # servo ON only while moving
        "jog_speed_rpm":   200,
    },
    "laser": {
        "name":            "Laser axis",
        "port":            "/dev/laser_axis",
        "mm_per_rev":      4.0,
        "pulses_per_rev":  10000,
        "max_travel_mm":   420.0,
        "home_offset_mm":  -2.5,         # NL release point = -2.5 mm ...
        "home_then_zero":  True,         # ... then move 2.5 mm up to 0 mm
        "orientation":     "vertical",   # 0 mm = bottom, + = up
        "home_side":       "bottom",
        # No brake on this axis. False = servo off after every move (as requested).
        # If the axis sinks when the servo is off, set this to True.
        "hold_after_move": False,
        "jog_speed_rpm":   200,
    },
}
AXES = tuple(AXIS_CONFIG)      # ("transfer", "laser")

# Legacy names used by the (currently disabled) program code and the recipe editor
SERIAL_PORT          = AXIS_CONFIG["transfer"]["port"]
MAX_TRAVEL_MM        = AXIS_CONFIG["transfer"]["max_travel_mm"]
HOME_SIDE            = AXIS_CONFIG["transfer"]["home_side"]
# Laser-axis heights used by part recipes. The minimum is the collision rule's
# laser minimum: with the transfer axis under the laser the head must stay above it.
PART_HEIGHT_MIN_MM   = 200.0
PART_HEIGHT_MAX_MM   = AXIS_CONFIG["laser"]["max_travel_mm"]

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
    "homing_transfer": {"speed_fast": 50, "speed_slow": 10, "timeout": 200},   # rpm, rpm, s
    "homing_laser":    {"speed_fast": 50, "speed_slow": 10, "timeout": 200},
    "laser": {
        "pulse_ms":    100,      # ms — laser trigger pulse duration
    },
    # Saved machine reference per axis. Only trusted at startup when the drive
    # is in absolute mode (C00.07 != 0) AND the reference was made in that mode.
    # Manual laser program
    "program": {
        "loading_mm":        0.0,     # transfer position for loading / unloading
        "laser_mm":          350.0,   # transfer position under the laser
        "out5_delay_s":      1.0,     # pause after OUT5 before returning
        "result_timeout_s":  120,     # max wait for OUT4/OUT5
        "complete_timeout_s": 15,     # max wait for OUT5 after the last side's OUT4
        "transfer_speed_rpm": 300,
        "laser_speed_rpm":    300,
        "speed_override_pct": 100,    # scales both program speeds (10-100 %)
    },
    "reference_transfer": {"established": False, "abs_mode": False},
    "reference_laser":    {"established": False, "abs_mode": False},
    # Group collision rule: while the transfer axis is above transfer_limit_mm,
    # the laser axis may not be below laser_min_mm (manual, automatic and jog).
    "collision": {"enabled": True, "transfer_limit_mm": 200.0, "laser_min_mm": 200.0},
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
    RETURNING_INITIAL = "RETURNING_INITIAL"


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

POS_TOL_MM        = 0.02     # arrival window
BRAKE_CMD_DELAY_S = 0.15     # >= C05.13 (default 100 ms) + margin

# Fixed drive configuration - checked when a drive connects, written only if different.
# (group, offset, value, description)
REQUIRED_CONFIG = [
    (0x00, 0x00, 0,  "C00.00 control mode = position"),
    (0x03, 0x00, 1,  "C03.00 reference = internal position planning"),
    (0x11, 0x00, 3,  "C11.00 planning mode = PP"),
    (0x11, 0x01, 0,  "C11.01 reference type = absolute"),
    (0x11, 0x02, 1,  "C11.02 update = immediate"),
    (0x11, 0x03, 1,  "C11.03 start group = 1"),
    (0x11, 0x04, 1,  "C11.04 end group = 1"),
    (0x04, 0x10, 1,  "C04.10 DI5 = S-ON"),
    (0x04, 0x14, 19, "C04.14 DI6 = position planning trigger"),
    (0x04, 0x38, 9,  "C04.38 DO5 = referencing completion"),
]

# Read-only sanity checks - reported, never written automatically.
EXPECT = [
    (0x0A, 0x0D, 0,  "C0A.0D CN6 storage (1 = every write goes to EEPROM)"),
    (0x05, 0x00, -3, "C05.00 stop mode at S-ON off (-3 = zero-speed stop + dynamic brake)"),
]


class DriveError(Exception):
    pass


class A6Drive:
    """One A6-RS drive on its own USB port. All geometry comes from AXIS_CONFIG."""

    def __init__(self, key, cfg):
        self.key = key
        self.name = cfg["name"]
        self.port = cfg["port"]
        self.mm_per_rev = cfg["mm_per_rev"]
        self.pulses_per_rev = cfg["pulses_per_rev"]
        self.units_per_mm = self.pulses_per_rev / self.mm_per_rev
        self.max_travel_mm = cfg["max_travel_mm"]
        self.home_offset_mm = cfg["home_offset_mm"]
        self.home_then_zero = cfg["home_then_zero"]
        self.hold_default = cfg["hold_after_move"]
        self.ser = None
        self._lock = threading.Lock()          # one frame at a time
        self.motion_lock = threading.RLock()   # one motion at a time on this axis
        self.servo_is_on = False
        self.abs_mode = None                   # C00.07, read in verify_config
        self.config_ok = False                 # verify_config passed
        self.last_ok = 0.0                     # monotonic time of last good frame
        self.last_error = None

    def _p(self, msg):
        print(f"[{self.name}] {msg}")

    # ── link ─────────────────────────────────────────────────────────────
    def connect(self):
        try:
            self.ser = serial.Serial(self.port, BAUD_RATE, bytesize=8,
                                     parity='N', stopbits=1, timeout=0.3)
            return True
        except Exception as e:
            self.ser = None
            self.last_error = f"cannot open {self.port}: {e}"
            return False

    def disconnect(self):
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
        except Exception:
            pass
        self.ser = None
        self.config_ok = False
        self.servo_is_on = False

    def connected(self):
        return self.ser is not None and self.ser.is_open

    def online(self, max_age_s=2.0):
        """Port open, configured, and the drive answered recently."""
        return (self.connected() and self.config_ok
                and time.monotonic() - self.last_ok < max_age_s)

    def _xfer(self, frame, n):
        """Send frame, return validated reply or raise DriveError."""
        if not self.connected():
            raise DriveError("port closed")
        with self._lock:
            try:
                self.ser.reset_input_buffer()
                self.ser.write(frame)
                resp = self.ser.read(n)
                if len(resp) >= 2 and resp[1] & 0x80:
                    resp += self.ser.read(8)       # drain rest of error frame
                    raise DriveError(f"exception reply {resp.hex(' ')} to {frame.hex(' ')}")
            except (serial.SerialException, OSError) as e:
                # USB unplugged / device gone: close so the monitor can reconnect
                self.disconnect()
                raise DriveError(f"serial link lost: {e}")
        if len(resp) != n:
            raise DriveError(f"short reply ({len(resp)}/{n}) to {frame.hex(' ')}")
        if crc(resp[:-2]) != resp[-2:]:
            raise DriveError(f"CRC error {resp.hex(' ')}")
        self.last_ok = time.monotonic()
        return resp

    # ── raw access (raise on failure) ────────────────────────────────────
    def w16(self, grp, off, val):
        f = w16(grp, off, val)
        if self._xfer(f, 8)[:6] != f[:6]:
            raise DriveError(f"echo mismatch C{grp:02X}.{off:02X}")

    def w32(self, grp, off, val):
        f = w32(grp, off, val)
        if self._xfer(f, 8)[:6] != f[:6]:          # FC10 reply = 8 bytes
            raise DriveError(f"echo mismatch C{grp:02X}.{off:02X}")

    def r16(self, grp, off, signed=True):
        resp = self._xfer(r(grp, off, 1), 7)
        v = (resp[3] << 8) | resp[4]
        return v - 65536 if signed and v > 32767 else v

    def r32(self, grp, off):
        resp = self._xfer(r(grp, off, 2), 9)
        lo = (resp[3] << 8) | resp[4]
        hi = (resp[5] << 8) | resp[6]
        v = (hi << 16) | lo
        return v - 0x100000000 if v > 0x7FFFFFFF else v

    # ── tolerant wrappers (return None on failure) ───────────────────────
    def _safe(self, fn, *a):
        try:
            return fn(*a)
        except DriveError as e:
            self.last_error = str(e)
            return None

    def write16(self, grp, off, val):
        try:
            self.w16(grp, off, val); return True
        except DriveError as e:
            self.last_error = str(e); return False
    def read16(self, grp, off):         return self._safe(self.r16, grp, off)
    def read32(self, grp, off):         return self._safe(self.r32, grp, off)
    def get_speed(self):                return self._safe(self.r16, 0x40, 0x01)   # None on failure
    def get_position_raw(self):         return self._safe(self.r32, 0x40, 0x16)
    def get_encoder_position_raw(self): return self._safe(self.r32, 0x40, 0x18)
    def get_state(self):                return self._safe(self.r16, 0x41, 0x0A, False)
    def get_do_bits(self):              return self._safe(self.r16, 0x40, 0x05, False)

    def to_mm(self, units):
        return round(units / self.units_per_mm, 3) if units is not None else None

    def get_position_mm(self):
        return self.to_mm(self.get_position_raw())

    # ── servo enable / disable ───────────────────────────────────────────
    def servo_off(self):
        """Always safe to call (stop button, shutdown, fault)."""
        if not self.connected():
            self.servo_is_on = False
            return False
        try:
            self.w16(0x04, 0x11, 0)
            self.servo_is_on = False
            return True
        except DriveError as e:
            self._p(f"servo_off failed: {e}")
            return False

    def servo_on(self, timeout=1.0):
        """Enable and wait until the drive reports running + C05.13 delay."""
        if self.servo_is_on and self.get_state() == 2:
            return True
        self.w16(0x04, 0x11, 1)
        t_end = time.time() + timeout
        while time.time() < t_end:
            st = self.r16(0x41, 0x0A, False)
            if st == 2:
                time.sleep(BRAKE_CMD_DELAY_S)      # C05.13: no command right after S-ON
                self.servo_is_on = True
                return True
            if st == 3:
                raise DriveError("drive FAULT during enable")
            time.sleep(0.02)
        self.servo_off()
        raise DriveError("servo did not reach running state")

    def clear_fault(self):
        self._safe(self.w16, 0x31, 0x00, 1)
        time.sleep(0.2)

    # ── configuration check (servo OFF) ──────────────────────────────────
    def verify_config(self, log=print):
        """Returns list of warnings. Writes only parameters that differ.
        Raises DriveError if the drive is faulted or C00.02 does not match."""
        warnings = []
        self.config_ok = False
        with self.motion_lock:
            self.servo_off()
            time.sleep(0.1)
            st = self.r16(0x41, 0x0A, False)
            if st == 3:
                raise DriveError("drive in FAULT - clear it before configuring")
            for grp, off, val, desc in REQUIRED_CONFIG:
                cur = self.r16(grp, off, False)
                if cur != val:
                    log(f"Config: {desc} (was {cur}) -> writing")
                    self.w16(grp, off, val)
            for grp, off, val, desc in EXPECT:
                cur = self.r16(grp, off)
                if cur != val:
                    warnings.append(f"{desc}: drive has {cur}")
            ppr = self.r32(0x00, 0x02)                     # C00.02 pulses / rev
            if ppr != self.pulses_per_rev:
                raise DriveError(
                    f"C00.02 = {ppr}, expected {self.pulses_per_rev} "
                    f"({self.units_per_mm:g} units/mm at {self.mm_per_rev} mm/rev) - positions would be wrong")
            log(f"C00.02 = {ppr} pulses/rev, lead {self.mm_per_rev} mm/rev -> {self.units_per_mm:g} units/mm")
            self.abs_mode = self.r16(0x00, 0x07, False)    # C00.07
            log(f"C00.07 = {self.abs_mode} ("
                f"{'incremental - homing required after every power-up' if self.abs_mode == 0 else 'absolute mode'})")
        for w in warnings:
            log(f"WARNING: {w}")
        self.config_ok = True
        return warnings

    # ── motion ───────────────────────────────────────────────────────────
    def move_to(self, target_mm, speed_rpm=300, stop_flag=None, log_cb=None, hold=None):
        log = log_cb or (lambda m: None)
        hold = self.hold_default if hold is None else hold
        if not (0.0 <= target_mm <= self.max_travel_mm):
            log(f"Target {target_mm}mm out of range [0-{self.max_travel_mm:g}]"); return False
        if not self.motion_lock.acquire(blocking=False):
            log("Move rejected - another motion is active on this axis"); return False
        target = int(round(target_mm * self.units_per_mm))
        tol = int(POS_TOL_MM * self.units_per_mm)
        trig_on = False
        pos = None
        try:
            start = self.r32(0x40, 0x16)
            if abs(start - target) <= tol:
                log(f"Already at {target_mm:.2f}mm")
                if hold:
                    self.servo_on()
                return True

            # 1. trajectory (During-operation params - no servo-off needed)
            self.w32(0x11, 0x06, target)
            self.w16(0x11, 0x08, speed_rpm)
            self.w32(0x11, 0x0A, 200)
            self.w32(0x11, 0x0C, 200)

            # 2. enable and wait until really running
            self.servo_on()

            if stop_flag and stop_flag():                  # released before motion began
                log("Move cancelled before start"); self.servo_off(); return False

            # 3. trigger (edge; >= 3 ms width required, 50 ms used)
            log(f"Moving to {target_mm:.2f}mm...")
            self.w16(0x04, 0x15, 1); trig_on = True
            time.sleep(0.05)
            self.w16(0x04, 0x15, 0); trig_on = False

            # 4. wait: in window AND stopped, twice in a row
            dist_mm = abs(target - start) / self.units_per_mm
            mm_s = max(speed_rpm, 1) * self.mm_per_rev / 60.0
            deadline = time.time() + 5 + 2.0 * dist_mm / mm_s   # 2x nominal travel time + 5 s
            ok_reads = 0
            while time.time() < deadline:
                if stop_flag and stop_flag():
                    log("Move stopped"); self.servo_off(); return False
                pos = self.r32(0x40, 0x16)
                spd = self.r16(0x40, 0x01)
                if abs(pos - target) <= tol and spd == 0:
                    ok_reads += 1
                    if ok_reads >= 2:
                        break
                else:
                    ok_reads = 0
                if self.r16(0x41, 0x0A, False) == 3:
                    raise DriveError("drive FAULT during move")
                time.sleep(0.05)
            else:
                log(f"Move timeout - at {self.to_mm(pos)}mm"); self.servo_off(); return False

            log(f"Arrived at {self.to_mm(pos):.3f}mm")
            if not hold:
                self.servo_off()
            return True

        except DriveError as e:
            log(f"DRIVE ERROR: {e}")
            if trig_on:
                self._safe(self.w16, 0x04, 0x15, 0)
            self.servo_off()
            return False
        finally:
            self.motion_lock.release()

    def jog(self, positive, distance_mm, speed_rpm=200, stop_flag=None, log_cb=None):
        """positive=True -> away from home (+mm)."""
        cur = self.get_position_mm()
        if cur is None:
            return False
        target = cur + distance_mm if positive else cur - distance_mm
        target = round(max(0.0, min(self.max_travel_mm, target)), 3)
        return self.move_to(target, speed_rpm, stop_flag=stop_flag, log_cb=log_cb)

    def home(self, speed_fast, speed_slow, timeout_s, stop_flag=None, log_cb=None):
        """Method 17 homing. Per the manual the drive stops just after the NL switch
        releases and uses that stop position as home; C10.0B sets the coordinate
        of that point. If home_then_zero, the axis then moves to 0 mm."""
        log = log_cb or (lambda m: None)
        if not self.motion_lock.acquire(blocking=False):
            log("Homing rejected - another motion is active on this axis"); return False
        try:
            log("=== HOMING START (Method 17 - toward NL switch) ===")
            self.servo_off(); time.sleep(0.1)
            self.w16(0x10, 0x01, 17)
            self.w16(0x10, 0x02, speed_fast)
            self.w16(0x10, 0x03, speed_slow)
            self.w32(0x10, 0x04, 1000)
            self.w32(0x10, 0x06, 1000)
            self.w32(0x10, 0x08, int(timeout_s * 1000))
            self.w32(0x10, 0x0B, int(round(self.home_offset_mm * self.units_per_mm)))
            self.servo_on()
            self.w16(0x10, 0x00, 1)

            deadline = time.time() + 3
            while time.time() < deadline:
                if stop_flag and stop_flag(): break
                if abs(self.r16(0x40, 0x01)) > 2: break
                time.sleep(0.1)
            else:
                log("No motion - check NL switch wiring")
                self.w16(0x10, 0x00, 0); self.servo_off(); return False
            log("Moving toward NL switch...")

            deadline = time.time() + timeout_s
            prev = None; stable = 0; pos = None
            while time.time() < deadline:
                if stop_flag and stop_flag():
                    log("Homing stopped by user")
                    self.w16(0x10, 0x00, 0); self.servo_off(); return False
                do = self.r16(0x40, 0x05, False)
                spd = self.r16(0x40, 0x01)
                pos = self.r32(0x40, 0x16)
                if (do & 0x10) == 0 and spd == 0:
                    stable = stable + 1 if pos == prev else 0
                    prev = pos
                    if stable >= 2: break
                time.sleep(0.2)
            else:
                log(f"Homing timeout after {timeout_s}s")
                self.w16(0x10, 0x00, 0); self.servo_off(); return False

            self.w16(0x10, 0x00, 0)
            log(f"Home found - NL release point = {self.to_mm(pos):.3f}mm")
            if not self.home_then_zero:
                self.servo_off()
        except DriveError as e:
            log(f"DRIVE ERROR during homing: {e}")
            self._safe(self.w16, 0x10, 0x00, 0)
            self.servo_off()
            return False
        finally:
            self.motion_lock.release()

        if self.home_then_zero:
            log(f"Moving {abs(self.home_offset_mm):g} mm to position 0 mm...")
            if not self.move_to(0.0, 100, stop_flag=stop_flag, log_cb=log_cb):
                log("Move to 0 mm after homing FAILED")
                return False
        log("=== HOMING COMPLETE - at 0.000 mm ===")
        return True

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
                # Migrate single-axis settings from earlier versions
                if "homing" in saved and "homing_transfer" not in saved:
                    saved["homing_transfer"] = {k: v for k, v in saved["homing"].items()
                                                if k in ("speed_fast", "speed_slow", "timeout")}
                if "absolute_position" in saved and "reference_transfer" not in saved:
                    old = saved["absolute_position"]
                    saved["reference_transfer"] = {
                        "established": bool(old.get("reference_established")),
                        "abs_mode": bool(old.get("reference_abs_mode")),
                    }
                # Merge with defaults to handle missing keys
                merged = json.loads(json.dumps(DEFAULT_SETTINGS))
                for section, vals in saved.items():
                    if section in merged:
                        merged[section].update(vals)
                    elif isinstance(vals, dict):
                        merged[section] = vals      # keep e.g. "appearance"
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
        """Every part must explicitly define side count and one axis position per side."""
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
            return False, "An axis position is required for every side"
        if len(positions) != sides:
            return False, f"Part configuration incomplete: {sides} sides require {sides} position entries"

        for idx, p in enumerate(positions, start=1):
            if not isinstance(p, dict) or p.get("mm") in (None, ""):
                return False, f"Position for side {idx} is required"
            try:
                mm = float(p.get("mm"))
            except (TypeError, ValueError):
                return False, f"Position for side {idx} is invalid"
            if mm < PART_HEIGHT_MIN_MM:
                return False, (f"Side {idx}: part height must be above {PART_HEIGHT_MIN_MM:g} mm "
                               f"(collision rule) - {mm:g} mm entered")
            if mm > PART_HEIGHT_MAX_MM:
                return False, f"Side {idx}: part height must be at most {PART_HEIGHT_MAX_MM:g} mm"
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
      - Program Start moves to side 1 position and arms MARK / foot pedal.
      - UI MARK pulses GPIO17; foot pedal starts EzCad2 itself and GPIO22 only
        tells this controller that a cycle has started.
      - OUT4 advances to the next side and the Pi triggers that next side.
      - OUT5 completes the part, resets the OUT4 count, and prepares side 1
        again for the next physical part.
    """

    def __init__(self):
        self.drives   = {k: A6Drive(k, AXIS_CONFIG[k]) for k in AXES}
        self.drive    = self.drives["transfer"]   # legacy program code uses the transfer axis
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

        # Per-axis runtime state (filled by startup / telemetry).
        self.ax = {k: {
            "position": None, "raw": None, "speed": None,
            "reference": False,          # homed / trusted reference
            "busy": None,                # None | "homing" | "jog"
            "jog_enabled": False,        # Axis-jog toggle
            "error": None,
            "warnings": [],
            "_stop": False,              # per-axis stop request
            "motion": None,              # (start_mm | None, target_mm) while a move/homing runs
            "jog_id": 0,                 # id of the current hold-to-run jog
            "jog_hb": 0.0,               # last "still holding" heartbeat (monotonic)
            "_next_connect": 0.0,
        } for k in AXES}
        self._axis_lock = threading.Lock()

        # Legacy single-axis fields (mirrors of the transfer axis)
        self.z_position                 = None
        self.z_position_raw             = None
        self.encoder_position_raw       = None
        self.encoder_position_available = False
        self.speed_rpm                  = 0
        self.reference_established      = False
        self._telemetry_stop            = False
        self._telemetry_thread          = None

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
        self.manual_pending            = None    # step to continue after Resume
        self.manual_trigger_source     = None
        self.manual_result_deadline    = 0.0
        self.manual_awaiting_complete  = False   # last side done, OUT5 must follow
        self.manual_program_state      = "NO_PART"
        self.manual_error              = None
        self._manual_motion_stop       = False
        self._manual_operation_lock    = threading.Lock()
        self.signal_counts             = {"OUT4": 0, "OUT5": 0}
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

    def axis_log(self, axis, msg):
        self.log(f"[{AXIS_CONFIG[axis]['name']}] {msg}")

    def _refresh_absolute_position(self, axis="transfer"):
        d = self.drives[axis]
        raw = d.get_position_raw()
        if raw is not None:
            a = self.ax[axis]
            a["raw"] = raw
            a["position"] = d.to_mm(raw)
            if axis == "transfer":
                self.z_position_raw = raw
                self.z_position = a["position"]
                self.encoder_position_available = True
        return raw

    def axis_ready(self, axis):
        """Drive online and a trusted reference exists."""
        return bool(self.ax[axis]["reference"] and self.drives[axis].online())

    def homing_required(self, axis):
        return self.drives[axis].online() and not self.ax[axis]["reference"]

    def position_ready(self):
        # Legacy: the (disabled) programs only use the transfer axis
        return self.axis_ready("transfer")

    def get_status(self):
        # IMPORTANT: no Modbus transactions here - values come from telemetry.
        axes = {}
        for k in AXES:
            d, a, cfg = self.drives[k], self.ax[k], AXIS_CONFIG[k]
            axes[k] = {
                "name": cfg["name"],
                "orientation": cfg["orientation"],
                "home_side": cfg["home_side"],
                "max_travel_mm": cfg["max_travel_mm"],
                "port": cfg["port"],
                "port_open": d.connected(),
                "connected": d.online(),
                "position": a["position"],
                "raw": a["raw"],
                "speed": a["speed"],
                "homed": a["reference"],
                "homing_required": self.homing_required(k),
                "ready": self.axis_ready(k),
                "busy": a["busy"],
                "jog_enabled": a["jog_enabled"],
                "servo_on": d.servo_is_on,
                "abs_mode": d.abs_mode,
                "error": a["error"],
                "warnings": a["warnings"],
            }
        part = self.current_part
        t = axes["transfer"]
        return {
            "state": self.state,
            "mode": self.mode,
            "manual_enabled": MANUAL_ENABLED,
            "auto_enabled": AUTO_ENABLED,
            "programs_enabled": MANUAL_ENABLED,
            "program_config": self.program_config(),
            "manual_pending": str(self.manual_pending) if self.manual_pending else None,
            "axes": axes,
            "homing_suggested": [axes[k]["name"] for k in AXES if axes[k]["homing_required"]],
            "collision": self._collision_status(),

            # Legacy single-axis keys (transfer axis)
            "part": part["name"] if part else self.manual_selected_part_name,
            "step": self.manual_side_index + 1 if self.manual_program_started and part else 0,
            "total_steps": int(part.get("sides", 0)) if part else 0,
            "cycle_count": self.cycle_count,
            "total_cycles": self.total_cycles,
            "z_position": t["position"],
            "z_position_raw": t["raw"],
            "speed_rpm": t["speed"],
            "connected": t["connected"],
            "auto_advance": self.auto_advance,
            "encoder_position_available": t["position"] is not None,
            "reference_established": t["homed"],
            "position_ready": t["ready"],
            "homed": t["homed"],

            # Manual-program status for GUI interlocks.
            "manual_selected_part": self.manual_selected_part_name,
            "manual_program_started": self.manual_program_started,
            "manual_program_state": self.manual_program_state,
            "manual_stopped": self.manual_stopped,
            "manual_mark_enabled": self.manual_mark_enabled,
            "manual_initial_ready": self.manual_initial_ready,
            "manual_awaiting_result": self.manual_awaiting_result,
            "manual_awaiting_complete": self.manual_awaiting_complete,
            "manual_side_index": self.manual_side_index,
            "manual_side_number": self.manual_side_index + 1 if part else 0,
            "manual_total_sides": int(part.get("sides", 0)) if part else 0,
            "manual_out4_count": self.manual_out4_count,
            "manual_parts_completed": self.manual_parts_completed,
            "signal_counts": dict(self.signal_counts),
            "manual_error": self.manual_error,
        }

    # ── Group collision rule ────────────────────────────────────────────────
    #
    # Forbidden state: transfer > transfer_limit_mm AND laser < laser_min_mm.
    # "Worsening" moves (transfer increasing, laser decreasing, any homing from an
    # unknown start) are checked against the other axis' possible positions,
    # including any move it is currently executing. Unknown / unhomed positions
    # are treated as worst case. Moves away from the zone are always allowed.

    def collision_config(self):
        c = self.settings.get_section("collision")
        return (bool(c.get("enabled", True)),
                float(c.get("transfer_limit_mm", 200.0)),
                float(c.get("laser_min_mm", 200.0)))

    def _axis_range(self, axis):
        """(lo, hi) positions the axis may occupy now, or None if unknown."""
        a = self.ax[axis]
        pos = a["position"] if (a["reference"] and self.drives[axis].online()) else None
        m = a["motion"]
        if m:
            if m[0] is None:
                return None
            vals = [m[0], m[1]] + ([pos] if pos is not None else [])
            return (min(vals), max(vals))
        return None if pos is None else (pos, pos)

    def _collision_limit(self, axis):
        """Limit a worsening target on `axis` must respect, or None if unrestricted."""
        enabled, t_lim, l_min = self.collision_config()
        if not enabled:
            return None
        if axis == "laser":
            rng = self._axis_range("transfer")
            return l_min if (rng is None or rng[1] > t_lim) else None
        rng = self._axis_range("laser")
        return t_lim if (rng is None or rng[0] < l_min) else None

    def _begin_motion(self, axis, start_mm, target_mm, clamp=False, what="Move"):
        """Check the collision rule and register the motion. Returns (target, error)."""
        name = AXIS_CONFIG[axis]["name"]
        with self._axis_lock:
            if axis == "laser":
                worsening = start_mm is None or target_mm < start_mm
            else:
                worsening = start_mm is None or target_mm > start_mm
            if worsening:
                lim = self._collision_limit(axis)
                if lim is not None:
                    bad = target_mm < lim if axis == "laser" else target_mm > lim
                    if bad:
                        room = (start_mm is not None and
                                (start_mm > lim + 0.01 if axis == "laser" else start_mm < lim - 0.01))
                        if clamp and room:
                            self.axis_log(axis, f"{what} limited to {lim:g} mm by group collision rule")
                            target_mm = lim
                        else:
                            _, t_lim, l_min = self.collision_config()
                            if axis == "laser":
                                why = (f"laser may not go below {l_min:g} mm while the transfer axis "
                                       f"is (or may be) above {t_lim:g} mm")
                            else:
                                why = (f"transfer may not go above {t_lim:g} mm while the laser axis "
                                       f"is (or may be) below {l_min:g} mm")
                            return None, f"{name}: {what.lower()} blocked by group collision rule - {why}"
            self.ax[axis]["motion"] = (start_mm, target_mm)
            return target_mm, None

    def _collision_status(self):
        enabled, t_lim, l_min = self.collision_config()
        return {
            "enabled": enabled,
            "transfer_limit_mm": t_lim,
            "laser_min_mm": l_min,
            # True when a worsening move on that axis is currently restricted
            "laser_restricted": enabled and self._collision_limit("laser") is not None,
            "transfer_restricted": enabled and self._collision_limit("transfer") is not None,
        }

    def _end_motion(self, axis):
        self.ax[axis]["motion"] = None

    def move_axis(self, axis, target_mm, speed_rpm=300, stop_flag=None, log_cb=None, hold=None):
        """Single entry point for program moves - enforces the collision rule."""
        d = self.drives[axis]
        start = d.get_position_mm() if self.ax[axis]["reference"] else None
        target, err = self._begin_motion(axis, start, float(target_mm))
        if err:
            (log_cb or self.log)(err)
            return False
        try:
            return d.move_to(target, speed_rpm, stop_flag=stop_flag, log_cb=log_cb, hold=hold)
        finally:
            self._end_motion(axis)

    # ── Drive connection / telemetry ────────────────────────────────────────

    def _connect_axis(self, axis, quiet=False):
        """Open port, check configuration, decide whether homing is required."""
        d, a = self.drives[axis], self.ax[axis]
        if not d.connect():
            a["error"] = f"Drive not found on {d.port}"
            if not quiet:
                self.axis_log(axis, f"NOT CONNECTED - {d.last_error}")
            return False
        self.axis_log(axis, f"Connected on {d.port}")
        try:
            a["warnings"] = d.verify_config(lambda m: self.axis_log(axis, m))
        except DriveError as e:
            a["error"] = f"Configuration check failed: {e}"
            self.axis_log(axis, a["error"])
            d.disconnect()
            return False
        a["error"] = None
        self._refresh_absolute_position(axis)
        spd = d.get_speed()
        a["speed"] = spd

        saved = self.settings.get_section(f"reference_{axis}")
        trusted = bool(saved.get("established") and saved.get("abs_mode")
                       and d.abs_mode not in (None, 0))
        a["reference"] = trusted
        if saved.get("established") and not trusted:
            self.settings.update_section(f"reference_{axis}", {"established": False})
        if trusted:
            self.axis_log(axis, f"Absolute mode + saved reference - position {a['position']} mm, homing not required")
        else:
            self.axis_log(axis, "HOMING REQUIRED - Configure > "
                                f"{AXIS_CONFIG[axis]['name']} homing > START HOMING")
        return True

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
            for axis in AXES:
                d, a = self.drives[axis], self.ax[axis]
                if d.connected() and d.config_ok:
                    self._refresh_absolute_position(axis)
                    a["speed"] = d.get_speed()
                    if not d.connected():            # link lost during the reads
                        self._on_axis_lost(axis)
                elif d.ser is not None:                  # was open, port closed underneath
                    d.disconnect()
                    self._on_axis_lost(axis)
                elif not a["busy"] and time.monotonic() >= a["_next_connect"]:
                    # Drive missing or unplugged: retry every 5 s
                    a["_next_connect"] = time.monotonic() + 5.0
                    was_error = a["error"]
                    if self._connect_axis(axis, quiet=bool(was_error)):
                        self.axis_log(axis, "Drive is online")
            self._update_overall_state()
            time.sleep(0.35)

    def _on_axis_lost(self, axis):
        a = self.ax[axis]
        a["reference"] = False                        # position can no longer be trusted
        a["jog_enabled"] = False
        a["position"] = a["raw"] = a["speed"] = None
        a["error"] = "Drive connection lost"
        if self.manual_program_started and self.manual_program_state != "ERROR":
            threading.Thread(target=self._manual_fault, daemon=True,
                             args=(f"{AXIS_CONFIG[axis]['name']} drive connection lost",)).start()
        a["_next_connect"] = time.monotonic() + 2.0
        self.axis_log(axis, "CONNECTION LOST - reference cleared, homing required after reconnect")

    def _update_overall_state(self):
        if self.manual_program_started or self.state in (State.HOMING, State.MOVING_Z):
            return
        online = [k for k in AXES if self.drives[k].online()]
        new = State.READY if online else State.ERROR
        if new != self.state and self.state in (State.IDLE, State.READY, State.ERROR):
            self.set_state(new)

    # ── Startup ─────────────────────────────────────────────────────────────

    def startup(self):
        self.log("=== Laser Controller Starting ===")
        for axis in AXES:
            self._connect_axis(axis)
            self.ax[axis]["_next_connect"] = time.monotonic() + 5.0
        online = [AXIS_CONFIG[k]["name"] for k in AXES if self.drives[k].online()]
        missing = [AXIS_CONFIG[k]["name"] for k in AXES if not self.drives[k].online()]
        need = [AXIS_CONFIG[k]["name"] for k in AXES if self.homing_required(k)]
        self.log(f"Drives online: {', '.join(online) or 'none'}"
                 + (f" | NOT connected: {', '.join(missing)}" if missing else ""))
        if need:
            self.log(f"Homing suggested for: {', '.join(need)}")
        if not AUTO_ENABLED:
            self.log("Automatic cycle is disabled in this version - Manual program is available")
        self.set_state(State.READY if online else State.ERROR)
        self._start_telemetry()
        self._start_gpio_monitor()

    # ── Homing (per axis) ───────────────────────────────────────────────────

    def start_homing(self, axis="transfer"):
        if axis not in AXES:
            return False, "Unknown axis"
        if self.manual_program_started:
            return False, "Cannot home while a Laser Program is started"
        d, a = self.drives[axis], self.ax[axis]
        if not d.online():
            return False, f"{d.name} drive is not connected"
        with self._axis_lock:
            if a["busy"]:
                return False, f"{d.name} is busy ({a['busy']})"
            a["busy"] = "homing"
        # Homing drives toward the NL switch (below 0 mm) from a possibly unknown start
        start = a["position"] if a["reference"] else None
        _, err = self._begin_motion(axis, start, min(0.0, d.home_offset_mm), what="Homing")
        if err:
            a["busy"] = None
            if axis == "laser":
                err += ". Home the transfer axis first and keep it at or below the limit."
            self.axis_log(axis, err)
            return False, err
        a["_stop"] = False
        threading.Thread(target=self._do_home, args=(axis,), daemon=True).start()
        return True, None

    def stop_homing(self, axis="transfer"):
        if axis in AXES:
            self.ax[axis]["_stop"] = True
            self.axis_log(axis, "Homing stop requested")

    def _do_home(self, axis):
        d, a = self.drives[axis], self.ax[axis]
        try:
            h = self.settings.get_section(f"homing_{axis}")
            a["reference"] = False
            ok = d.home(
                speed_fast=int(h["speed_fast"]), speed_slow=int(h["speed_slow"]),
                timeout_s=int(h["timeout"]),
                stop_flag=lambda: a["_stop"],
                log_cb=lambda m: self.axis_log(axis, m),
            )
            self._refresh_absolute_position(axis)
            if ok:
                a["reference"] = True
                a["error"] = None
                abs_ok = d.abs_mode not in (None, 0)
                self.settings.update_section(f"reference_{axis}",
                                             {"established": True, "abs_mode": abs_ok})
                self.axis_log(axis, "Reference established" +
                              (" and saved (absolute mode)" if abs_ok
                               else " - valid until power-off (incremental mode)"))
            else:
                a["error"] = "Homing failed or was stopped - home again"
        finally:
            self._end_motion(axis)
            a["busy"] = None

    def invalidate_position_reference(self, axis="transfer"):
        self.ax[axis]["reference"] = False
        self.settings.update_section(f"reference_{axis}",
                                     {"established": False, "abs_mode": False})
        self.axis_log(axis, "Position reference invalidated - homing required")

    # ── GPIO edge monitor ──────────────────────────────────────────────────

    def _start_gpio_monitor(self):
        if self._gpio_monitor_thread and self._gpio_monitor_thread.is_alive():
            return
        self._gpio_monitor_stop = False
        self._gpio_monitor_thread = threading.Thread(target=self._gpio_monitor_loop, daemon=True)
        self._gpio_monitor_thread.start()

    def _gpio_monitor_loop(self):
        """Polls the EzCad2 outputs with debouncing. A line must be stable for
        SIGNAL_DEBOUNCE_S before an edge counts, and further edges of the same
        signal are ignored for SIGNAL_LOCKOUT_S - contact bounce on a 1 s pulse
        would otherwise be counted as a second OUT4/OUT5."""
        readers = {"OUT4": self.gpio.intermediate_finish,
                   "OUT5": self.gpio.part_complete,
                   "PEDAL": self.gpio.foot_pedal}
        stable = {k: fn() for k, fn in readers.items()}     # debounced level
        raw_prev = dict(stable)
        changed_at = {k: 0.0 for k in readers}
        rise_at = {k: 0.0 for k in readers}
        accepted_at = {k: 0.0 for k in readers}

        while not self._gpio_monitor_stop:
            try:
                now = time.monotonic()
                for name, read in readers.items():
                    raw = read()
                    if raw != raw_prev[name]:
                        raw_prev[name] = raw
                        changed_at[name] = now
                        continue
                    if raw == stable[name] or now - changed_at[name] < SIGNAL_DEBOUNCE_S:
                        continue

                    stable[name] = raw                       # debounced edge
                    if raw:
                        if now - accepted_at[name] < SIGNAL_LOCKOUT_S:
                            self.log(f"SIGNAL {name} edge ignored - bounce within "
                                     f"{SIGNAL_LOCKOUT_S * 1000:.0f} ms of the previous pulse")
                            continue
                        accepted_at[name] = now
                        rise_at[name] = now
                        if name == "PEDAL":
                            self._on_foot_pedal()
                        else:
                            self.signal_counts[name] += 1
                            self.log(f"SIGNAL {name} #{self.signal_counts[name]} - "
                                     f"side {self.manual_side_index + 1}, "
                                     f"awaiting={self.manual_awaiting_result}"
                                     + (", awaiting OUT5" if self.manual_awaiting_complete else ""))
                            (self._on_out4 if name == "OUT4" else self._on_out5)()
                    elif rise_at[name]:
                        width = (now - rise_at[name]) * 1000.0
                        rise_at[name] = 0.0
                        self.log(f"SIGNAL {name} low again after {width:.0f} ms")
                        if width < 100:
                            self.log(f"WARNING: {name} pulse is only {width:.0f} ms - "
                                     f"100 ms or more is safer")

                self._check_result_timeout()
            except Exception as exc:
                self.log(f"GPIO monitor error: {exc}")
            time.sleep(0.005)

    def _on_foot_pedal(self):
        """The pedal is wired to EzCad2 (first trigger) and to GPIO22 (this input).
        It is only a valid MARK when the cell is ready at the loading position."""
        if self.manual_program_state == "READY_TO_MARK" and self.manual_mark_enabled:
            self.log("Foot pedal detected on GPIO22")
            self.manual_mark(source="pedal")
            return
        if self.manual_program_started:
            self._manual_fault(
                "Foot pedal pressed out of sequence - EzCad2 has received an extra trigger. "
                "Stop and restart the EzCad2 program, then RESET here.")
            return
        self.log("Foot pedal ignored - no Laser Program is ready. "
                 "EzCad2 may now be armed: make sure its program is restarted before starting.")

    # ── Part selection / validation ─────────────────────────────────────────

    def select_manual_part(self, part_name):
        part_name = str(part_name or "").strip()

        # Do not let a dropdown change disturb an active side/motion.
        if (part_name != (self.manual_selected_part_name or "") and
                (self.manual_awaiting_result or self.manual_program_state in {
                    "MOVING_TO_LOAD", "MOVING_TO_LASER", "MOVING_NEXT_SIDE",
                    "RETURNING_LOAD", "RESETTING"
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
            self.manual_pending = None
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
        self.manual_pending = None
        self.manual_program_state = program_state
        self.manual_error = None

    # ── Manual Laser Program (two axes) ─────────────────────────────────────
    #
    # Cycle:
    #   START            transfer -> loading position, laser -> side 1 height
    #   READY_TO_MARK    operator loads the part, presses MARK or the pedal
    #                    (MARK sends the EzCad2 arming pulse; the pedal is
    #                     wired to EzCad2 and arms it itself)
    #   MOVING_TO_LASER  transfer -> laser position (collision rule applies)
    #                    positions verified, then GPIO17 fires the mark pulse
    #   WAITING_RESULT   EzCad2 OUT4 -> laser to next side height, pulse again
    #                    EzCad2 OUT5 -> delay, then back to the loading position

    def program_config(self):
        p = self.settings.get_section("program")
        return {
            "loading_mm":      float(p.get("loading_mm", 0.0)),
            "laser_mm":        float(p.get("laser_mm", 350.0)),
            "out5_delay_s":    float(p.get("out5_delay_s", 1.0)),
            "result_timeout_s": float(p.get("result_timeout_s", 120)),
            "complete_timeout_s": float(p.get("complete_timeout_s", 15)),
            "transfer_rpm":    int(p.get("transfer_speed_rpm", 300)),
            "laser_rpm":       int(p.get("laser_speed_rpm", 300)),
            "override_pct":    int(p.get("speed_override_pct", 100)),
        }

    def _prog_stop_requested(self):
        return self._manual_motion_stop or self.manual_stopped

    def _prog_move(self, axis, target_mm, speed_rpm, what):
        """Program move. Returns 'ok' | 'stopped' | 'failed'.
        The speed override scales every program move; jog and homing are not affected."""
        pct = max(1, min(100, self.program_config()["override_pct"]))
        speed_rpm = max(1, int(round(speed_rpm * pct / 100.0)))
        self.set_state(State.MOVING_Z)
        self.log(f"{what}: {AXIS_CONFIG[axis]['name']} -> {target_mm:.3f} mm "
                 f"at {speed_rpm} rpm" + (f" ({pct} % override)" if pct != 100 else ""))
        ok = self.move_axis(axis, target_mm, speed_rpm,
                            stop_flag=self._prog_stop_requested, log_cb=self.log)
        if ok:
            return "ok"
        return "stopped" if self._prog_stop_requested() else "failed"

    def _verify_positions(self, transfer_mm, laser_mm):
        """Before the laser fires, both axes must be: servo OFF (U41.0A = 1, so the
        drive cannot be executing anything), standing still, and within tolerance
        of the expected position. Retried for VERIFY_RETRY_S."""
        deadline = time.time() + VERIFY_RETRY_S
        while True:
            why = None
            for axis, want in (("transfer", transfer_mm), ("laser", laser_mm)):
                d = self.drives[axis]
                name = AXIS_CONFIG[axis]["name"]
                if not d.online():
                    return False, f"{name} drive is not connected"
                st = d.get_state()                 # 0 not ready, 1 ready (servo off), 2 running, 3 fault
                pos = d.get_position_mm()
                spd = d.get_speed()
                if pos is None or spd is None or st is None:
                    why = f"{name}: could not read the drive"
                elif st == 3:
                    return False, f"{name} drive is in FAULT"
                elif st == 2:
                    why = f"{name} servo is still ON"
                elif abs(spd) > SPEED_STOPPED_RPM:
                    why = f"{name} is still moving ({spd} rpm)"
                elif abs(pos - want) > POS_VERIFY_TOL_MM:
                    why = (f"{name} is at {pos:.3f} mm, expected {want:.3f} mm "
                           f"(tolerance {POS_VERIFY_TOL_MM:g} mm)")
                if why:
                    break
            if why is None:
                return True, None
            if time.time() >= deadline or self._prog_stop_requested():
                return False, why
            time.sleep(0.1)

    def _fire_laser(self, side_index, transfer_mm, laser_mm):
        """Verify both axes, pulse GPIO17 and start waiting for OUT4/OUT5."""
        for axis in AXES:                      # no servo may be energised while marking
            self.drives[axis].servo_off()
        ok, why = self._verify_positions(transfer_mm, laser_mm)
        if not ok:
            self._manual_fault(f"Laser pulse blocked - {why}")
            return False
        pulse_ms = self.settings.get("laser", "pulse_ms") or 100
        self.manual_side_index = side_index
        self.current_step = side_index + 1
        self.manual_awaiting_result = True
        self.manual_result_deadline = time.monotonic() + self.program_config()["result_timeout_s"]
        self.manual_program_state = "WAITING_SIDE_RESULT"
        self.set_state(State.LASER_FIRING)
        total = int(self.current_part["sides"])
        self.log(f"Side {side_index + 1}/{total}: GPIO17 pulse {pulse_ms} ms - waiting for OUT4/OUT5")
        self.gpio.pulse_laser_start(pulse_ms)
        self.set_state(State.WAITING_LASER)
        return True

    def _side_mm(self, index):
        return float(self.current_part["positions"][index]["mm"])

    # ── Start ───────────────────────────────────────────────────────────────

    def start_manual_program(self, part_name):
        if not MANUAL_ENABLED:
            return False, "Manual program is disabled"
        ok, err = self.select_manual_part(part_name)
        if not ok:
            return False, err
        part = self.parts.load(self.manual_selected_part_name)
        ok, msg = self.parts.validate(part)
        if not ok:
            self.manual_program_state = "CONFIG_ERROR"
            self.manual_error = msg
            return False, msg
        for axis in AXES:
            if not self.drives[axis].online():
                return False, f"{AXIS_CONFIG[axis]['name']} drive is not connected"
            if not self.ax[axis]["reference"]:
                return False, f"{AXIS_CONFIG[axis]['name']} is not homed - home it in Configure"
            if self.ax[axis]["busy"]:
                return False, f"{AXIS_CONFIG[axis]['name']} is busy ({self.ax[axis]['busy']})"
        if self.manual_program_started:
            return False, "Laser Program is already started"

        self.current_part = part
        self.total_steps = int(part["sides"])
        self.manual_program_started = True
        self.manual_stopped = False
        self._manual_motion_stop = False
        self.manual_mark_enabled = False
        self.manual_initial_ready = False
        self.manual_awaiting_result = False
        self.manual_awaiting_complete = False
        self.manual_side_index = 0
        self.manual_out4_count = 0
        self.manual_pending = None
        self.manual_error = None
        for axis in AXES:
            self.ax[axis]["jog_enabled"] = False
        self.log(f"=== LASER PROGRAM START - part '{part['name']}', {self.total_steps} side(s) ===")
        threading.Thread(target=self._go_to_loading, args=(True,), daemon=True).start()
        return True, None

    def _go_to_loading(self, first=False):
        """Transfer to the loading position, then laser to the side 1 height."""
        cfg = self.program_config()
        with self._manual_operation_lock:
            if self._prog_stop_requested():
                return self._park_stopped("to_loading")
            self.manual_program_state = "MOVING_TO_LOAD"
            self.manual_mark_enabled = False
            self.manual_initial_ready = False
            # Transfer first: moving toward 0 mm always leaves the collision zone,
            # so the laser can then go to any height.
            r = self._prog_move("transfer", cfg["loading_mm"], cfg["transfer_rpm"],
                                "Loading position")
            if r != "ok":
                return self._after_move(r, "to_loading", "Failed to move the transfer axis to the loading position")
            r = self._prog_move("laser", self._side_mm(0), cfg["laser_rpm"], "Side 1 height")
            if r != "ok":
                return self._after_move(r, "to_loading", "Failed to move the laser axis to the side 1 height")

            self.manual_side_index = 0
            self.current_step = 1
            self.manual_out4_count = 0
            self.manual_pending = None
            self.manual_mark_enabled = True
            self.manual_initial_ready = True
            self.manual_program_state = "READY_TO_MARK"
            self.set_state(State.READY)
            self.log("At loading position - load the part, then press MARK or the foot pedal")

    def _after_move(self, result, pending, fail_msg):
        if result == "stopped":
            return self._park_stopped(pending)
        self._manual_fault(fail_msg)

    def _park_stopped(self, pending):
        self.manual_pending = pending
        self.manual_mark_enabled = False
        self.manual_initial_ready = False
        self.manual_program_state = "PROGRAM_STOPPED"
        self.set_state(State.PROGRAM_STOPPED)

    # ── Mark / pedal ────────────────────────────────────────────────────────

    def manual_mark(self, source="gui"):
        if not MANUAL_ENABLED:
            return False, "Manual program is disabled"
        if not self.manual_program_started or not self.current_part:
            return False, "Press Laser Program Start first"
        if self.manual_stopped:
            return False, "Laser Program is stopped - press Resume"
        if self.manual_program_state != "READY_TO_MARK" or not self.manual_mark_enabled:
            return False, "MARK is locked until the cell is ready at the loading position"
        if self.manual_awaiting_result:
            return False, "Already waiting for EzCad2 OUT4/OUT5"

        self.manual_mark_enabled = False
        self.manual_initial_ready = False
        self.manual_trigger_source = source
        if source == "pedal":
            # The pedal is wired to EzCad2 and has already given it the first
            # (arming) trigger; the Pi must not repeat it.
            self.log("Foot pedal accepted - EzCad2 armed by the pedal")
        else:
            pulse_ms = self.settings.get("laser", "pulse_ms") or 100
            self.log(f"MARK accepted - EzCad2 arming pulse on GPIO17 ({pulse_ms} ms)")
            self.gpio.pulse_laser_start(pulse_ms)
        threading.Thread(target=self._go_to_laser, daemon=True).start()
        return True, None

    def _go_to_laser(self):
        cfg = self.program_config()
        with self._manual_operation_lock:
            if self._prog_stop_requested():
                return self._park_stopped("to_laser")
            self.manual_program_state = "MOVING_TO_LASER"
            r = self._prog_move("transfer", cfg["laser_mm"], cfg["transfer_rpm"], "Laser position")
            if r != "ok":
                return self._after_move(r, "to_laser", "Failed to move the transfer axis to the laser position")
            if self._prog_stop_requested():
                return self._park_stopped("fire")
            self._fire_laser(0, cfg["laser_mm"], self._side_mm(0))

    # ── OUT4 / OUT5 ─────────────────────────────────────────────────────────

    def _out_of_step(self, sig):
        """An OUT4/OUT5 while the program is not waiting for a result means EzCad2
        is running ahead of the controller - stop before anything is mis-marked."""
        self._manual_fault(
            f"{sig} arrived while the controller was not waiting for a result - "
            "EzCad2 is out of step (it was not at 'Loading Started' when the cycle began). "
            "Stop and restart the EzCad2 program, then RESET here.")

    def _on_out4(self):
        if not self.manual_program_started or not self.current_part:
            self.log("OUT4 ignored - no Laser Program started")
            return
        if self.manual_awaiting_complete:
            self._manual_fault("OUT4 arrived while waiting for the OUT5 that completes the part - "
                               "EzCad2 is out of step")
            return
        if not self.manual_awaiting_result:
            if self.manual_program_state != "ERROR":
                self._out_of_step("OUT4")
            return
        total = int(self.current_part["sides"])
        current = self.manual_side_index
        self.manual_awaiting_result = False
        self.manual_out4_count += 1
        if current >= total - 1:
            # Last side: the part counts as complete only on OUT4 followed by OUT5
            cfg = self.program_config()
            self.manual_awaiting_complete = True
            self.manual_result_deadline = time.monotonic() + cfg["complete_timeout_s"]
            self.manual_program_state = "WAITING_PART_COMPLETE"
            self.log(f"OUT4: last side {current + 1}/{total} finished - waiting for OUT5 to confirm the part")
            return
        self.log(f"OUT4: side {current + 1}/{total} finished")
        threading.Thread(target=self._next_side, args=(current + 1,), daemon=True).start()

    def _next_side(self, index):
        cfg = self.program_config()
        with self._manual_operation_lock:
            if self._prog_stop_requested():
                return self._park_stopped(("side", index))
            self.manual_program_state = "MOVING_NEXT_SIDE"
            r = self._prog_move("laser", self._side_mm(index), cfg["laser_rpm"],
                                f"Side {index + 1} height")
            if r != "ok":
                return self._after_move(r, ("side", index),
                                        f"Failed to move the laser axis for side {index + 1}")
            if self._prog_stop_requested():
                return self._park_stopped(("fire", index))
            self._fire_laser(index, cfg["laser_mm"], self._side_mm(index))

    def _on_out5(self):
        if not self.manual_program_started or not self.current_part:
            self.log("OUT5 ignored - no Laser Program started")
            return
        total = int(self.current_part["sides"])
        current = self.manual_side_index

        if self.manual_awaiting_complete:
            # Expected: OUT4 (last side done) followed by OUT5 (part complete)
            self.manual_awaiting_complete = False
            self.manual_result_deadline = 0.0
            self.manual_parts_completed += 1
            self.cycle_count += 1
            self.log(f"OUT5: OUT4+OUT5 sequence complete - part finished "
                     f"({self.manual_parts_completed} total)")
            threading.Thread(target=self._after_part, daemon=True).start()
            return

        if not self.manual_awaiting_result:
            # A short OUT5 right at EzCad2 program start is a reset glitch, not a result
            if self.manual_program_state in ("READY_TO_MARK", "ERROR", "PROGRAM_STOPPED"):
                self.log("OUT5 ignored - no side is running")
                return
            self._out_of_step("OUT5")
            return

        self.manual_awaiting_result = False
        if current != total - 1:
            self._manual_fault(f"OUT5 received on side {current + 1}/{total} - OUT4 expected")
        else:
            self._manual_fault("OUT5 received on the last side without its OUT4 first - "
                               "a part counts as complete only after OUT4 then OUT5")

    def _after_part(self):
        delay = self.program_config()["out5_delay_s"]
        self.manual_program_state = "RETURNING_LOAD"
        self.set_state(State.RETURNING_INITIAL)
        if delay > 0:
            self.log(f"Waiting {delay:g} s before returning to the loading position")
            end = time.monotonic() + delay
            while time.monotonic() < end:
                if self._prog_stop_requested():
                    return self._park_stopped("to_loading")
                time.sleep(0.05)
        self._go_to_loading()

    # ── Result timeout watchdog ─────────────────────────────────────────────

    def _check_result_timeout(self):
        if not self.manual_result_deadline:
            return
        if not (self.manual_awaiting_result or self.manual_awaiting_complete):
            return
        if time.monotonic() > self.manual_result_deadline:
            waiting_complete = self.manual_awaiting_complete
            self.manual_awaiting_result = False
            self.manual_awaiting_complete = False
            self.manual_result_deadline = 0.0
            cfg = self.program_config()
            if waiting_complete:
                self._manual_fault(f"Last side reported OUT4 but no OUT5 within "
                                   f"{cfg['complete_timeout_s']:g} s - part not confirmed complete")
            else:
                self._manual_fault(f"No OUT4/OUT5 from EzCad2 within {cfg['result_timeout_s']:g} s - "
                                   f"check the EzCad2 program")

    def _manual_fault(self, message):
        self._manual_motion_stop = True
        self.manual_awaiting_result = False
        self.manual_awaiting_complete = False
        self.manual_mark_enabled = False
        self.manual_initial_ready = False
        self.manual_program_state = "ERROR"
        self.manual_error = message
        for axis in AXES:
            self.drives[axis].servo_off()
        self.log(f"MANUAL PROGRAM ERROR: {message}")
        self.set_state(State.ERROR)

    # ── Stop / Resume / Reset ───────────────────────────────────────────────

    def stop_manual_program(self):
        if not self.manual_program_started:
            return False, "No Laser Program is started"
        self.manual_stopped = True
        self._manual_motion_stop = True
        for axis in AXES:
            self.ax[axis]["_stop"] = True
            self.drives[axis].servo_off()
        self.manual_mark_enabled = False
        self.manual_initial_ready = False
        if self.manual_program_state not in ("ERROR",):
            self.manual_program_state = "PROGRAM_STOPPED"
            self.set_state(State.PROGRAM_STOPPED)
        self.log("Laser Program STOPPED - axes stopped where they are"
                 + (" (EzCad2 keeps marking the current side)" if self.manual_awaiting_result else ""))
        return True, None

    def resume_manual_program(self):
        if not self.manual_program_started:
            return False, "No Laser Program is started"
        if self.manual_program_state == "ERROR":
            return False, "Clear the error with RESET first"
        if not self.manual_stopped:
            return False, "Laser Program is not stopped"
        for axis in AXES:
            if not self.drives[axis].online():
                return False, f"{AXIS_CONFIG[axis]['name']} drive is not connected"
            if not self.ax[axis]["reference"]:
                return False, f"{AXIS_CONFIG[axis]['name']} lost its reference - home it again"
            self.ax[axis]["_stop"] = False
        self.manual_stopped = False
        self._manual_motion_stop = False
        pending = self.manual_pending
        self.manual_pending = None
        self.log(f"Laser Program RESUME ({pending or 'waiting'})")

        if self.manual_awaiting_result:
            self.manual_program_state = "WAITING_SIDE_RESULT"
            self.set_state(State.WAITING_LASER)
            self.log("Still waiting for EzCad2 OUT4/OUT5 - no new pulse is sent")
            return True, None
        if pending == "to_laser":
            threading.Thread(target=self._go_to_laser, daemon=True).start()
        elif pending == "fire":
            cfg = self.program_config()
            threading.Thread(target=lambda: self._fire_laser(0, cfg["laser_mm"], self._side_mm(0)),
                             daemon=True).start()
        elif isinstance(pending, tuple) and pending[0] == "side":
            threading.Thread(target=self._next_side, args=(pending[1],), daemon=True).start()
        elif isinstance(pending, tuple) and pending[0] == "fire":
            cfg = self.program_config()
            idx = pending[1]
            threading.Thread(target=lambda: self._fire_laser(idx, cfg["laser_mm"], self._side_mm(idx)),
                             daemon=True).start()
        else:
            threading.Thread(target=self._go_to_loading, daemon=True).start()
        return True, None

    def end_manual_program(self):
        """End the run: stop both axes and release the program, keeping the part
        selected so LASER PROGRAM START can be pressed again."""
        if not self.manual_program_started:
            return False, "No Laser Program is started"
        self._manual_motion_stop = True
        for axis in AXES:
            self.ax[axis]["_stop"] = True
            self.drives[axis].servo_off()
        time.sleep(0.2)
        self.manual_program_started = False
        self.manual_stopped = False
        self._manual_motion_stop = False
        self.manual_awaiting_result = False
        self.manual_awaiting_complete = False
        self.manual_result_deadline = 0.0
        self.manual_mark_enabled = False
        self.manual_initial_ready = False
        self.manual_side_index = 0
        self.manual_out4_count = 0
        self.manual_pending = None
        self.manual_error = None
        self.current_part = None
        self.manual_program_state = "PART_SELECTED" if self.manual_selected_part_name else "NO_PART"
        self.set_state(State.PROGRAM_SELECTED if self.manual_selected_part_name else State.READY)
        self.log("Laser Program released - jog and homing are available again")
        return True, None

    def reset_manual_program(self, confirm_abort=False):
        """RESET ends the run: both axes stop, the program is released and
        LASER PROGRAM START becomes available again. The part stays selected;
        START then moves the axes back to the loading position."""
        if not self.manual_program_started:
            return False, "No Laser Program is started"
        if self.manual_awaiting_result and not self.manual_stopped:
            return False, "Press STOP first - EzCad2 may still be marking"
        if (self.manual_awaiting_result or self.manual_awaiting_complete) and not confirm_abort:
            return False, ("confirm_ezcad2: EzCad2 may still be marking or armed. "
                           "Stop and restart the EzCad2 program, then confirm.")
        ok, err = self.end_manual_program()
        if ok:
            self.log("Laser Program RESET - press LASER PROGRAM START to begin again")
        return ok, err

    # ── Axis jog (per axis, toggle-enabled) ────────────────────────────────

    JOG_DIRECTIONS = {
        # direction -> positive (+mm, away from home)?
        "transfer": {"left": None, "right": None, "plus": True, "minus": False},
        "laser":    {"up": True, "down": False, "plus": True, "minus": False},
    }

    def set_jog_enabled(self, axis, on):
        if axis not in AXES:
            return False, "Unknown axis"
        a = self.ax[axis]
        if on:
            if not self.drives[axis].online():
                return False, f"{AXIS_CONFIG[axis]['name']} drive is not connected"
            a["jog_enabled"] = True
            self.axis_log(axis, "Jog ENABLED")
        else:
            a["jog_enabled"] = False
            if a["busy"] == "jog":
                a["_stop"] = True                     # stops the running jog; servo goes off
            self.axis_log(axis, "Jog DISABLED")
        return True, None

    def stop_axis(self, axis):
        """Stop only this axis (jog or homing). The jog toggle stays as it is."""
        if axis not in AXES:
            return False, "Unknown axis"
        a = self.ax[axis]
        a["_stop"] = True
        if not a["busy"]:
            self.drives[axis].servo_off()
        self.axis_log(axis, "STOP")
        return True, None

    JOG_DEADMAN_S = 0.5      # no heartbeat for this long -> treated as released

    def jog_hold(self, axis, jog_id):
        """Heartbeat from the browser while the arrow is still held."""
        a = self.ax.get(axis)
        if a and a["busy"] == "jog" and a["jog_id"] == jog_id:
            a["jog_hb"] = time.monotonic()
            return True
        return False

    def jog_release(self, axis, jog_id=None):
        """Arrow released: stop this jog (the toggle stays ON)."""
        a = self.ax.get(axis)
        if a and a["busy"] == "jog" and (jog_id is None or a["jog_id"] == jog_id):
            a["_stop"] = True
        return True

    def jog(self, axis, direction, distance_mm):
        """Start a hold-to-run jog of at most distance_mm.
        Returns (ok, error, jog_id). The move stops on release, on a missing
        heartbeat (dead-man), at the entered distance, or at any limit."""
        ok, err = self._jog_start(axis, direction, distance_mm)
        return ok, err, (self.ax[axis]["jog_id"] if ok else None)

    def _jog_start(self, axis, direction, distance_mm):
        if axis not in AXES:
            return False, "Unknown axis"
        cfg, d, a = AXIS_CONFIG[axis], self.drives[axis], self.ax[axis]
        if direction not in self.JOG_DIRECTIONS[axis]:
            return False, f"Invalid direction '{direction}' for {cfg['name']}"
        try:
            distance_mm = float(distance_mm)
        except (TypeError, ValueError):
            return False, "Invalid jog distance"
        if not (0.0 < distance_mm <= cfg["max_travel_mm"]):
            return False, f"Jog distance must be 0.1-{cfg['max_travel_mm']:g} mm"
        if self.manual_program_started:
            return False, "Jog is locked while a Laser Program is started"
        if not a["jog_enabled"]:
            return False, f"Switch {cfg['name']} ON first"
        if not d.online():
            return False, f"{cfg['name']} drive is not connected"
        if not a["reference"]:
            return False, f"{cfg['name']} is not homed - home it first"

        positive = self.JOG_DIRECTIONS[axis][direction]
        if positive is None:                          # transfer left/right
            positive = (direction == "right") == (cfg["home_side"] == "left")

        with self._axis_lock:
            if a["busy"]:
                return False, f"{cfg['name']} is busy ({a['busy']})"
            a["busy"] = "jog"

        cur = d.get_position_mm()
        if cur is None:
            a["busy"] = None
            return False, f"{cfg['name']}: position could not be read"
        target = cur + distance_mm if positive else cur - distance_mm
        target = round(max(0.0, min(cfg["max_travel_mm"], target)), 3)
        target, err = self._begin_motion(axis, cur, target, clamp=True, what="Jog")
        if err:
            a["busy"] = None
            self.axis_log(axis, err)
            return False, err
        a["_stop"] = False
        a["jog_id"] += 1
        a["jog_hb"] = time.monotonic()

        def released():
            if a["_stop"] or not a["jog_enabled"]:
                return True
            if time.monotonic() - a["jog_hb"] > self.JOG_DEADMAN_S:
                self.axis_log(axis, "Jog heartbeat lost - stopping (dead-man)")
                return True
            return False

        def run():
            try:
                self.axis_log(axis, f"Jog {direction} (hold) up to {target:.3f} mm")
                quiet = ("Moving to", "Arrived", "Move stopped", "Move cancelled", "Already at")
                ok = d.move_to(target, cfg["jog_speed_rpm"], stop_flag=released,
                               log_cb=lambda m: None if m.startswith(quiet) else self.axis_log(axis, m))
                pos = d.get_position_mm()
                if ok:
                    self.axis_log(axis, f"Jog reached {target:.3f} mm - stopped")
                elif d.connected():
                    self.axis_log(axis, f"Jog stopped at {pos if pos is not None else '--'} mm")
                else:
                    self._on_axis_lost(axis)
            finally:
                self._end_motion(axis)
                a["busy"] = None
        threading.Thread(target=run, daemon=True).start()
        return True, None

    # ── Automatic mode (legacy; not changed for the new manual side protocol) ─

    def start_auto(self, part_name, total_cycles):
        if not AUTO_ENABLED:
            self.log("Automatic cycle is disabled until the two-axis program logic is defined")
            return False
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
                ok = self.move_axis("transfer", target, 300, stop_flag=lambda: self._stop_flag, log_cb=self.log)
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
        self.drive.servo_off()              # release the axis at end of auto run
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

    # ── Global controls ────────────────────────────────────────────────────

    def stop(self):
        # Header EMERGENCY STOP: stop every axis, switch servos off, lock jog.
        if self.manual_program_started:
            self.stop_manual_program()
        self._stop_flag = True
        for axis in AXES:
            self.ax[axis]["_stop"] = True
            self.drives[axis].servo_off()
        self.log("EMERGENCY STOP - all axes stopped, servos off")

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
        for axis in AXES:
            self.ax[axis]["_stop"] = True
            self.drives[axis].servo_off()
            self.drives[axis].disconnect()
        self.gpio.cleanup()
        self.log("Shutdown complete")


# Singleton
controller = LaserController()