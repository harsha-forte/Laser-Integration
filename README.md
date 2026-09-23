# Laser Cell Console

Raspberry Pi control for a fiber laser marking cell with two A6-RS servo axes and an
EzCad2 marking program. A Flask web UI on port 5000 provides operator control from any
browser on the network.

- `laser_ctrl.py` — hardware engine: Modbus drive layer, GPIO, collision rule, manual program
- `laser_app.py` — Flask web UI and JSON API (the whole page is embedded in this file)
- `99-a6rs.rules` — udev rules giving each drive a fixed device name
- `servo_off.py` — standalone utility to disable a servo from the command line
- `parts/` — part recipes (JSON, one per part)
- `settings.json` — machine-local settings, written by the UI

## Hardware

| Item | Detail |
|---|---|
| Controller | Raspberry Pi (Raspberry Pi OS), Python 3 + Flask |
| Drives | 2 × A6-RS servo, Modbus RTU over USB-C (CN6), 115200 8N1, slave address 1 |
| Transfer axis | horizontal, 5 mm/rev, 0–390 mm, `/dev/transfer_axis` |
| Laser axis | vertical, 4 mm/rev, 0–420 mm, `/dev/laser_axis` |
| Marking | EzCad2, triggered and monitored over GPIO |

### GPIO (BCM numbering)

| Pin | Direction | Function |
|---|---|---|
| 17 | out | Mark trigger → EzCad2 GIN15 |
| 27 | out | Remark (reserved) |
| 22 | in | Foot pedal acknowledgement |
| 26 | in | EzCad2 OUT4 — one side finished |
| 4 | in | EzCad2 OUT5 — part complete |

### Required drive parameters

The app checks these at startup and writes the PP setup itself. These must be set once
per drive, by hand or in the commissioning software:

| Parameter | Value | Why |
|---|---|---|
| C00.02 | 10000 | pulses/rev; the app refuses to run if it differs |
| C00.07 | 1 | absolute mode, so the reference survives a power cycle |
| C05.00 | -3 | zero-speed stop + dynamic brake when the servo switches off |
| C0A.0D | 0 | CN6 storage off, so runtime writes don't wear the EEPROM |

With C0A.0D = 0, set the other three **before** switching storage off, otherwise they
are lost at the next power cycle.

## Install

```bash
sudo apt install python3-venv
git clone <repo-url> Laser_Project && cd Laser_Project
python3 -m venv venv
venv/bin/pip install flask pyserial crcmod RPi.GPIO
```

Fixed device names for the two drives:

```bash
sudo cp 99-a6rs.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
ls -l /dev/transfer_axis /dev/laser_axis
```

The rules match each drive's USB serial number, so the names follow the drives
regardless of which socket they use. Replace the serials in the file for other hardware
(`udevadm info -a -n /dev/ttyACM0 | grep -m1 serial`).

## Run

```bash
cd ~/Desktop/Laser_Project
source venv/bin/activate
sudo venv/bin/python3 laser_app.py
```

Then open `http://<pi-address>:5000`. The startup log shows which drives are online and
which axes need homing.

## Operation

### Homing

Both axes home with Method 17 toward their NL switch. The transfer axis takes the switch
release point as 0 mm and stays there; the laser axis takes it as -2.5 mm and then moves
up to 0 mm. Home the **transfer axis first**: the collision rule treats an unhomed axis as
being in the worst possible position, which blocks laser homing.

### Manual program

1. Select a part, press **LASER PROGRAM START** — transfer to the loading position, then
   laser to the side 1 height.
2. Load the part, press **MARK** or the foot pedal. MARK sends EzCad2's arming pulse; the
   pedal is wired to EzCad2 and arms it directly.
3. The transfer axis moves to the laser position. Both axes are verified (servo off,
   stopped, within 0.05 mm) before GPIO17 fires.
4. **OUT4** moves the laser axis to the next side height and fires again.
5. **OUT4 then OUT5** on the last side completes the part: short delay, then back to the
   loading position for the next one.

**STOP** halts both axes and remembers the step; **RESUME** continues it; **RESET** ends
the run and makes START available again.

The matching EzCad2 program waits for two triggers on the first side (arming + mark) and
one per following side, and emits OUT4 after every side plus OUT5 at the end.

### Axis jog

Switch an axis ON, then press and **hold** an arrow. The axis moves while held and stops
on release, at the entered maximum distance, or at a limit. A missing browser heartbeat
(0.5 s) stops the axis as a dead-man. Jog and homing are locked while a program runs.

### Group collision rule

While the transfer axis is above its limit (default 200 mm), the laser axis may not be
below its minimum (default 200 mm), and the reverse. It applies to jog, homing and all
program moves. Jog moves stop at the limit; program moves that would break the rule are
refused. Unhomed or disconnected axes count as worst case. Part heights below the laser
minimum are rejected when a recipe is saved.

## Configure tab

| Group | Parameters |
|---|---|
| Transfer / Laser axis homing | fast search speed, slow approach, timeout, start/stop |
| Manual program | loading position, laser position, OUT5 delay, OUT4/OUT5 timeout, OUT5-after-OUT4 timeout, speed per axis, speed override (%) |
| Group collision configuration | on/off, transfer limit, laser minimum |
| Laser trigger | GPIO17 pulse width |
| Part recipes | side count and one part height per side |

Value ranges are shown next to each field. Program parameters cannot be changed while a
program is running.

## Safety and interlocks

- The laser fires only with both servos off, both axes stopped and both positions verified.
- Unexpected OUT4/OUT5 means EzCad2 is out of step → error, no further pulses.
- Missing OUT4/OUT5 within the timeout → error.
- Inputs are debounced (20 ms stable, 400 ms lockout) so contact bounce is not double counted.
- A drive disconnecting during a program → error; its reference is dropped and homing is
  required after reconnect.
- The foot pedal pressed out of sequence → error, since EzCad2 then holds an extra trigger.
- EMERGENCY STOP stops both axes and switches the servos off.

The laser axis is vertical with no brake. Its servo switches off after every move
(`hold_after_move` in `AXIS_CONFIG`); set it to `True` if the head sinks.

## Diagnostics

The machine event terminal has **COPY** and **DOWNLOAD** buttons. Every OUT4/OUT5 edge is
logged with a running count, the active side and the pulse width, which is usually enough
to tell a missed pulse from an extra one.

Common drive alarms:

| Alarm | Meaning | First thing to check |
|---|---|---|
| Er47.0 | excessive position deviation | speed too high for the gain; lower it or auto-tune |
| Er41.1 | locked rotor over-temperature | mechanical jam, or homing into a hard stop |
| Er20.x | encoder / multi-turn data | encoder cable; reset with F31.10 = 4, then re-home |

After any mechanical work or encoder cable removal, re-home. In absolute mode the app
otherwise trusts the saved reference at startup.

## Files not in version control

`settings.json` is machine-local and changes whenever the UI saves. Keep `parts/` in the
repo only if the recipes belong to this machine.

## Status

The manual program is in production use. The automatic cycle (`AUTO_ENABLED = False`)
predates the two-axis design and is not yet reworked.