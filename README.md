# Pixhawk (FMUv3) toolkit -- macOS

Bring up a Pixhawk 1 class flight controller from macOS: flash it, configure
it, calibrate it, and check its wiring, without Mission Planner.

Ported from `~/apm28-tools`, which does the same job for an APM 2.8. The
port-handling, the verified-write discipline and the measure-don't-guess
habits carried over; everything tied to a 2015 autopilot did not.

## The board this was built against

    board_type 9 (fmuv3)        flash 2080768 bytes (2 MB)
    STM32F42x, OTP blank        sn 002800233134510131383230
    ArduCopter 4.7.1 (dbe79216) ChibiOS

Sold as "Pixhawk Pro FMU v10" in a cube-shaped case. It is **not** a Hex Cube
Black -- the inventory settles it: one IMU (MPU6000 on SPI), one barometer,
one onboard magnetometer (LSM303D on SPI), and a blank OTP area. A Cube
reports three IMUs and two barometers. This is a clone of the 3DR Pixhawk 1
design with the good 2 MB silicon, which is what makes `fmuv3` firmware the
right target.

`fmuv3` rather than `Pixhawk1` deliberately: both are board_id 9 and both
fit, but the fmuv3 hwdef probes a superset of sensors -- mpu6000, mpu9250,
icm20948, icm20602, lsm9ds0, two MS5611s -- so a clone that populates
something unexpected still comes up. The `Pixhawk1` target probes only what a
genuine 3DR board carries.

## Propellers off

Off the motors, not pointed away. Outputs are undefined while the bootloader
runs and again when fresh firmware boots. Do all firmware and configuration
work on USB power with the flight battery unplugged; the battery goes on only
for the ESC and motor checks, and the props stay off for those too.

## Quick start

```bash
./setup_env.sh              # one-time: Python venv
./identify.py               # read-only: what is on the board, what it needs
./wiring_check.py           # read-only: is it all connected right
```

Activating the venv is optional. `#!/usr/bin/env python3` picks the *system*
python, which has no pymavlink, so the APM toolkit's tools died with
`ModuleNotFoundError` whenever anyone forgot `source .venv/bin/activate` --
a tool that works in one shell and not another is indistinguishable from a
broken tool. `lib/cubelink.py` checks for pymavlink at import and, if it is
missing, `execv`s the project venv's interpreter. `./tool.py` therefore works
from any shell.

Then `RUNBOOK.md`, which is the ordered procedure.

## What each tool does

| Tool | What it does |
|---|---|
| `identify.py` | Read-only: board, firmware, every sensor and its bus, parameters, and a checklist |
| `wiring_check.py` | Read-only: power rails, safety, IMU/baro/compass/GPS/RC/outputs, blockers vs pending |
| `preflight.py` | Everything at once before a flight; blockers vs warnings, board's PreArm treated as authoritative |
| `paramgrep.py` | What is this parameter called in *this* firmware |
| `preload_params.py` | Apply/back up/diff/restore parameter profiles, every write verified |
| `fetch_firmware.sh` | Download and validate an ArduCopter `.apj` |
| `flash_pixhawk.sh` | Flash it, with the bootloader checks that make it safe; `--identify` writes nothing |
| `calibrate_accel.py` | Drives the firmware's six-position accel routine |
| `level_calibrate.py` | The level reference (`AHRS_TRIM`), measured and verified |
| `calibrate_compass.py` | Drives the onboard MAG_CAL, and refuses a bad fit |
| `calibrate_radio.py` | `RCn_MIN/MAX/TRIM` from live travel, with two refusals that matter |
| `which_compass.py` | Which magnetometer is physically on the mast |
| `check_imu.py` | Gyro and accel health, by rotation rather than by watching it sit |
| `check_stick_sense.py` | Which WAY each stick moves it -- the fault no PWM range can show |
| `rc_live.py` | All channels live, travel and jitter; `--failsafe` for the TX-off test |
| `mode_monitor.py` | Mode-switch PWM -> slot -> mode, live |
| `gps_check.py` | Fix, satellites, HDOP, wander; separates "not detected" from "no fix" |
| `motor_test.py` | One motor at a time: wiring order and rotation. Needs the battery |
| `log_download.py` | List and pull dataflash logs |
| `gcs.sh` | MAVProxy as a ground station |
| `lint.sh` | Static checks; `tests/devid_test.py` for the decoders |

## What is different from the APM 2.8 toolkit

**No firmware ceiling.** The APM was frozen at ArduCopter 3.2.1 because
nothing newer fits in 256 KB of ATmega flash. This board runs current
ArduPilot, so the advice is the opposite: stay current, and upgrade *before*
calibrating, since a major-version change rewrites defaults.

**No MAVLink bridge.** The APM speaks only MAVLink 1 and QGroundControl emits
only v2, which is why `mavbridge.py` had to exist. This board speaks v2 and
QGC talks to it natively. Nothing like `mavbridge.py` is shipped here.

**No solder bridge to cut.** On the APM, the onboard magnetometer shares the
I2C bus with the external one and the trace has to be cut with a knife. Here
the onboard mag is on SPI, so both compasses coexist and the EKF gets a
second opinion for free.

**No hand-rolled sphere fit, and no orientation solver.** ArduPilot 4.x does
the magnetometer fitting on board, including soft-iron terms and -- with
`COMPASS_AUTO_ROT` -- the external compass's orientation. The APM toolkit's
`solve_compass_orient.py` existed because 3.2.1 could not do any of that.

**One RC input.** The APM took eight individual PWM leads. This board has one
RCIN port expecting SBUS or PPM. Moving the receiver over is a wiring change,
not a parameter change.

**Verified writes, unchanged.** ArduPilot still clamps out-of-range writes and
ignores read-only ones, so every write here is confirmed by reading back the
`PARAM_VALUE` the board broadcasts.

## Three traps specific to 4.7

**Parameters were renamed and re-unitised.** `ANGLE_MAX 3000` became
`ATC_ANGLE_MAX 30`, `RTL_ALT 1500` became `RTL_ALT_M 15`, `WPNAV_SPEED 500`
became `WP_SPD 5`, `LAND_SPEED` became `LAND_SPD_MS`. Worst of all,
`ARMING_CHECK` became `ARMING_SKIPCHK` with the **opposite sense**: the old
one listed checks to run, the new one lists checks to skip. Copying a value
across would disable a check. `./paramgrep.py` exists to ask the board rather
than guess.

**A driver's parameters do not exist until it is enabled *and* the board has
rebooted.** `BATT_VOLT_PIN` is genuinely absent until `BATT_MONITOR` is set
and the board restarts -- the parameter count went 990 -> 1003 -> 1009 across
that sequence here. `preload_params.py` makes a second pass, and `--reboot`
forces it.

**A dropped `PARAM_VALUE` looks exactly like a missing parameter.** ~1000
parameters stream over USB and loss is normal; a board that has just rebooted
dropped all but 123 of 1009 in one run here. `fetch_params` tracks indices
and keeps filling the gaps, because reporting a dropped parameter as absent
sends you chasing renames that never happened.

## The GPS module is two cables

A GPS/compass module has a 6-pin serial lead and a 4-pin I2C lead, and they
fail independently. Compass seen but no GPS means the serial lead; GPS seen
but no compass means the I2C lead. `wiring_check.py` says which, and says so
in those terms.

## Third-party code

`tools/uploader.py` is ArduPilot's firmware uploader, vendored verbatim from
`Tools/scripts/uploader.py` at tag `Copter-4.7.1`, BSD licensed, copyright the
PX4 Development Team. Its copyright header is intact. It is vendored rather
than fetched at flash time so that flashing works offline and the exact
uploader used is recorded alongside the image it wrote.

Everything else here is original.

## Not covered here

Deliberately not automated:

* **ESC calibration** -- props off, battery in, throttle high at power-up. It
  commands full throttle by design; scripting it would be scripting a hazard.
* **Flight mode assignment on the transmitter** -- radio-specific.
* **PID tuning and the harmonic notch filter** -- airframe-specific, and the
  notch needs a hover log to set. The profiles here ship stock gains.
