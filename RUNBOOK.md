# Bring-up runbook

Everything here needs the USB port or your hands, so it runs from your
Terminal.

**Propellers off for all of it.** The battery goes on only at step 8.

## Step 0 -- before anything

* Props off the motors. Not pointed away -- off.
* Battery unplugged.
* Quit QGroundControl and any MAVProxy. A serial port has one owner, and
  QGC's AutoConnect reopens it on its own even after you disconnect the link.
* Plug the board **straight into the Mac**, not through a dock. Measured
  through a Thunderbolt hub here: 4.47 V on a 5 V rail.

```bash
cd ~/pixhawk-pro-b-cube
./setup_env.sh
```

No need to activate the venv -- the tools re-exec into it themselves if the
shell's python lacks pymavlink.

## Step 1 -- what have we actually got

```bash
./identify.py
```

Writes nothing. Firmware, board id, every sensor and which bus it is on, the
parameter summary, and a checklist. Everything below depends on it.

## Step 2 -- back up before changing anything

```bash
./preload_params.py --backup
```

Lands in `logs/params-<timestamp>.params`, Mission-Planner readable.

## Step 3 -- firmware, if it is old

```bash
./flash_pixhawk.sh --identify          # reads the bootloader, writes nothing
./fetch_firmware.sh fmuv3 stable
./flash_pixhawk.sh --target fmuv3
```

Do this **before** calibrating: a major-version change rewrites defaults for
parameters it knows and orphans ones that no longer exist. Flashing after a
calibration means doing the calibration twice.

A 1 MB board (old F427 silicon) cannot run these images -- use `fmuv2` or
`Pixhawk1-1M`. `--identify` prints the real flash size, which is the only
trustworthy answer.

## Step 4 -- wiring, with no battery

```bash
./wiring_check.py
```

The headline check. Power rails, safety switch, IMUs, barometer, compasses
and their buses, GPS detection, RC decoding with a jitter test, the motor
output map, the battery monitor, logging. Splits results into blocking,
worth-a-look, and pending.

Fix anything blocking here before going on. Then the individual tools for
whatever it flagged:

```bash
./gps_check.py              # GPS: detected vs no-fix are different faults
./rc_live.py                # per-channel travel and jitter
./which_compass.py          # which magnetometer is on the mast
./check_imu.py              # gyro and accel, by rotation
```

## Step 5 -- parameters

```bash
./preload_params.py --list
./preload_params.py --dry-run base frame-quad-x power-module-3s failsafe-3s \
    fltmodes-3pos-land flight-logging
./preload_params.py base frame-quad-x power-module-3s failsafe-3s \
    fltmodes-3pos-land flight-logging
```

Every write is read back and confirmed. Note that enabling a driver creates
its sub-parameters only at the **next boot** -- `BATT_VOLT_PIN` does not
exist until `BATT_MONITOR` is set and the board has restarted. The tool
re-reads and makes a second pass, and `--reboot` forces the issue.

`fltmodes-3pos-land` puts **LAND** on the third switch position, not RTL.
RTL flies a compass bearing home: until the compass is calibrated and the
GPS proven outdoors, it would do that confidently in the wrong direction.
Switch to `fltmodes-3pos-rtl` once both are done.

## Step 6 -- calibrate, in this order

```bash
./calibrate_accel.py        # six positions, flat surface
./level_calibrate.py        # the level reference, a different thing
./calibrate_compass.py      # assembled aircraft, every attitude
./calibrate_radio.py --write  # transmitter ON, both extremes on every channel
```

The order is load-bearing:

* **accel before compass** -- the compass fit is interpreted in the body
  frame the accelerometers define.
* **level after accel** -- it measures how the board sits in the airframe,
  which only means something once the sensor itself is calibrated.
* **radio before failsafes** -- `FS_THR_VALUE` is only meaningful once
  `RC3_MIN` is measured.

Two refusals are deliberate. `calibrate_radio.py` will not run with the
transmitter off (signal loss puts ch3 at ~900 us, and writing that as
`RC3_MIN` destroys the throttle failsafe), and will not accept a sweep under
800 us. `calibrate_compass.py` will not accept a fit whose offsets exceed the
pre-arm limit.

## Step 7 -- modes and failsafes, tested

```bash
./mode_monitor.py           # flick the switch; PWM, slot and mode must agree
./check_stick_sense.py      # which WAY each stick moves it
./rc_live.py --failsafe     # transmitter OFF; ch3 must fall below FS_THR_VALUE
```

On the FS-i6 only **SwC** is a 3-position switch. Assign it in *Functions
setup -> Aux. channels -> Channel 5*, and save with a **LONG press of
CANCEL** -- a short press discards, which is the usual reason an assignment
"did not stick".

## Step 8 -- the battery goes on. Props still off

```bash
# 1. voltage: compare against a meter before trusting the failsafe
./wiring_check.py

# 2. ESC calibration -- all four at once, manual on purpose:
#      props off, aircraft held down, safety switch off,
#      throttle HIGH at power-up, follow the ESC beeps, throttle down.
#    Not scripted here: it commands full throttle by design.

# 3. then, one motor at a time
./motor_test.py --props-off --all
```

Read the motor test's sequence-to-motor mapping before concluding anything is
miswired; sequence 2 is the **rear-right** motor.

## Step 9 -- the whole set, every time after this

```bash
./preflight.py
```

Blockers versus warnings, and it treats the board's own PreArm messages as
authoritative. Then a tethered arm test before anything leaves the ground.

## If something goes wrong

| Symptom | Cause |
|---|---|
| `cannot open port` / busy | QGroundControl or another tool here holds it |
| No heartbeat | board in the bootloader, no firmware, or port held |
| Board not in `/dev/cu.*` | charge-only cable, or a hub; plug in directly |
| `Device not configured` after a reboot | the old `/dev` node has not gone yet; retry |
| Motor test "vehicle not landed" | land detector still settling; wait ~15 s |
| Motor test accepted, nothing spins | throttle below the ESC start threshold, or safety switch engaged |
| "not present in this firmware" on a write | 4.7 renamed it, or its parent driver is disabled -- `./paramgrep.py <part of the name>` |
| Heading wrong but calibration "good" | orientation, not offsets; rotate the aircraft and watch the GCS heading |
