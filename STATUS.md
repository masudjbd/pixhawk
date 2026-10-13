# Status

## Where this aircraft is, 2026-10-02

The APM 2.8 (ArduCopter 3.2.1) has been replaced by a Pixhawk 1 class board.
The airframe, radio and pack are the same ones `~/apm28-tools` was built
against: quad X, FlySky FS-i6 + FS-iA6B, 3S, 3DR-style power brick.

**Battery disconnected and propellers off throughout this session.** Nothing
below involved a live ESC.

### Board identity -- settled

    bootloader: board 9,0 rev 5, flash size 2080768 (2 MB), ext flash 0
    chip:       STM32F42x
    OTP:        vid/pid ffffffff  -> blank, i.e. a clone, not 3DR/Hex silicon
    serial:     002800233134510131383230  (matches the firmware's CPU UID line)

Sold as "Pixhawk Pro FMU v10, cube case". The sensor inventory rules out a Hex
Cube Black: a Cube reports three IMUs and two barometers, this board reports
one of each. It is a Pixhawk 1 clone with the 2 MB silicon, so `fmuv3`
firmware is correct and a 1 MB image would be a downgrade.

### Firmware -- upgraded

| | before | after |
|---|---|---|
| version | ArduCopter 3.6.12 (cb570c06), NuttX, Feb 2019 | **ArduCopter 4.7.1 (dbe79216), ChibiOS** |
| estimator | EKF2 (`AHRS_EKF_TYPE 2`) | EKF3; EKF2 no longer exists |
| parameters | 808 | 990 -> 1009 as drivers were enabled |
| compasses found | 1 | **2** -- 3.6.12 never probed the onboard LSM303D |

Flashed with `./flash_pixhawk.sh --target fmuv3`, image verified by the
bootloader. The 3.6.12 parameter set is saved in
`logs/params-unknown-20261002-002900.params`.

The second compass appearing is the notable part: it was not a wiring change,
it was a firmware that probes for it.

**And then it went away again.** `COMPASS_DEV_ID2` read 131594 (LSM303D on
SPI) on the first boot after the flash, and reads 0 after two soft reboots,
with `SCALED_IMU2` no longer streaming either. A MAVLink reboot does not
power-cycle an SPI sensor, so this may be a warm-reset probe failure rather
than a dying chip -- **pull the USB lead, plug it back in, and run
`./identify.py` to compare a cold boot against a warm one.** Either way,
only the external IST8310 can be relied on until that is settled, and the
aircraft is flyable on one compass.

### Sensors, as the board reports them

| | device | bus |
|---|---|---|
| IMU 1 | MPU6000 (accel + gyro) | SPI bus 1, addr 0x04 |
| IMU 2/3 | none | -- |
| barometer | 1 (MS5611 class) | SPI |
| compass 1 | IST8310 | I2C bus 0, addr 0x0E -- the GPS mast module |
| compass 2 | LSM303D | SPI bus 1 -- onboard |

Single IMU: normal for this clone, and it means an IMU fault in flight is
unrecoverable rather than a switchover. Worth knowing, not worth fixing.

### Configuration applied

`./preload_params.py base frame-quad-x power-module-3s failsafe-3s
fltmodes-3pos-land flight-logging` -- 15 writes, all verified by read-back,
plus a second pass after a reboot for the battery driver's parameters.

* `FRAME_CLASS 1` / `FRAME_TYPE 1` -- quad X. Was 0, i.e. no mixer at all.
* `BATT_MONITOR 4`, and its analog parameters came up at the fmuv3 defaults
  that happen to match a 3DR brick exactly: `BATT_VOLT_PIN 2`,
  `BATT_CURR_PIN 3`, `BATT_VOLT_MULT 10.1`, `BATT_AMP_PERVLT 17.0`.
  **Still to verify against a meter.**
* `BATT_LOW_VOLT 10.5` / `BATT_CRT_VOLT 9.9`, RTL on low, land on critical.
* `FS_THR_ENABLE 1`, `FS_THR_VALUE 975` -- the APM airframe's measured
  numbers, pending a fresh `RC3_MIN` from this board.
* Modes: STABILIZE / ALT_HOLD / **LAND** on the three reachable slots. LAND
  rather than RTL on purpose until the compass and GPS are proven.
* Logging raised to 196607, `LOG_DISARMED 1` for bench sessions.

### With the battery connected -- 2026-10-02, props off

Pack: 3S reading **11.69 V** (3.90 V/cell), 0.11 A idle, 98% remaining. The
battery failsafe no longer trips. **`BATT_VOLT_MULT` is still unverified
against a meter** -- the fmuv3 default of 10.1 happens to suit this brick, but
a divider tolerance of a few percent is the difference between a correct
reading and an RTL on a healthy pack.

**The low 5 V rail was the USB hub, not the board.** 4.46 V on USB through a
Thunderbolt dock; **5.06 V** with the power module supplying it. Vservo 5.01 V.
Nothing to fix.

#### Motor test -- all four spin; 1150 us was below the ESC start threshold

`./motor_test.py --props-off --all --pwm 1150 --yes`, safety switch
disengaged (verified: the outputs-enabled bit was set), props off.

At **1150 us** all four commands were ACCEPTED and **not one motor turned.**
At **1250-1350 us** they spin. The ESCs are standard PWM SimonK/BLHeli units
and have never been calibrated, so their start thresholds sit above 1150.

That is the whole explanation, and it is worth remembering in both
directions: an accepted command with a silent motor says nothing about the
wiring, and a throttle figure that works on one aircraft's ESCs can be below
another's floor. Use **1250 us** on uncalibrated ESCs.

Outcome, 2026-10-02: **all four motors spin on their correct output.**
Sequence 1 (front-right) was watched directly; sequences 2, 3 and 4 were
reported as done.

**Rotation confirmed correct on all four** (reported 2026-10-02): motor 1
front-right CCW, motor 2 rear-left CCW, motor 3 front-left CW, motor 4
rear-right CW. **ESC signal order and motor rotation are both correct; no
leads need swapping.**

**The board side was proven good first.** A direct probe (sample SERVO_OUTPUT_RAW at
50 Hz while commanding sequence 1 at several PWM values) showed:

    idle                 ch1=1000 ch2=1000 ch3=1000 ch4=1000
    seq 1 at 1150 us     ch1=1150 ch2=1000 ch3=1000 ch4=1000
    seq 1 at 1250 us     ch1=1250 ch2=1000 ch3=1000 ch4=1000
    seq 1 at 1350 us     ch1=1350 ch2=1000 ch3=1000 ch4=1000

Correct channel, correct value, only that channel. So the mixer, the IOMCU
and the output pin all work, and the fault is downstream of the pin: ESC
power, the signal leads, or the motors. **ESC calibration is not the cause** --
1350 us is about 35% throttle, far above any ESC's start threshold.

This file briefly recorded the opposite. It was wrong: order and rotation are
**NOT** verified, and nothing about the ESC or motor wiring has been
confirmed. An accepted command proves the mixer and the link, and nothing
downstream of the output pin.

Two things learned here that the tool now encodes:

* ArduCopter checks RC calibration for **percent** throttle and skips it for
  **direct PWM** -- `mavlink_motor_test_check(chan, throttle_type != 1, ...)`.
  With no receiver wired, a percent test is refused with "RC not calibrated"
  while `--pwm 1150` runs fine. That is why `--pwm` exists.
* With the safety switch engaged the test is **ACCEPTED and nothing moves**,
  which is indistinguishable from four dead ESCs. `motor_test.py` now checks
  the outputs-enabled bit and refuses up front rather than letting that be
  misread.

### Open items, in the order they need doing

1. **GPS not detected** (`fix_type 0`, 0 satellites, `GPS1_TYPE 1` = auto,
   `SERIAL3_PROTOCOL 5`). The module's IST8310 answers on I2C, so the module
   has power and its 4-pin lead is good -- which points at the **6-pin serial
   lead**: unplugged, in the wrong port, or swapped with the I2C one.
2. ~~No RC input~~ -- RESOLVED, and the radio is calibrated (2026-10-02):

   | ch | min | max | trim | |
   |---|---|---|---|---|
   | 1 roll | 1010 | 2000 | 1510 | full sweep |
   | 2 pitch | 1000 | 1988 | 1485 | full sweep |
   | 3 throttle | 1023 | 2000 | 1023 | full sweep |
   | 4 yaw | 1000 | 1989 | 1489 | full sweep |
   | 5 mode | 1000 | 2000 | -- | SwC, all three detents seen (1000/1500/2000) |

   `FS_THR_VALUE 975` sits 48 us below `RC3_MIN 1023` -- the right
   relationship, still to be TESTED with the transmitter off.

   Channels 6-8 never moved, and the first run wrote `MIN = MAX` for them
   from a single sample. Restored to firmware defaults, and
   `calibrate_radio.py` now refuses to write an aux channel whose travel is
   under the 200 us connectivity floor -- the same "do not conclude from a
   measurement that did not look" rule the control channels already had.

   **Stick sense checked and corrected 2026-10-02.** Pitch was reversed,
   exactly as on the APM 2.8 airframe: stick forward gave ch2 trim +503, i.e.
   a positive demand, i.e. nose up, i.e. the aircraft would have flown
   BACKWARD. Roll measured correct in the same capture (+490), which is what
   rules out a board mounted 180 degrees round. Fixed with `RC2_REVERSED 1`
   (`profiles/45-rc-pitch-reverse.params`) and re-verified: all four axes now
   read correct.

   A trap worth keeping: the first capture was taken while the sticks were
   being SWEPT, and peak-deviation then reports whichever extreme happens to
   sit further from trim -- which produced a confident "roll, pitch and yaw
   all reversed" verdict that was pure noise. `check_stick_sense.py --capture`
   now tracks both extremes per channel and reports a swept channel as
   inconclusive rather than reversed. Hold ONE corner; do not sweep.

   Still to do: the failsafe test and `./mode_monitor.py`.
3. ~~5 V rail low~~ -- RESOLVED: it was the USB hub. 5.06 V on the power
   module.
4. **Servo rail is bridged to the board's 5 V supply** (read ~4.4 V with the
   battery out, tracking the logic rail within 150 mV), as many clones do.
   Harmless, but do not also feed the rail from a BEC.
5. **Verify pack voltage against a meter** and adjust `BATT_VOLT_MULT` if the
   board's 11.69 V disagrees. Do this before trusting the battery failsafe.
6. **ESC calibration** -- not yet done. The motor test proves order and
   direction; ESC calibration matches the four throttle ranges to each other,
   which is what stops an aircraft from yawing or leaning on takeoff. Stays
   manual: props off, battery in, throttle high at power-up.
7. **Nothing else is calibrated**: accel offsets zero, both compasses' offsets
   zero, RC endpoints at firmware defaults. Order is in `RUNBOOK.md`.
8. **Onboard LSM303D is intermittent** -- detected on a cold boot, absent
   after soft reboots. Power-cycle and re-run `./identify.py`.
9. **`COMPASS_EXTERNAL` is 0** on the I2C compass. Run `./which_compass.py`
   to settle which sensor is physically on the mast before applying
   `20-compass-mast-primary.params`, and record the answer in
   `airframe.local.json`.
10. ~~Motor wiring and rotation~~ -- VERIFIED. All four spin on the correct
    output, all four in the correct direction, at 1250 us.

### RC input -- RESOLVED 2026-10-02. Two faults stacked on each other.

`chancount 8`, channels 1-4 moving with real travel and jitter.

It took seven "still nothing" checks because there were **two** independent
faults producing one identical symptom:

1. the receiver's 3-pin lead was in the **MAIN OUT** block, not the `RC`
   column. It still powered the receiver -- this clone bridges the servo rail
   to the board's 5 V supply -- so the LED sat solid red and looked healthy
   while the signal went to an output pin.
2. **PPM output was off** in the transmitter, so even once the lead was in
   `RC`, the CH1 pin was emitting plain PWM.

Fixing either one alone changes nothing, which is what made it feel like a
dead board. The continuity test (receiver CH1 signal pin to board `RC` signal
pin) was what finally separated "wiring" from "signal" -- it should have been
the first measurement, not the twelfth.

### RC input -- the FS-i6's PPM output is OFF by default

Four wiring attempts failed identically with `chancount 0` while the receiver
LED sat solid red. None of them were wiring faults:

* the `Servo(i-BUS/S.BUS)` port emits **i-BUS** by default, which the IOMCU
  cannot decode
* `PPM/CH1` emits plain **PWM** until PPM is enabled **in the transmitter**
  (System setup -> RX Setup -> PPM output -> On), and it is off by default
* the FS-i6 non-X **cannot emit S.BUS** without a TX firmware upgrade, so PPM
  is the only option for this radio on a Pixhawk-class board

A solid receiver LED proves the RF link and says nothing about the output
pin. `RC_PROTOCOLS 1` already enables every protocol, so there was never
anything to change at the ArduPilot end. Check RX Setup before touching a
connector.

`SERIAL2_PROTOCOL` was set to 23 (RC input) as an i-BUS fallback via TELEM2
during diagnosis, and has been reverted to 2 (MAVLink2) now that PPM works,
so TELEM2 is free for a telemetry radio.

### Accelerometer calibration -- DONE 2026-10-03

Six positions, all banked, board reported "Calibration successful".

| axis | INS_ACCOFFS (m/s/s) | INS_ACCSCAL |
|---|---|---|
| X | -0.069 | 0.9950 |
| Y | -0.256 | 1.0004 |
| Z | +0.036 | 0.9847 |

Offsets far under the 3.5 limit, scale factors within 1.5% of 1.0.

**The apparent tilt was sensor error, not mounting.** Roll read -6.50, then
-8.75, then -3.51 degrees across successive readings before calibration, and
**-0.39 after**. A fixed mounting angle cannot wander like that. Trimming it
away with AHRS_TRIM beforehand -- which was asked for twice -- would have
baked accelerometer error into the level reference and then needed undoing.
The board's own trim report after calibration (`Trim OK: roll=5.54
pitch=-0.95`) is the real mounting offset.

#### Level reference -- DONE 2026-10-03, after the accel calibration

    before   roll +0.46 deg (sd 0.003)   pitch -0.14 deg
    after    roll +0.01 deg              pitch -0.02 deg
    AHRS_TRIM_X 5.54 -> 6.00 deg,  AHRS_TRIM_Y -0.96 -> -1.09 deg

The first attempt was REFUSED with sd 1.686 deg -- the aircraft was being
held rather than resting. Settled on a flat surface it reads sd 0.003, and
only then is the measurement worth anything. `AHRS_TRIM_X 6.00` agrees with
the board's own `Trim OK: roll=5.54` from the accel calibration, which is
what confirms it as a real mounting offset rather than residual sensor error.

#### The bug that cost four restarts: ArduPilot stops talking in text

`AP_AccelCal::gcs_vehicle_position()` sets `_use_gcs_snoop = false` the moment
a GCS answers with `MAV_CMD_ACCELCAL_VEHICLE_POS`. From that point the board
asks for each position by **sending COMMAND_LONG** with command
`MAV_CMD_ACCELCAL_VEHICLE_POS` and `param1` = the step, and reports
success (16777215) or failure (16777216) the same way. The human-readable
"Place vehicle X and press any key" text is only for GCSs that have not
proved they speak the protocol.

A tool watching only STATUSTEXT therefore banks position 1 and then waits
forever while the board asks, unheard, for position 2. Symptom: ack
ACCEPTED, then silence. Three other things masked it along the way:

* **No heartbeats.** pymavlink sends none, and ArduPilot will not run the
  interactive calibration for a channel it does not consider a live GCS. It
  worked the very first time only because APM Planner was open supplying
  them. `cubelink.heartbeat()` now does this, rate-limited rather than
  threaded (pymavlink's writer is not thread-safe).
* **APM Planner holding the serial port**, which killed two runs outright.
  One owner per port.
* **State does not survive a reconnect.** A stepwise design with one process
  per position cannot work: the board abandons the calibration when the GCS
  stops heartbeating, so it must be one long-lived connection. Running it as
  a persistent background process and reading its log is what made a
  conversational, one-position-at-a-time workflow possible.

### Compass calibration -- DONE 2026-10-03

Onboard MAG_CAL routine, aircraft assembled with the battery in place.

    status    success
    offsets   x +47  y +77  z +38   magnitude 98
    fitness   12.3
    board     "Mag(0) good orientation: 0 11.2"

Offsets of 98 are well inside the 150 "good" mark and nowhere near the 600
pre-arm limit -- the compass is not sitting in the ESC or battery field.
Fitness 12.3 is acceptable (under 16) rather than good (under 8); worth
redoing with more attitude coverage if heading ever looks wrong.

The board logged `MagCal: No location, fix_radius skipped`: with no GPS fix
it could not check the fitted radius against the expected field strength for
this location. Not a blocker, but the check did not run.

Only compass 1 (the IST8310 on I2C) exists to calibrate -- the onboard
LSM303D has still not re-appeared since the 4.7.1 flash.

### ESCs and spin thresholds -- RESOLVED 2026-10-03

**Symptom:** armed, three motors spinning, one not -- and as pitch/roll moved,
the idle motor would start while a different one stopped.

**Cause:** not wiring, not the mixer, not calibration. The stock
`MOT_SPIN_ARM 0.10` sends **1100 us** at armed idle, and every ESC on this
aircraft needs **~1250 us** to break away. Which motor happened to be turning
was luck; stabilisation pushing an output up carried that motor over its
threshold while another dropped below its own.

**Measured** (props off, ESCs calibrated, pack ~11.0 V):

    1100 us  none start        1210 us  none start
    1150 us  none start        1250 us  ALL FOUR start

All four matched at 1250, which is what makes it safe to work around.

**Fix:** `MOT_SPIN_ARM 0.26`, `MOT_SPIN_MIN 0.30`, measured rather than
guessed -- ArduPilot's documented procedure. Verified: all four now spin
steadily at armed idle and none drops out under pitch/roll.
See `profiles/35-spin-thresholds.params`.

**ESC calibration itself ran correctly** (full beep sequence heard: tone,
cell-count beeps, long beep at max, confirmation at min) and changed the
thresholds not at all -- 1250 before and after. These ESCs simply start high.
`ESC_CALIBRATION` was set to 1 for the procedure and **returned to 0**
afterwards: left at 1, any boot with a high throttle re-enters calibration
pass-through, which is exactly how the APM 2.8 ended up driving all four
outputs at 1996 us on a receiver failsafe.

**Still open on this:** 1250 us is high for a calibrated ESC (1050-1100 is
typical), and the pack was at 3.66 V/cell when measured -- less voltage, less
torque, higher break-away throttle. Re-measure on a full pack; these numbers
may come down. Also watch `MOT_THST_HOVER` (0.35 default, learned in flight)
against `MOT_SPIN_MIN` 0.30: if hover lands near the floor there is little
room left to reduce a motor for stabilisation, and the answer then is
better-matched ESCs rather than more tuning.

### Things worth not relearning

* `fetch_params` must fill gaps by index. A just-rebooted board delivered 123
  of 1009 parameters in one pass here, and every missing one would otherwise
  have been reported as "not present in this firmware".
* A driver's parameters appear only after it is enabled **and** the board
  reboots. `BATT_VOLT_PIN` did not exist until both had happened.
* 4.7 renamed and re-unitised much of the configuration, and
  `ARMING_CHECK` -> `ARMING_SKIPCHK` **inverts the meaning**. Writing the old
  value would have disabled a pre-arm check.
* The first version of the devid table named this aircraft's IST8310 an
  ICM20948. `tests/devid_test.py` pins the real readings so it cannot drift
  back.
* After a MAVLink reboot, macOS keeps the stale `/dev/cu.*` node around for a
  moment; reconnecting immediately fails with errno 6, "Device not
  configured". Wait for the node to disappear first.
* **Two tools on one serial port do not fail at open time on macOS.** Both
  processes get a working descriptor for `/dev/cu.*` and then steal each
  other's bytes, so the failure surfaces much later as pyserial's "device
  reports readiness to read but returned no data (device disconnected or
  multiple access on port?)" from a random read. `connect()` now checks
  `port_holder()` BEFORE opening and refuses with the holder's name, and
  every tool runs through `cubelink.run()`, which turns a mid-run serial
  failure into that same message instead of a traceback.
* `./tool.py` runs under the SYSTEM python, which has no pymavlink. The APM
  toolkit answered this with a README note telling people to activate the
  venv; they forget, and the tool then looks broken. `cubelink` now `execv`s
  the venv interpreter when pymavlink is missing, so running a tool directly
  works from any shell.
