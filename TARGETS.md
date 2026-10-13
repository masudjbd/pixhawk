# Target values, and what to do when they move

## The distinction that matters

**Calibration describes the physical aircraft.** When the board shifts on its
mount, `AHRS_TRIM` genuinely *should* change -- the aircraft really is at a
new angle, and restoring an old value re-imposes a measurement that is no
longer true. **Re-measure. Never restore.**

**Configuration describes your choices.** Nothing physical alters it. If it
differs from your baseline, something overwrote it, and pushing the baseline
back is exactly right. **Restore.**

`./preload_params.py --diff` splits a diff along that line, and `--restore`
touches configuration only.

## Live readings -- what good looks like

Check with `./preflight.py`. These are readings, not parameters.

| Reading | Good | Acceptable | Wrong, and what it means |
|---|---|---|---|
| 5 V logic rail | 4.9-5.3 V | 4.8-5.4 | under 4.3 -> ArduPilot refuses to arm; usually the cable or a hub |
| roll / pitch at rest, level surface | within 0.5 deg | within 1 deg | over 2 -> level reference off; it will accelerate sideways |
| attitude steadiness (sd) | under 0.1 deg | under 0.3 | over 0.5 -> something is rocking it: cable tension, a short leg |
| gyro bias at rest | under 0.001 rad/s | under 0.01 | over 0.05 -> it is moving, or an axis is faulty |
| vibration (VIBRATION msg) | under 5 m/s/s | under 15 | over 30, or any clipping -> the IMU is saturating |
| accel magnitude at rest | 9.6-10.0 | 8.5-11.0 | far off -> a dead axis; calibration cannot move it this much |
| Alt REL sitting on the ground | under 0.3 m | under 0.5 | 1 m+ -> barometer drifting; ALT_HOLD will chase it |
| battery voltage, 3S | 11.1-12.6 V | 10.5-12.6 | under 1 V -> sense lead unplugged, not a flat pack |
| GPS fix | 3 (3D) | 3 | 0 = not detected (wiring); 1 = detected, no sky |
| GPS satellites | 10+ | 8+ | under 8 -> position modes not trustworthy |
| GPS HDOP | under 1.5 | under 2.0 | over 2 -> poor geometry; wait or move |
| compass offsets, magnitude | under 150 | under 400 | over 600 -> fails pre-arm; too near power wiring |
| compass fitness (MAG_CAL) | under 8 | under 16 | over 16 -> incomplete rotation coverage |
| EKF variances | under 0.3 | under 0.5 | over 1.0 -> the estimator does not trust itself |

## Configuration -- restore these if they move

| Parameter | Value | Why |
|---|---|---|
| `FRAME_CLASS` / `FRAME_TYPE` | 1 / 1 | quad X |
| `ARMING_SKIPCHK` | 0 | skip no pre-arm checks (4.7's inverted replacement for `ARMING_CHECK`) |
| `ATC_ANGLE_MAX` | 30 | degrees; 45 once trimmed |
| `AHRS_EKF_TYPE` / `EK3_ENABLE` | 3 / 1 | EKF3; EKF2 no longer exists |
| `INS_GYRO_FILTER` | 20 | Hz, suits a soft-mounted board |
| `FS_THR_ENABLE` / `FS_THR_VALUE` | 1 / 975 | must sit below real `RC3_MIN`, above the receiver's loss output |
| `FS_GCS_ENABLE` | 0 | on only with a telemetry radio |
| `BATT_MONITOR` | 4 | voltage and current |
| `BATT_VOLT_PIN` / `BATT_CURR_PIN` | 2 / 3 | fmuv3 analog pins |
| `BATT_VOLT_MULT` / `BATT_AMP_PERVLT` | 10.1 / 17.0 | 3DR-style brick; verify against a meter |
| `BATT_LOW_VOLT` / `BATT_CRT_VOLT` | 10.5 / 9.9 | ~3.5 / 3.3 V per cell on 3S |
| `BATT_FS_LOW_ACT` / `BATT_FS_CRT_ACT` | 2 / 1 | RTL on low, land on critical |
| `COMPASS_USE` / `COMPASS_LEARN` | 1 / 0 | in-flight learning corrupts a good calibration |
| `COMPASS_AUTO_ROT` | 2 | fit the external compass's orientation during calibration |
| `LOG_BITMASK` | 196607 | detailed; the SD card can afford it |
| `FLTMODE1/4/6` | the three reachable slots | see `profiles/` |

## Calibration -- re-measure these, never restore

| Parameter | Changes when | Re-measure with |
|---|---|---|
| `AHRS_TRIM_X/Y` | the board moves on its mount, any crash, a new mount | `./level_calibrate.py` |
| `INS_ACCOFFS_*`, `INS_ACCSCAL_*` | rarely; accel ageing | `./calibrate_accel.py` |
| `COMPASS_OFS_*`, `COMPASS_DIA/ODI` | anything magnetic moves: wiring, battery position | `./calibrate_compass.py` |
| `COMPASS_ORIENT*` | the compass module is remounted | `./calibrate_compass.py` (AUTO_ROT fits it) |
| `RC*_MIN/MAX/TRIM` | transmitter endpoints, subtrim, a new radio | `./calibrate_radio.py --write` |
| `INS_GYROFFS_*` | every boot, automatically | nothing to do |

## The routine

Take a baseline when the aircraft is right:

```bash
./preload_params.py --backup --label GOLDEN
cp logs/params-*-GOLDEN.params logs/GOLDEN.params
```

Before every flight: `./preflight.py`

After any crash, hard landing or hardware change:

```bash
./preload_params.py --diff logs/GOLDEN.params
./preload_params.py --restore logs/GOLDEN.params   # configuration only
./level_calibrate.py                               # re-measure the level reference
./preflight.py
```

Then take a fresh baseline, so GOLDEN always reflects a known-good aircraft
rather than an increasingly stale one.

## On "self-healing"

Configuration can heal itself -- that is what `--diff` and `--restore` do,
and it is safe because those values have no physical meaning. Calibration
cannot, and should not: there is no way to know from software that the board
is now sitting 3.5 degrees left. What can be automated is the **noticing**.
