# Wiring map, and what each lead has to prove

This board is a **Pixhawk 1 class / FMUv3** flight controller (sold as
"Pixhawk Pro FMU v10, cube case"). It is not a Hex Cube Black, and that
matters: it has one IMU rather than three, one barometer rather than two, and
its OTP area is blank, so it is a clone of the 3DR design with the 2 MB
silicon. Confirmed by the bootloader itself:

    board_type 9 (fmuv3), flash size 2080768, OTP vid/pid ffffffff
    sn 002800233134510131383230, STM32F42x

## What moved from the APM 2.8, and why

| | APM 2.8 | this board |
|---|---|---|
| RC input | eight individual PWM leads | **one** RCIN port, SBUS or PPM |
| Flashing | avrdude over ISP, per-frame firmware | PX4 bootloader, one image, frame is a parameter |
| Compass | onboard mag on the same I2C bus -- needed a **solder bridge cut** | onboard mag is on SPI; **nothing to cut**, both compasses usable |
| Outputs | 8 PWM from the ATmega | MAIN 1-8 from the IO coprocessor, AUX 1-6 from the FMU |
| Battery sense | analog pins 12/13, mult 10.1/17.0 | analog pins 2/3, mult 10.1/17.0 (same brick) |
| Safety | none | hardware safety switch inhibits all outputs |
| GCS | MAVLink 1 only; QGC needed a bridge | MAVLink 2; QGC works natively |

The RC change is the one that bites. **Individual PWM leads cannot work
here.** There is one input, and the receiver has to put everything on it.

## Port by port

| Port | Goes to | Verify with |
|---|---|---|
| **POWER** (6-pin) | power module, between battery and ESC distribution | `./wiring_check.py` battery section, then a meter |
| **MAIN OUT 1-4** | ESC signal leads, quad X order below | `./motor_test.py --props-off` |
| **MAIN OUT 5-8** | unused on a quad | -- |
| **AUX OUT 1-6** | gimbal, lights, relays | -- |
| **RCIN** | FS-iA6B **SBUS** output | `./rc_live.py` |
| **GPS** (6-pin) | GPS module serial -> SERIAL3 | `./gps_check.py` |
| **I2C** (4-pin) | GPS module's magnetometer | `./wiring_check.py` compass section |
| **TELEM1 / TELEM2** | telemetry radio, if fitted | `./identify.py` serial section |
| **SD card** | logging | `./log_download.py --list` |
| **safety switch** | the button with the LED | `./wiring_check.py` safety section |
| **buzzer** | audible arming tones | listen |

### The GPS module is TWO cables

A GPS/compass module has a 6-pin serial lead **and** a 4-pin I2C lead, doing
different jobs. They fail independently, and the diagnosis follows directly:

* compass seen, GPS not seen -> the **6-pin serial** lead
* GPS seen, compass not seen -> the **4-pin I2C** lead
* neither seen -> the module has no power; check both, and the module's LED

This is exactly the state this aircraft was found in: the IST8310 answering
on I2C while the GPS driver never came up.

### Receiver: ONE lead, and not where the ESCs go

The mistake this board invites, coming from an APM 2.8: running an individual
white/red/black lead from each of the receiver's CH1-CH6 outputs. A Pixhawk
has **one** RC input. Those six leads have nowhere to go.

**MAIN OUT 5-8 are outputs, not spare inputs.** Putting a receiver there
connects two sets of driven pins together. MAIN OUT carries ESC signals only.

    FS-iA6B  PPM/CH1    ->  board RCIN (the 'RC' column)   one 3-wire lead
    ...and PPM must be switched ON in the transmitter: System setup ->
    RX Setup -> PPM output -> On. It is OFF by default, and without it the
    pin emits plain PWM, which the IOMCU cannot decode. See FSI6.md.
    FS-iA6B  CH1..CH6   ->  nothing              leave empty
    ESCs                ->  MAIN OUT 1,2,3,4
    nothing             ->  MAIN OUT 5..8

| wire | function | pin |
|---|---|---|
| white / yellow / orange | signal | `S` / `SIG` |
| red | +5 V | middle, `+` |
| black / brown | ground | `-` / `G` / `GND` |

Red sits in the middle at both ends, so a reversed plug swaps **signal and
ground** -- check the silkscreen before powering up.

RCIN is powered by the flight controller, so the receiver comes up on **USB
alone**. An LED that stays dark on USB is the lead or its orientation, not a
missing battery.

Which port on the receiver, in order of preference: a header labelled
**SBUS**; failing that **i-BUS** (the servo-side one, NOT `SENS`, which is the
sensor bus); failing that **PPM**. `RC_PROTOCOLS 1` means the firmware
auto-detects all three, so nothing needs configuring. With only CH1-CH6 and no
serial output at all, a PPM encoder is required.

Verify with `./rc_live.py`: every channel should read a value and **jitter a
few microseconds**. A bit-identical reading is not a decoded channel.

### Quad X motor order

Set by the mixer once `FRAME_CLASS 1` / `FRAME_TYPE 1` is written. You do not
choose it and you do not set `SERVOn_FUNCTION` by hand.

    MAIN OUT 1 -> Motor 1   front-right   CCW
    MAIN OUT 2 -> Motor 2   rear-left     CCW
    MAIN OUT 3 -> Motor 3   front-left    CW
    MAIN OUT 4 -> Motor 4   rear-right    CW

"Front" is where the **arrow on the board** points, not where the airframe
looks like its front. X vs plus is about the arms' position relative to that
arrow.

`./motor_test.py` walks these in ArduPilot's **test sequence** order, which is
clockwise from the front-right arm and is NOT the motor numbering:

    sequence 1 -> Motor 1 (front-right)
    sequence 2 -> Motor 4 (rear-right)
    sequence 3 -> Motor 2 (rear-left)
    sequence 4 -> Motor 3 (front-left)

Misreading sequence 2 as "motor 2" makes a correctly wired quad look
scrambled. It is the single most common false alarm in this whole procedure.

### ESC leads

Each ESC has a 3-wire servo lead: signal, +5 V, ground. Only **signal and
ground** are needed -- the board does not power ESCs and most ESC BECs should
not back-feed the rail. Check the silkscreen for which row is signal rather
than working from memory, and note:

* a plug fitted backwards gives no output; it does not damage anything
* the servo rail (+) is **not** powered from USB. With the battery out, the
  ESCs are dead no matter what the board does -- which is why the motor and
  ESC checks are the last ones in the runbook
* on this board the servo rail reads ~4.4 V with the battery **out**, tracking
  the 5 V logic rail within 150 mV. That means the rail is bridged to the
  board's own 5 V supply, as many clones do. Consequence: do not feed the rail
  from a second BEC as well

### Power rail health

`./wiring_check.py` reads the 5 V logic rail. Measured here: **4.47 V through
a Thunderbolt dock**, which is low -- ArduPilot refuses to arm under 4.3 V,
and the margin for peripherals is gone well before that. Plug the board
straight into the Mac for bench work.

## The failsafe that has to be tested, not configured

The FS-iA6B holds its **last frame** on signal loss unless it is told
otherwise. On the APM airframe it held 1996 us on the throttle channel, which
drove the board into ESC-calibration mode on the next boot and sat there
commanding full throttle on all four outputs.

So: configure the receiver's failsafe to drop the throttle channel low, then
prove it with `./rc_live.py --failsafe` -- transmitter off, watch channel 3
fall below `FS_THR_VALUE`. An untested failsafe is the one that bites.
