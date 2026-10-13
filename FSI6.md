# FlySky FS-i6 and FS-iA6B on this board

Carried over from the APM 2.8 airframe, with the one change that matters.

## PPM OUTPUT IS OFF BY DEFAULT -- this is the one that costs an evening

The FS-i6 (non-X) **cannot output S.BUS at all** without a transmitter
firmware upgrade, and the FS-iA6B's `Servo(i-BUS/S.BUS)` port emits **i-BUS**
by default -- which the Pixhawk's IOMCU does not decode. So the only protocol
this radio can give a Pixhawk-class board is **PPM on the CH1 pin**, and PPM
is **disabled by default**.

    Menu -> System setup -> RX Setup -> PPM output -> On
    save with a LONG press of CANCEL

The receiver must be powered and bound while you do this: AFHDS 2A is
two-way, and the transmitter pushes the setting to the receiver.

With PPM enabled, the receiver puts every channel on the CH1 pin and
**disables the CH2-CH6 PWM outputs**.

### Why this is so hard to diagnose

Every symptom points at wiring, and none of them are:

* receiver LED **solid red** -- bound, powered, receiving. The RF link is
  perfect and says nothing about what the output pin is doing.
* `chancount 0` at the flight controller, identically, whichever receiver
  port you try -- because i-BUS and plain PWM are both undecodable by the
  IOMCU, so two different wiring attempts fail the same way.
* nothing in the parameters is wrong. `RC_PROTOCOLS 1` already enables every
  protocol, so there is nothing to "switch on" at the ArduPilot end.

On this airframe that pattern survived four rewiring attempts -- SBUS port to
RCIN, PPM/CH1 to RCIN, re-seated against the known-good ESC lead orientation.
The board was never the problem and neither was the cable. **Check the
transmitter's RX Setup before re-seating a single connector.**

## The change: SBUS, not eight PWM leads

The APM took a separate PWM lead per channel. This board has **one** RCIN
port. Wire **PPM/CH1** to RCIN with a single 3-wire lead (see the PPM section
above -- SBUS is not available on this transmitter).
RCIN is powered by the flight controller, so it works on USB alone -- a
receiver LED that stays dark on USB means the lead or its orientation, not a
missing battery.

`RC_PROTOCOLS 1` (all protocols) lets the firmware auto-detect, so SBUS,
i-BUS or PPM all work without further configuration. The FS-iA6B's i-BUS
output also works if that is what is to hand.

On SBUS every channel arrives in one frame, so either all of them decode or
none do. **A single dead channel among live ones is a transmitter mixing
problem, not wiring.**

## The mode switch

Channel 5 must reach three positions to give manual / assisted / recovery.
**SwC is the only 3-position switch on the FS-i6** -- SwA, SwB and SwD are
all 2-position.

*Functions setup -> Aux. channels -> Channel 5 = SwC*, and **save with a LONG
press of CANCEL**. A short press discards, which is the usual reason an
assignment "did not stick".

Slot mapping at ArduCopter's fixed `readSwitch()` boundaries: ~1000 -> slot 1,
~1500 -> slot 4, ~2000 -> slot 6. Those three are the only ones a
3-position switch reaches, which is why the profiles repeat values into the
neighbouring slots -- PWM drift across a boundary then changes nothing.

Confirm with `./mode_monitor.py` before trusting any of it: endpoint and
subtrim settings move the actual PWM.

## The failsafe, and the thing it did to the APM

**The FS-iA6B holds its LAST FRAME on signal loss** unless it is configured to
do otherwise. On the APM airframe it held **1996 us on channel 3** -- full
throttle -- and ArduCopter 3.2.1 sampled that at startup, decided the pilot
wanted ESC calibration, printed `Entering ESC Cal: restart APM.` and stopped
before its main loop while driving all four ESCs at 1996. It looked like five
dead sensors.

That specific failure mode does not exist on ArduCopter 4.x -- there is no
throttle-high-at-boot ESC calibration path. But the receiver behaviour is
unchanged, and a receiver that holds a high throttle on signal loss is still
a receiver whose failsafe does nothing.

Verified on the APM airframe: with the transmitter genuinely switched **off**,
ch3 falls to **900 us**, below `FS_THR_VALUE` 975, and the board correctly
refuses to arm. So the holding behaviour is real, and a true power-off does
produce a low channel 3. Both facts matter: configure the receiver's failsafe,
and then test it.

```bash
./rc_live.py --failsafe     # transmitter off; watch ch3 cross FS_THR_VALUE
```

Calibrated endpoints from the APM airframe, as a starting expectation:
`RC3_MIN 1025`, `FS_THR_VALUE 975`. The threshold must sit **below** the
measured `RC3_MIN` and **above** whatever the receiver emits on loss.

## Never recalibrate with the transmitter off

On signal loss ch3 reads ~900. Calibrating then writes `RC3_MIN 900`, which
is below `FS_THR_VALUE` -- and the throttle failsafe is silently gone.
`./calibrate_radio.py` refuses to run with no signal and refuses to write an
`RC3_MIN` at or below the configured threshold, for exactly this reason.

## Pitch direction

On the APM airframe the FS-i6's elevator output ran opposite to what ArduPilot
expects: pushing the pitch stick forward flew the aircraft backward, and the
fix was `RC2_REV -1` (measured, after confirming roll was correct, which is
what ruled out a board mounted 180 degrees round).

**Re-measure this on the new board rather than assuming it carries over.**
Nothing in a parameter dump can see it: a reversed channel and a correct one
produce identical PWM ranges, and the aircraft self-levels either way, so it
hovers perfectly and only the *sign relative to trim* separates them. On 4.x
the parameter is `RC2_REVERSED`, not `RC2_REV`.
