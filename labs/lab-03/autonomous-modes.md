# V2 functions and P-control modes

The firmware starts stopped in MANUAL. Autonomous modes must be explicitly selected. This milestone provides a first implementation for hardware tuning, not a physically calibrated robot.

## Current functions

- Manual forward/reverse, curves, spins, individual wheels and timed moves through Serial Monitor or the existing Python controller.
- Wheel/preset test sequences, per-wheel PWM trim and direction inversion.
- STOP / S / !, finite-duration stopping and a 500 ms controller heartbeat timeout.
- Independent front/right echo measurement and LOG ON/OFF snapshots every 500 ms.
- MODE FOLLOW: forward distance keeping with the front sensor.
- MODE WALL: right-wall distance regulation with front-obstruction handling.
- Runtime P gains and wall base speed. Settings reset when the Arduino resets.

## Start from Arduino Serial Monitor

Upload the whole V2 folder, including the new `Navigation.ino` tab. Close/disconnect the Python controller, choose 9600 baud and Newline, then use:

```text
LOG ON
STATUS
KP FOLLOW 6.0
MODE FOLLOW
STOP
KP WALL 3.0
BASE 100
MODE WALL
STOP
MODE MANUAL
```

MODE commands stop previous motion before starting the selected behavior. Changing KP or BASE also stops and returns to MANUAL: re-enter the mode after each tuning change. STOP/S/! and manual movement commands cancel autonomy. HELP also stops. LOG and STATUS do not cancel the mode.

The desktop controller still provides manual driving; mode buttons are not added in this milestone. Use Serial Monitor for autonomous commands. A controller HELLO/CONFIG reconnect stops autonomy. If autonomy is selected in an active controller session, its heartbeat timeout still applies. Standalone Serial Monitor mode has no host heartbeat; it continues until STOP, a mode change or the autonomous conditions below stop it.

## Measurement vs movement policy

Sensor acquisition never directly writes motor outputs. `Safety.ino` is a legacy filename: it now measures/validates/logs sensors. `Navigation.ino` decides how distance affects autonomous motion. `Motor.ino` owns motor output and manual motion timing. `Serial.ino` handles commands. All pin assignments are in the main sketch.

MANUAL keeps the requested logging-only behavior: missing/close sensor readings do not block or stop manual driving. FOLLOW requires only a valid fresh front reading. WALL requires both fresh front and right readings. Invalid or stale required readings stop output; the selected mode waits and can resume when measurements recover. After a corner-turn timeout it returns to MANUAL and requires a new command.

Green means a valid front reading, not a clear path or autonomous enable indication. In a moving autonomous mode, LOG appends its state and signed requested wheel PWM. Wheel trim/inversion are applied afterward; STATUS shows actual PWM too.

## FOLLOW: 20 cm, no automatic reverse

`error = front_cm - 20`

`forward_pwm = clamp(Kp_follow × error, 0, 180)`

Both wheels receive the same requested forward speed. At/below 20 cm the motors stop immediately, bypassing the gradual speed ramp. After stopping, a reading must exceed 22 cm to resume; this small deadband reduces repeated start/stop near 20 cm. No automatic reverse is used. A missing/stale front reading or a distance above 150 cm stops the output instead of chasing a far-away echo. Right readings are logged but do not affect FOLLOW.

This is distance keeping for an object ahead, not person recognition or directional tracking. A front ultrasound beam cannot determine whether the target moved left or right. The 150 cm capture limit is an initial design choice in `FOLLOW_MAX_CM`.

## WALL: return to 20 cm on the right

`error = right_cm - 20`

`correction = Kp_wall × error`

`left_pwm = base_pwm + correction`

`right_pwm = base_pwm - correction`

Too far from the right wall gives a faster left wheel and steers right. Too close gives a faster right wheel and steers left. At 20 cm both requests equal the base speed. Correction is clamped to ±base speed and outputs to 180, so ordinary wall tracking never reverses a wheel.

When front distance reaches 20 cm, tracking stops, pauses unpowered for 250 ms and begins a left pivot at PWM 90. It returns to tracking when front clearance reaches 25 cm. A pivot that cannot clear the front within 2.5 seconds stops and returns to MANUAL. A required invalid reading stops output. This is an initial inside-corner maneuver requiring physical verification, not a measured 90° turn. Right turns around outside corners and doorways are not implemented; an invalid right reading stops rather than being interpreted as a turn instruction.

## Tuning the P term

There are no I or D terms yet. Both gains are in PWM units per centimetre of error. Initial gains are FOLLOW 6.0 and WALL 3.0; these are uncalibrated defaults. `KP FOLLOW value` and `KP WALL value` accept finite numbers from 0 to 20. `BASE value` accepts 0–180 and sets wall-tracking speed only.

The control update is every 50 ms; wheel requests change by at most 10 PWM per update. Required-reading failures and the FOLLOW stop threshold bypass that ramp. Wheel direction reversals pause unpowered for 250 ms. The gradual change limits abrupt commands but does not prove physically smooth motion.

1. First verify ruler accuracy and independent readings from both sensors, then wheel direction with the chassis supported.
2. Establish the lowest base PWM at which both wheels reliably move. A gain cannot correct inadequate motor supply or a stalled motor; the USB bench supply may limit this test.
3. Use a straight wall and fixed base speed. Start with a small wall gain (for example `KP WALL 1.0`), then enter MODE WALL. Observe correction from slightly farther and slightly closer than 20 cm.
4. STOP before each gain change. Increase gradually if correction is weak; reduce if it repeatedly weaves across the target. Keep speed and initial position comparable and record gain, distance trace and requested wheel PWM.
5. Tune FOLLOW separately with a broad flat target kept ahead of the sensor. Low PWM near the target may not overcome motor friction, so verify actual settling distance rather than treating the equation as proof.
6. Validate the left-corner state separately after straight-wall tracking is stable.

A motor stop here disables drive and lets the robot coast. A 20 cm software threshold does not guarantee the chassis physically stops 20 cm away. Measure coasting distance at the chosen speed before relying on that clearance. Alternating pings give each sensor a nominal 140 ms update; samples expire after 350 ms.

## Verification

The UNO R4 WiFi build and host tests verify P correction signs, independent sensor failures, 20/22 cm holding, no reverse in FOLLOW, wall steering, reversal pauses, corner timeout, tuning validation, manual override and controller timeout. Hardware behavior, gains, coasting distance and sensor cross-talk remain to be verified.
