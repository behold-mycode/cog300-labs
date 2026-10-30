# Object following and right-wall following

Choose a mode in the browser sidebar. Space/Esc, Stop, a new driving key, settings edits or connection loss cancels autonomous control. Both modes use the current power setting and calibrated wheel inversion. Defaults are in `Robot.h`; control logic is in `Navigation.cpp`.

## Follow object

Only the front sensor affects motion. Default target: **20 cm**, deadband: **±2 cm**.

- Too far: both wheels forward.
- Too close: both wheels reverse.
- Inside the deadband: stop.
- Invalid/stale reading or object farther than **150 cm**: hold stopped.

The command magnitude is `clamp(6 × abs(distance − target), minimumPWM, maximumPWM)` outside the deadband. Battery defaults are maximum 255 and minimum 255 PWM. A minimum of zero removes the motor-starting floor. At full output the car may overshoot/oscillate; tune on the real robot using a broad flat target, measured distance and short trials. No turning, object identification or rear sensing is provided.

## Follow right wall — PD tuning version

Default target: **15 cm from the right sensor to the wall**. The right sensor points sideways, perpendicular to the front sensor. Maximum PWM remains 255. These gains and distances are starting values for floor calibration, not a guarantee of exact clearance or heading.

### Tracking

`error = right_cm - target_cm`

`correction = 14 * error + 12 * filtered_distance_rate`

P uses PWM/cm; D uses PWM/(cm/s). A positive correction slows the right wheel and steers toward the wall; a negative correction slows the left wheel and steers away. D increases response when drifting away and brakes an approach that is already closing the gap. The derivative uses the **acquisition timestamps of distinct right readings**, not loop time. Clamp the measured rate to ±50 cm/s, then apply a first-order filter (`rate += 0.4 * (new_rate - rate)`). The controller holds the correction between side samples; front updates do not recompute D. Reset derivative history on mode/phase transitions and after an invalid-data stop.

The outside wheel stays at the configured maximum. Steering can reduce the inside wheel to 25% of maximum (formerly 50%), enabling tighter corrections while keeping a positive forward command. `minimum` is the object-following PWM floor; it does not flatten PD steering to full output.

### Continuous left corners

A front obstacle at **32 cm** starts a forward left arc, before the old 20 cm trigger. Both wheels remain commanded forward: the left wheel receives 35–55% of maximum, the right wheel full configured power. The front-clearance error adjusts this differential. No wheel reversal means the motor driver's 250 ms reversal pause is not triggered during an ordinary corner.

Do not finish merely because front distance crossed a fixed threshold. Finish only after at least 160 ms of actual forward turning and **two distinct valid front echoes** showing clearance. With a usable side reading below 50 cm, require `front >= max(55, 2.5 * right)`; otherwise require 90 cm of front clearance. The ratio was reduced from 4 to 2.5 after the physical robot was observed turning about 120 degrees. This makes the turn exit earlier without changing wheel speeds, PD gains or the two-echo confirmation. The ratio is an approximate alignment cue for planar walls, not a measured angle. A missing right echo cannot cause endless rotation while waiting for side validity.

After the corner, a short **560 ms powered alignment phase** keeps moving forward. It allows mild left correction if too close, but suppresses an immediate right correction that would undo the corner. Then normal PD tracking resumes. When the side echo is unavailable, allow up to 300 ms of forward reacquisition with valid/recent front clearance; stop if the side remains unavailable. Alignment has a 1.5-second total timeout.

A front reading at or below **10 cm** invokes emergency backup (250 ms of powered reverse), followed by an escape spin. This is a fallback for starting too close or an unexpected obstacle, not the ordinary corner motion. Corner/backup/escape phases each stop and exit after 2.5 seconds if unsuccessful. A valid side wall beyond 100 cm starts a bounded four-second rightward search. The desktop disconnects on navigation timeouts and requires reconnecting.

### Echo loss and timing

Each wall-mode sensor may reuse its last good reading for **less than 300 ms from that measurement**, covering one missed 140 ms sensor cycle. Missing echoes never mean clear space and never refresh this age. With no previous measurement, or after the grace expires, stop. Front-invalid stops reset clearance confirmation and PD history. Object-follow mode retains its immediate invalid-reading stop.

Raw telemetry still reports `-1` for an invalid echo. States ending in `_HELD` mean the wall controller is temporarily using a recent measurement. `REACQUIRING_WALL` identifies the bounded forward side reacquisition after a corner. Telemetry is now 10 Hz, so short transitions are visible in downloaded logs.

### Floor trial and limitations

Restart the desktop controller and apply `CONFIG 255 255 1 0 20 15`. Start parallel to a long flat wall, 15 cm from the right sensor. First observe tracking, then one inside left corner. Check that **both wheels actually keep rolling** through the arc. A positive PWM command does not establish that the inner wheel has enough torque; an inner-wheel stall requires motor/supply calibration and may require a larger turning radius.

Record the log and note the measured clearance and heading after the corner. Tune one aspect at a time: `WALL_KP` for distance correction, `WALL_KD` for damping, `FRONT_TURN_CM` for turn start, and `CORNER_CLEAR_RATIO` for turn exit. No encoders or heading sensor are present, so a 90-degree turn is inferred and must be checked on the floor. Rear clearance is not sensed during emergency backup.

The host kinematic regression tests the actual firmware at three idealized speeds (20, 30, 40 cm/s), with a 14 cm wheel track, 8/6 cm sensor offsets, linear PWM response, alternating 70 ms samples, and no slip or inertia. It checks one forward corner without stopping, reversing, collision or re-entering the turn. These are **model assumptions**, not measured dimensions or physical validation of this robot.

## Sensors and motor timing

Sensors alternate one ping every 70 ms (each sensor about 7 Hz), with 30 ms echo timeout. Echo timing uses interrupts on A1/A3 for UNO R4 WiFi. Accepted range is 2–400 cm. Samples expire after 350 ms. Sensor acquisition is separate from navigation and never blocks manual driving.

Wheel reversal inserts a 250 ms unpowered interval. Repeated requests do not restart that interval. Stops cancel pending output. Configuration changes also pause before later movement. Logs run at 10 Hz independently of control/sensor updates.

## Serial Monitor examples

After resetting the board, at 115200 baud / Newline:

```text
CONFIG 255 255 1 0 20 15
MODE FOLLOW
STOP
MODE WALL
STOP
```

For a battery tuning trial, a **starting example**, not a verified calibration, is `CONFIG 150 80 1 0 20 20`. Increase the minimum if the wheels stall; reduce output or gain if control overshoots. Gain/deadband/range/turn constants are grouped in `Robot.h`. Keep the current signal wiring; follow the battery wiring guide before changing the supply.
