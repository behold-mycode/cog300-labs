# Object following and right-wall following

Choose a mode in the browser sidebar. Space/Esc, Stop, a new driving key, settings edits or connection loss cancels autonomous control. Both modes use the current power setting and calibrated wheel inversion. Defaults are in `Robot.h`; control logic is in `Navigation.cpp`.

## Follow object

Only the front sensor affects motion. Default target: **20 cm**, deadband: **±2 cm**.

- Too far: both wheels forward.
- Too close: both wheels reverse.
- Inside the deadband: stop.
- Invalid/stale reading or object farther than **150 cm**: hold stopped.

The command magnitude is `clamp(6 × abs(distance − target), minimumPWM, maximumPWM)` outside the deadband. Battery defaults are maximum 255 and minimum 255 PWM. A minimum of zero removes the motor-starting floor. At full output the car may overshoot/oscillate; tune on the real robot using a broad flat target, measured distance and short trials. No turning, object identification or rear sensing is provided.

## Follow right wall

Default wall target: **12 cm**. The intended floor-test band is 10–15 cm; this is not yet physically verified. Set the wall distance in the controller. The right sensor is assumed to point sideways.

Tracking uses pure P control: `error = right_cm - target_cm`, `correction = 8 * error` PWM, limited to half the maximum PWM. There is no integral or derivative term and no deadband jump. Twice the distance error produces twice the steering correction until saturation. The outer wheel runs at maximum PWM and the inner wheel never drops below half that value. This helps avoid a stalled inner wheel; calibrate the maximum on the floor. Minimum follow PWM applies only to object following.

Front distance at or below 10 cm starts a **250 ms powered backup**, followed by a left spin. Front distance at or below 20 cm starts a left spin directly. Front recovery takes priority over missing right readings. A turn requires at least **550 ms of powered rotation**, front clearance of **35 cm**, and a valid right wall within 100 cm before tracking resumes. Left-spin magnitude also uses P control: `8 * (35 - front_cm)` PWM, clamped between the configured minimum (at least 1) and maximum. With minimum and maximum both set to 255, corner turns stay at full power. Lowering the minimum enables proportional slowing as clearance increases; the minimum keeps the wheels moving during turn commitment and reacquisition. This controls clearance, not measured heading. Motor reversal downtime does not count toward powered motion. Each backup/turn phase has a 2.5-second elapsed timeout that stops and exits wall mode. A timeout requires selecting the mode again, not reconnecting.

A valid right reading beyond 100 cm starts a gentle forward-right search, bounded to four seconds. Invalid echoes are not treated as open space: invalid front readings hold stopped; invalid side readings hold tracking/search stopped but do not prevent front recovery. Persistent invalid readings during a maneuver lead to its timeout. A search timeout also exits wall mode. Stop and connection-loss handling remain active throughout.

### First battery floor trial

Restart the controller server to load the new defaults, upload the firmware, and apply `CONFIG 255 255 1 0 20 12`. Start parallel to a long flat wall, 12 cm away. Check that too-far readings steer right and too-close readings steer left. Then test a front corner and a start farther from the side wall. Record telemetry and measured clearance. Adjust maximum PWM if wheels stall or motion is too fast; tune `TURN_MIN_MS` from observed turns. There are no encoders to confirm a 90-degree turn and no rear sensor, so leave space behind the robot for the short backup.

## Sensors and motor timing

Sensors alternate one ping every 70 ms (each sensor about 7 Hz), with 30 ms echo timeout. Echo timing uses interrupts on A1/A3 for UNO R4 WiFi. Accepted range is 2–400 cm. Samples expire after 350 ms. Sensor acquisition is separate from navigation and never blocks manual driving.

Wheel reversal inserts a 250 ms unpowered interval. Repeated requests do not restart that interval. Stops cancel pending output. Configuration changes also pause before later movement. Logs run at 2 Hz independently of control/sensor updates.

## Serial Monitor examples

After resetting the board, at 115200 baud / Newline:

```text
CONFIG 255 255 1 0 20 12
MODE FOLLOW
STOP
MODE WALL
STOP
```

For a battery tuning trial, a **starting example**, not a verified calibration, is `CONFIG 150 80 1 0 20 20`. Increase the minimum if the wheels stall; reduce output or gain if control overshoots. Gain/deadband/range/turn constants are grouped in `Robot.h`. Keep the current signal wiring; follow the battery wiring guide before changing the supply.
