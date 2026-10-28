# Object following and right-wall following

Choose a mode in the browser sidebar. Space/Esc, Stop, a new driving key, settings edits or connection loss cancels autonomous control. Both modes use the current power setting and calibrated wheel inversion. Defaults are in `Robot.h`; control logic is in `Navigation.cpp`.

## Follow object

Only the front sensor affects motion. Default target: **20 cm**, deadband: **±2 cm**.

- Too far: both wheels forward.
- Too close: both wheels reverse.
- Inside the deadband: stop.
- Invalid/stale reading or object farther than **150 cm**: hold stopped.

The command magnitude is `clamp(6 × abs(distance − target), minimumPWM, maximumPWM)` outside the deadband. Both default PWM limits are 255, so the weak-supply configuration runs at full output when moving. Lower minimum and maximum in Settings after switching to batteries; this enables proportional slowing near the target. A minimum of zero removes the motor-starting floor. At full output the car may overshoot/oscillate; tune on the real robot using a broad flat target, measured distance and short trials. No turning, object identification or rear sensing is provided.

## Follow right wall

Default wall target: **20 cm**, ±2 cm deadband. Base output is maximum PWM. Right-distance error produces a proportional steering correction (gain 8 PWM/cm), limited to the configured power. When too far from the right wall, slow the right wheel; when too close, slow the left. The outer wheel stays at the configured maximum. Actual trajectories depend on traction and wheel speed.

Front distance ≤20 cm starts a left spin. It ends when front distance reaches 25 cm. If it cannot clear within **2.5 seconds**, stop and exit the mode. The controller reports the timeout and requires reconnecting. Invalid front/right readings stop output; right distance above **100 cm** is treated as a lost wall and stops. Valid readings resume the selected mode unless Stop or the turn timeout has cancelled it. This handles local wall tracking and bounded left corners, not arbitrary mazes or searching around missing walls.

## Sensors and motor timing

Sensors alternate one ping every 70 ms (each sensor about 7 Hz), with 30 ms echo timeout. Echo timing uses interrupts on A1/A3 for UNO R4 WiFi. Accepted range is 2–400 cm. Samples expire after 350 ms. Sensor acquisition is separate from navigation and never blocks manual driving.

Wheel reversal inserts a 250 ms unpowered interval. Repeated requests do not restart that interval. Stops cancel pending output. Configuration changes also pause before later movement. Logs run at 2 Hz independently of control/sensor updates.

## Serial Monitor examples

After resetting the board, at 115200 baud / Newline:

```text
CONFIG 255 255 1 0 20 20
MODE FOLLOW
STOP
MODE WALL
STOP
```

For a battery tuning trial, a **starting example**, not a verified calibration, is `CONFIG 150 80 1 0 20 20`. Increase the minimum if the wheels stall; reduce output or gain if control overshoots. Gain/deadband/range/turn constants are grouped in `Robot.h`. Keep the current signal wiring; follow the battery wiring guide before changing the supply.
