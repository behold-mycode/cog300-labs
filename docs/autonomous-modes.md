# Object following and right-wall following

Choose a mode in the browser sidebar. Space/Esc, Stop, a new driving key, settings edits or connection loss cancels autonomous control. Both modes use the current power setting and calibrated wheel inversion. Defaults are in `Robot.h`; control logic is in `Navigation.cpp`.

## Follow object

Only the front sensor affects motion. Default target: **20 cm**, deadband: **±2 cm**.

- Too far: both wheels forward.
- Too close: both wheels reverse.
- Inside the deadband: stop.
- Invalid/stale reading or object farther than **150 cm**: hold stopped.

The command magnitude is `clamp(6 × abs(distance − target), minimumPWM, maximumPWM)` outside the deadband. Battery defaults are maximum 255 and minimum 255 PWM. A minimum of zero removes the motor-starting floor. At full output the car may overshoot/oscillate; tune on the real robot using a broad flat target, measured distance and short trials. No turning, object identification or rear sensing is provided.

## Follow right wall — simplified controller

Target: **15 cm** from the right sensor. Maximum PWM: **255**. Normal motion uses one command combining side-wall PD with front-obstacle avoidance:

```text
side_error = right_cm - target_cm
side = clamp(14 * side_error + 12 * filtered_right_rate, -limit, limit)
front_avoidance = 12 * max(0, 55 - front_cm)
steering = clamp(side - front_avoidance, -limit, limit)
```

Positive steering slows the right wheel; negative steering slows the left wheel. The outer wheel keeps the configured maximum. As a front obstacle gets closer, the leftward term grows continuously. Once front clearance returns, the side loop has immediate control in either direction. There is no inferred heading, wall-plane assumption, timed alignment phase, corner-clearance ratio, or minimum turn duration.

The derivative updates only with a new side measurement and uses its acquisition timestamp. Limit measured rate to ±50 cm/s, then filter with coefficient 0.65 (less delay than the previous 0.4). A held observation does not repeatedly recompute D. Short invalid intervals can retain the last good distance for less than 300 ms; an expired observation stops motion.

### Motor limits and calibration

`limit = maximumPWM - WALL_INNER_PWM`, floored at zero. `WALL_INNER_PWM` is provisionally **90**, taken from the earlier smooth corner output; it is **not a measured battery-powered running threshold**. Both wheels receive positive forward commands during normal navigation. Object-follow minimum PWM does not control wall steering.

The previous need for PWM 255 on the 5 V supply does not establish the battery-powered minimum. Before tuning gains further, use the controller's manual drive/standalone `TEST` commands to measure both wheels under floor load. In Serial Monitor after reset (controller disconnected), `TEST 120 120 500` commands a half-second straight test; test several values and record the lowest reliably rolling PWM for each wheel. Choose a wall inner limit above the higher running threshold. Check starting behavior separately. If the limit is too near maximum there is little steering authority; that is a motor/power constraint, not something a larger PD gain can solve. Actual testing requires the car on its driving surface with room to move.

### No-return and invalid measurements

The HC-SR04 can report no obstacle with a completed approximately 38 ms pulse ([supplier datasheet](https://www.elecrow.com/download/HC_SR04%20Datasheet.pdf)). Acquisition now waits up to 45 ms so this pulse can finish. Completed pulses longer than the 4 m measurement range and up to 40 ms are marked `RANGE_NO_RETURN`, not a fabricated distance. Too-short pulses, an echo line high before triggering, and unfinished/no-response pings are invalid. Samples expire independently.

With a fresh front no-return and a valid/recent right wall, tracking continues without a front repulsion term (`TRACKING_NO_FRONT_ECHO`). A front fault still stops after the bounded last-good allowance. A side no-return with a measured front distance starts bounded search; missing both usable directions stops (`NO_WALL_RETURN`). Raw DATA distance remains -1 for non-range results. This classification describes observed pulses, not a guarantee that hardware is healthy; a module that never produces a completed no-return pulse still stops as invalid.

### Recovery

A measured front distance at or below 10 cm triggers a 250 ms commanded backup and an escape spin until front clearance reaches 55 cm or a fresh no-return arrives. These are emergency actions, separate from ordinary forward steering. Each emergency phase has a 2.5 s limit. Front distance remaining at or below 20 cm for 2.5 s also stops the robot. Searching for a side wall beyond 100 cm or a side no-return is limited to four seconds. Stop and the connection watchdog remain active.

### Validation limits

The kinematic test exercises three wheel tracks (12, 14, 16 cm) at three idealized speeds (20, 30, 40 cm/s), sensor offsets 8/6 cm, and alternating 70 ms samples. It checks maximum heading throughout the corner (under 110 degrees), body clearance, and 10–20 cm wall distance after y=70 cm. It checks model wheel motion, not only positive PWM commands. The default model has linear PWM speed and no slip/inertia; an optional deadband argument demonstrates how a stalled inner wheel violates the motion requirement. This remains a software model, not battery calibration or a guarantee of exact 90-degree turns.

## Sensors and motor timing

Sensors alternate one ping every 70 ms (each sensor about 7 Hz), with 45 ms echo timeout. Echo timing uses interrupts on A1/A3 for UNO R4 WiFi. Accepted range is 2–400 cm. Samples expire after 350 ms. Sensor acquisition is separate from navigation and never blocks manual driving.

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
