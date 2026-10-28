# Hardware verification checklist

Software tests are not physical results. Record observed behavior and configuration in `docs/test-results.csv`; all new hardware trials remain NOT TESTED until performed.

## 1. Pins, wheel polarity and power

Keep current [wiring](front-right-wiring.md), secure loose wires, and raise the wheels. Upload the complete V2 sketch. Use Serial Monitor at **115200 baud / Newline**, after resetting the board (no GUI session).

Send one command at a time and observe the indicated wheel. Commands stop after 500 ms of actual output. `STOP` cancels immediately.

| Trial | Command |
| --- | --- |
| Left forward / reverse | `TEST 255 0 500` / `TEST -255 0 500` |
| Right forward / reverse | `TEST 0 255 500` / `TEST 0 -255 500` |
| Forward / reverse | `TEST 255 255 500` / `TEST -255 -255 500` |
| Moving left / right | `TEST 127 255 500` / `TEST 255 127 500` |
| Spin left / right | `TEST -255 255 500` / `TEST 255 -255 500` |

`CONFIG 255 255 0 1 20 20` restores full power and the current inversion defaults. Change only the corresponding inversion flag if that wheel is backward. Reversal has a 250 ms unpowered interval. Verify on the raised car before running on the floor.

## 2. Keyboard controller

Close Serial Monitor, launch the controller, select the port and Connect. No Enable step is needed.

- Hold/release each arrow or WASD key. Verify release stops, diagonals curve and opposing directions stop.
- While driving, Space/Esc must stop and require a fresh press.
- Switch windows/tabs and open/edit Settings: motion must stop.
- Apply wheel inversion/power changes while stopped; verify the new output.
- Close the page or interrupt Python: verify the browser/firmware watchdog stops the wheels. Reconnect must not resume prior movement.

## 3. Sensors and modes

Place a broad target in front and a flat wall to the right. Compare displayed distances with a ruler. Disconnect/misalign one sensor at a time; its reading must become unavailable, without corrupting the other sensor.

- Follow object: test farther than target, deadband and closer than target. Verify both wheels move together, including reverse when too close. Move target out of range: output stops.
- Follow wall: test too far/too close to wall. Verify steering direction. Bring a front obstruction within 20 cm: left turn; clearance ≥25 cm: resume wall tracking.
- Keep front blocked: stop and exit after 2.5 seconds. Missing wall or invalid required sensor: hold stopped.
- Space/Esc and window switching cancel both autonomous modes.
- Repeat after reducing maximum and minimum PWM for battery trials, using the [power guide](programming-and-wiring.md) before changing wiring.

Record oscillation, wheel stalls, overshoot, sensor dropouts and actual stop distance. Tune gains/power based on measurements. Do not mark a physical trial passed merely because TEST_DONE or a software test appears.
