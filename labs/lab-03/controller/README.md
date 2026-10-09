# Mac robot controller

A local desktop window with arrow-key driving and five repeatable lab-test presets. Uses USB serial; no browser, account, global keyboard monitoring, or Wi-Fi configuration is needed.

## Start

1. Open `../src/MyRobot-V1/MyRobot-V1.ino` in Arduino IDE, keeping its two other tabs together. Select **UNO R4 WiFi**, select its current USB port, and upload the updated firmware. The three-file course structure is preserved.
2. **Close Arduino Serial Monitor and Serial Plotter** so they do not compete with this controller for the serial port.
3. Double-click **Launch Robot Controller.command**. Python 3 with Tk is required; this Mac already has it. The launcher uses a local `.venv` and installs the pinned `pyserial` dependency on first use if necessary.
4. Select the board's `/dev/cu.usbmodem…` port and click **Connect**. Opening the port may reset the board. The controller waits for firmware version 2 and applies its visible calibration settings before enabling anything.
5. Support the wheels for initial tests. Click **Enable controls**. Use the focused controller window to drive or run tests.

No upload or motor movement is performed by the launcher. Firmware that does not support this controller is rejected with an upload instruction.

## Controls

| Input | Behavior |
| --- | --- |
| Hold ↑ | Forward |
| Hold ↓ | Reverse |
| Hold ← / → | Turn left/right in place |
| ↑ plus ← / → | Moving left/right curve |
| ↓ plus ← / → | Reverse with the corresponding side slower |
| Release all arrows | Stop/coast |
| Space or Esc | Stop and cancel the remaining preset |
| On-screen direction buttons | Hold to drive, release/leave button to stop |

Switching away from the window disables control and stops/cancels movement. Click Enable controls to resume. Editing fields also disables driving; arrow keys in an entry field edit its value rather than moving the car. Keys do not control the robot from other apps. After Space, release any held arrow before pressing it again.

## Preset tests and hotkeys

Use the buttons, or F1–F5 while controls are enabled. On a Mac, you may need **Fn + F1** etc. depending on the keyboard's function-key settings.

| Preset | Hotkey | Sequence |
| --- | --- | --- |
| Wheels | F1 | Left wheel forward, then right wheel forward |
| Straight | F2 | Forward, then reverse |
| Turns | F3 | Left moving curve, then right moving curve |
| Spins | F4 | Left in-place turn, then right in-place turn |
| Full lab | F5 | All four sequences above, eight steps total |

Each step uses the displayed PWM and test duration (initially 255 and 500 ms), followed by a 1-second stopped pause. Direction changes include a 250 ms unpowered settling period. The board enforces the test duration and acknowledges completion with a request ID; stale acknowledgments cannot advance a different test. Tests do not repeat automatically.

Completion means the command sequence finished, not that the physical robot passed. Record observations in the lab results sheet. Save log exports a timestamped text file through the normal Save dialog; saving stops/disables the robot first.

## Calibration

Start with 100% trim on both wheels, especially after fixing a loose wire. The initial inversion settings mirror the current sketch: left off, right on. Run Wheels and observe which way each wheel would propel the car. Toggle only the relevant Reverse direction checkbox if needed, then Apply. Apply stops/cancels motion and requires Enable controls again.

Trim scales that wheel's requested PWM, with a cap of 255. Example: left 80%, right 100%, requested 255 produces left 204 and right 255. This may make a motor stall if it falls below its starting threshold. A trim above 100% cannot increase a motor already at 255. A loose wire, friction or insufficient motor supply must be corrected separately.

Settings are sent at each connection and when Apply is clicked. They remain in the app during disconnect/reconnect but are not saved to disk; restarting the app restores the displayed defaults. The board also resets to its code defaults when reset or uploaded.

## Stopping and connection loss

The controller sends a heartbeat every 100 ms. During a controller session, the firmware stops and cancels the suite after 500 ms without a heartbeat or valid drive/test frame. It requires an explicit new connection handshake afterward; delayed frames cannot restart it. This also covers an application crash or freeze while the board remains powered. The host disables controls if acknowledgments stop arriving. Physical wheels may coast after drive power is removed; this is not a hardware emergency stop.

Serial Monitor still supports FORWARD, BACKWARD, LEFT, RIGHT and STOP, with optional PWM and duration for movements, plus TEST_ALL, P, M and HELP. LEFT/RIGHT mean in-place turns; moving curves are part of the GUI test presets or diagonal arrow controls. Older detailed aliases remain accepted for compatibility but are omitted from the main help menu. After a GUI watchdog expiry, reset the board before using standalone Serial Monitor commands; its desktop session is intentionally locked against stale movement.

## Verification and demo

The GUI includes a clearly labeled demo mode which never opens serial hardware:

```sh
./'Launch Robot Controller.command' --demo
```

Run software checks from the repository root:

```sh
labs/lab-03/controller/.venv/bin/python -m unittest discover -s labs/lab-03/controller -v
c++ -std=c++11 -Wall -Wextra -Werror -I labs/lab-03/tests labs/lab-03/tests/test_robot.cpp -o /tmp/cog300-test-robot
/tmp/cog300-test-robot
```

Tests include a host build of the actual Arduino sketch connected to the Python control state machine. They exercise the firmware handshake, live drive refresh, the entire GUI lab suite, release/stop/cancel, trim/direction configuration, and link-loss stopping. They do not verify real wiring, physical motion, speed, or USB motor-current capacity.
