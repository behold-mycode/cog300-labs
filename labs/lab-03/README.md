# Lab 3: Build your own bot

Arduino Uno R4 WiFi + L298N, with a Mac desktop controller for arrow-key driving and preset lab tests.

## Start here

1. Open [`src/MyRobot-V1/MyRobot-V1.ino`](src/MyRobot-V1/MyRobot-V1.ino) in Arduino IDE. Keep `Motor.ino` and `Serial.ino` in the same sketch folder.
2. Select **UNO R4 WiFi** and its current USB port, then upload. The robot starts stopped.
3. Close Serial Monitor/Plotter.
4. Double-click [`Launch Robot Controller.command`](controller/Launch%20Robot%20Controller.command), choose the USB port and Connect.
5. After the firmware check succeeds, raise the wheels and click **Enable controls**. Run **Wheels** first.

[Full controller instructions](controller/README.md)

## Simple controls

- Hold up/down arrows for forward/reverse.
- Hold left/right arrows to turn in place.
- Hold up plus left/right for a moving curve.
- Release the arrows to stop. Space or Esc cancels everything.
- Switching away from the controller stops/disables it.

On-screen buttons also support hold/release. Keyboard controls only operate while the app is focused and enabled. The firmware stops after 500 ms without controller updates.

## Lab tests

Use the buttons or F1–F5 (possibly Fn + F1 on a Mac): **Wheels**, **Straight**, **Turns**, **Spins**, **Full lab**. Each step has a timed motor command followed by a stopped pause. Full lab performs eight steps and includes the required forward/reverse, moving curves and in-place turns. Save the log and record actual observations; command completion is not proof of physical success.

[Detailed test procedure](test-plan.md) · [Results worksheet](test-results.csv)

## Wiring and calibration

| Uno R4 | L298N | Purpose |
| --- | --- | --- |
| D9 (PWM) | ENA | Left speed |
| D8 / D7 | IN1 / IN2 | Left direction |
| D10 (PWM) | ENB | Right speed |
| D6 / D5 | IN3 / IN4 | Right direction |
| GND | GND | Common ground |

Recorded motor wires: left black OUT1, left red OUT2; right red OUT3, right black OUT4. Confirm labels and actual wheel direction rather than relying on wire colours.

The left-wheel failure was traced by the user to a disconnected wire. Reconnect with power off, then retest at 100% trim on both wheels before assuming a motor fault. The GUI starts at 100/100 trim and left/right inversion off/on. The direction flags remain configurable because physical forward must be observed on this car. Changing calibration stops the robot and requires re-enabling controls.

Trim cannot exceed the hardware's maximum PWM of 255 or fix weak motor power. The USB-only course arrangement may be current-limited. See [power and wiring reference](programming-and-wiring.md) and [diagram](diagrams/uno-l298n-usb-wiring.svg). Keep the required enable/regulator jumper arrangement, and do not attach an external battery while motor supply is tied to Uno 5V.

## Code layout

- `MyRobot-V1.ino`: pins, startup defaults and main loop.
- `Motor.ino`: motor output, timed movement, test scheduling and live-drive refresh.
- `Serial.ino`: command parser, firmware handshake, settings, status and heartbeat timeout.
- `controller/`: native Tk desktop interface, control state machine, launcher and tests.
- `tests/`: simulated Arduino I/O and host tests of the actual sketch.

Standalone Serial Monitor remains available at 9600 baud / Newline. Use `FORWARD 180 500`, `BACKWARD 180 500`, `LEFT 180 500`, `RIGHT 180 500`, `STOP`, and `HELP`. A bare movement name uses the user's retained firmware defaults of PWM 255 / 3000 ms. GUI tests instead start at 500 ms and show their duration explicitly. The previous short/detailed commands remain accepted to avoid breaking saved procedures but are hidden from the main menu. The standalone legacy TEST_ALL runs ten steps; the simpler GUI Full lab runs eight.

## Validation

Firmware compiled for `arduino:renesas_uno:unor4wifi`. Host motor/parser tests and Python controller tests exercise stopping, input validation, direction and PWM calibration, live-drive updates, request IDs, the complete GUI preset sequence, and connection loss. Integration tests use a host build of the real sketch. Hardware upload and physical driving of this GUI version remain to be verified.

## Submission

Demonstrate forward/reverse, moving turns, spins, stopping, repeatable testing and calibration. Include the repository/usage instructions, current BOM, TinkerCAD/Fritzing diagram, team agreement and demonstration video as required by the [lab instructions](https://cogs-300.github.io/labs/lab03).
