# COGS 300 Robot

UNO R4 WiFi + L298N + front/right HC-SR04 sensors. **Use `Robot` and the browser controller together (firmware protocol 4).** Older code is in `archive/` and is not compatible with this controller.

## Start

1. Download this repository with **Code → Download ZIP** and extract it into a writable folder, or clone it. No Git installation is needed for the ZIP route.
2. To program the robot, install [Arduino IDE](https://www.arduino.cc/en/software/). In Boards Manager install **Arduino UNO R4 Boards**, then open [Robot.ino](Robot/Robot.ino). Keep every file in `Robot/` together. Select **Arduino UNO R4 WiFi** and the USB port, then Upload. If this version is already on the robot, skip this step.
3. Close Arduino Serial Monitor/Plotter.
4. Double-click [Launch Robot Controller.command](Launch%20Robot%20Controller.command). **No preinstalled Python, pip, Homebrew, Node, or Xcode is required to run the controller.** The first launch downloads private tooling/Python as needed and pyserial. Internet is required for first setup; later launches reuse the local installation.
5. Select the USB port, then **Connect**. After the firmware check, hold arrow keys or WASD to drive. Release to stop. Space/Esc stops every mode.

If macOS blocks double-clicking, open Terminal, type `zsh `, drag the launcher into the window, and press Return. Do not run from inside the ZIP. Keep Terminal open while driving. See [Mac setup](controller/README.md) for details and manual installation alternatives.

[Controller setup/troubleshooting](controller/README.md) · [Modes and tuning](docs/autonomous-modes.md) · [Serial protocol](docs/protocol.md) · [Hardware checks](docs/test-plan.md)

## Controls

- Up/W: forward; Down/S: reverse.
- Left/A or Right/D alone: turn in place. Combine with forward/reverse for a curve.
- Side buttons: Manual, Follow object, Follow right wall.
- Settings: maximum PWM, minimum object-follow PWM, independent wheel inversion, and separate target distances.
- Leaving the window, opening/editing settings, or losing the connection stops movement. New input is required to resume.

Battery defaults are **255 maximum PWM**, **255 minimum follow PWM**, and **15 cm wall distance**. These are starting values for floor calibration. Default inversion is motor A on / motor B off, matching the user-tested forward direction. Channel A drives the right wheel and B the left. Settings last while the Python server runs and are reapplied on connection; firmware reset restores defaults. Maximum PWM is a duty-cycle limit, not a guarantee of torque or speed. Power-supply and mechanical limitations still require hardware testing.

## Wiring — unchanged by this cleanup

| UNO R4 pin | Connection |
| --- | --- |
| D5 / D6 / D7 | L298N ENA / IN1 / IN2 (right wheel) |
| D8 / D9 / D10 | L298N IN3 / IN4 / ENB (left wheel) |
| A0 / A1 | Front TRIG / ECHO |
| A2 / A3 | Right TRIG / ECHO |
| D11 | Green LED via resistor |
| GND | Common motor driver and sensor ground |

[Full diagrams and battery transition](docs/front-right-wiring.md). Green indicates a valid front measurement. Manual driving is not blocked by the distance sensors; autonomous modes stop for invalid required measurements.

## Folder layout

```text
cog300-labs/
├── Launch Robot Controller.command
├── README.md
├── Robot/        Arduino sketch: open Robot.ino
├── controller/   Browser interface and Python serial bridge
├── wiring/       Wiring diagrams
├── docs/         Setup, modes, protocol and hardware checks
├── tests/        Firmware tests
├── archive/      Older sketch and preserved local backups
└── templates/    Reusable lab write-up
```

## Code layout

| File | Responsibility |
| --- | --- |
| `Robot/Robot.h` | Pins, configuration, module interfaces |
| `Robot/Robot.ino` | Startup and cooperative loop |
| `Robot/Motor.cpp` | PWM, inversion, reversal pause, bounded timed tests |
| `Robot/Sensors.cpp` | Alternating interrupt-based echo measurements and validity |
| `Robot/Navigation.cpp` | Object-distance and right-wall control |
| `Robot/Protocol.cpp` | Strict command parsing, watchdog, 10 Hz telemetry |
| `controller/robot_link.py` | Serial connection, browser ownership, liveness |
| `controller/controller.py` | Loopback HTTP server and launcher entry point |
| `controller/index.html`, `app.js`, `style.css` | Minimal browser UI |

Legacy command aliases, duplicated preset suites, trim multipliers and old obstacle-stop paths were removed. Serial Monitor uses **115200 baud / Newline**. Use `HELP`, `STATUS`, `LOG ON/OFF`, and `TEST left right duration_ms` for standalone testing. See [protocol.md](docs/protocol.md).

## Software checks

From the repository root:

```sh
python3 -m unittest discover -s controller -v
node controller/test_app.js
c++ -std=c++11 -Wall -Wextra -Werror -I tests tests/test_firmware.cpp -o /tmp/cog300-firmware
/tmp/cog300-firmware
c++ -std=c++11 -Wall -Wextra -Werror -I tests tests/test_sensors.cpp -o /tmp/cog300-sensors
/tmp/cog300-sensors
c++ -std=c++11 -Wall -Wextra -Werror -I tests tests/test_protocol.cpp -o /tmp/cog300-protocol
/tmp/cog300-protocol
c++ -std=c++11 -Wall -Wextra -Werror -I tests tests/test_wall.cpp -o /tmp/cog300-wall
/tmp/cog300-wall
c++ -std=c++11 -Wall -Wextra -Werror -I tests tests/test_corner_motion.cpp -o /tmp/cog300-corner
/tmp/cog300-corner
arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi Robot
```

Python tests use the standard library, fake serial and host-compiled real firmware; HTTP tests bind loopback. No robot is moved. Software checks cannot verify wiring, wheel directions, available motor power, or physical control tuning.

## Course and repository status

[Lab 3](https://cogs-300.github.io/labs/lab03) asks for forward/reverse, moving turns, spins and repeatable testing, plus the diagram/BOM/team agreement/video. The Serial Monitor test commands support that demonstration. [Lab 4](https://cogs-300.github.io/labs/lab04) also requires encoders; encoder acquisition is outside this ultrasonic-controller cleanup.

The complete software cleanup and earlier working changes were merged into `main`
in [PR #8](https://github.com/behold-mycode/cog300-labs/pull/8). The active sketch is
now `Robot/Robot.ino`; the launcher is at the project root. Local backups and
old diagnostics are preserved under `archive/` and remain untracked.

## Saved wall-following baseline

`wall-following-baseline-2026-10-29` preserves the mostly working hardware version before further tuning: full power (255), a 12 cm target, P wall steering, bounded search, close-front backup, and committed left turns. The user reports it mostly works; consistent 10–15 cm clearance remains a floor-test goal. With minimum and maximum both 255, corner-spin power is saturated at full output. Wiring and wheel polarity are specific to this robot and must be checked on another build.

### Checkpoint 2 — smooth PD tracking and earlier corner exit

Tag: `checkpoint-2-pd-wall-following-2026-10-30`. This preserves the uploaded tuning version: 15 cm right-wall target, full outer-wheel power, timestamp-based PD tracking, forward left arcs, bounded echo-loss handling, and a corner-clearance ratio reduced from 4 to 2.5. The user reported smooth motion with the preceding ratio but approximately 120-degree turns; the reduced ratio is uploaded and software-tested, with its physical angle still to be confirmed. Sensor-based approach/heading estimation and continuously adapted corner geometry are the next development direction, not features claimed by this checkpoint.
