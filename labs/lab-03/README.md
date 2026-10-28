# Lab 3 robot — current working version

UNO R4 WiFi + L298N + front/right HC-SR04 sensors. **Use `src/MyRobot-V2` and the browser controller together (firmware protocol 4).** V1 is historical and is not compatible with this controller.

## Start

1. Open [MyRobot-V2.ino](src/MyRobot-V2/MyRobot-V2.ino) in Arduino IDE. Keep **all files in that folder** together (`.ino`, `.h`, `.cpp`). Select UNO R4 WiFi and upload.
2. Close Arduino Serial Monitor/Plotter.
3. Double-click [Launch Robot Controller.command](controller/Launch%20Robot%20Controller.command). Python 3.9+ is required; the launcher installs the sole Python dependency, pyserial, in `.venv`. **Tk is no longer needed.**
4. Select the USB port, then **Connect**. After the firmware check, hold arrow keys or WASD to drive. Release to stop. Space/Esc stops every mode.

[Controller setup/troubleshooting](controller/README.md) · [Modes and tuning](autonomous-modes.md) · [Serial protocol](protocol.md) · [Hardware checks](test-plan.md)

## Controls

- Up/W: forward; Down/S: reverse.
- Left/A or Right/D alone: turn in place. Combine with forward/reverse for a curve.
- Side buttons: Manual, Follow object, Follow right wall.
- Settings: maximum PWM, minimum object-follow PWM, independent wheel inversion, and separate target distances.
- Leaving the window, opening/editing settings, or losing the connection stops movement. New input is required to resume.

Maximum and minimum moving PWM default to **255**. Default inversion is left off/right on. Settings last while the Python server runs and are reapplied on connection; firmware reset restores defaults. Maximum PWM is a duty-cycle limit, not a guarantee of torque or speed. Power-supply and mechanical limitations still require hardware testing.

## Wiring — unchanged by this cleanup

| UNO R4 pin | Connection |
| --- | --- |
| D5 / D6 / D7 | L298N ENA / IN1 / IN2 (left) |
| D8 / D9 / D10 | L298N IN3 / IN4 / ENB (right) |
| A0 / A1 | Front TRIG / ECHO |
| A2 / A3 | Right TRIG / ECHO |
| D11 | Green LED via resistor |
| GND | Common motor driver and sensor ground |

[Full diagrams and battery transition](front-right-wiring.md). Green indicates a valid front measurement. Manual driving is not blocked by the distance sensors; autonomous modes stop for invalid required measurements.

## Code layout

| File | Responsibility |
| --- | --- |
| `Robot.h` | Pins, configuration, module interfaces |
| `MyRobot-V2.ino` | Startup and cooperative loop |
| `Motor.cpp` | PWM, inversion, reversal pause, bounded timed tests |
| `Sensors.cpp` | Alternating interrupt-based echo measurements and validity |
| `Navigation.cpp` | Object-distance and right-wall control |
| `Protocol.cpp` | Strict command parsing, watchdog, 2 Hz telemetry |
| `controller/robot_link.py` | Serial connection, browser ownership, liveness |
| `controller/controller.py` | Loopback HTTP server and launcher entry point |
| `controller/index.html`, `app.js`, `style.css` | Minimal browser UI |

Legacy command aliases, duplicated preset suites, trim multipliers and old obstacle-stop paths were removed. Serial Monitor uses **115200 baud / Newline**. Use `HELP`, `STATUS`, `LOG ON/OFF`, and `TEST left right duration_ms` for standalone testing. See [protocol.md](protocol.md).

## Software checks

From the repository root:

```sh
python3 -m unittest discover -s labs/lab-03/controller -v
node labs/lab-03/controller/test_app.js
c++ -std=c++11 -Wall -Wextra -Werror -I labs/lab-03/tests labs/lab-03/tests/test_firmware.cpp -o /tmp/cog300-firmware
/tmp/cog300-firmware
c++ -std=c++11 -Wall -Wextra -Werror -I labs/lab-03/tests labs/lab-03/tests/test_sensors.cpp -o /tmp/cog300-sensors
/tmp/cog300-sensors
c++ -std=c++11 -Wall -Wextra -Werror -I labs/lab-03/tests labs/lab-03/tests/test_protocol.cpp -o /tmp/cog300-protocol
/tmp/cog300-protocol
arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi labs/lab-03/src/MyRobot-V2
```

Python tests use the standard library, fake serial and host-compiled real firmware; HTTP tests bind loopback. No robot is moved. Software checks cannot verify wiring, wheel directions, available motor power, or physical control tuning.

## Course and repository status

[Lab 3](https://cogs-300.github.io/labs/lab03) asks for forward/reverse, moving turns, spins and repeatable testing, plus the diagram/BOM/team agreement/video. The Serial Monitor test commands support that demonstration. [Lab 4](https://cogs-300.github.io/labs/lab04) also requires encoders; encoder acquisition is outside this ultrasonic-controller cleanup.

On 2026-10-28, the previous working commit `a046510` was confirmed pushed to `origin/lab-03/proportional-following`, but seven commits were not in `origin/main`. Open PRs #3–#7 contain the earlier iterations. This cleanup is based on that working branch, not the older `main`. Backups and diagnostics already present locally are preserved and remain untracked.
