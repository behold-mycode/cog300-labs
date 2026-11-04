# COGS 300 Robot

Arduino UNO R4 WiFi robot with an L298N motor driver, front and right HC-SR04 sensors, and a browser-based USB controller.

## Get started

1. Download this repository using **Code → Download ZIP**, then extract it. Keep the `Robot/` folder together.
2. Install Arduino IDE and the **Arduino UNO R4 Boards** package. Open [`Robot/Robot.ino`](Robot/Robot.ino), select **Arduino UNO R4 WiFi** and the robot's USB port, then upload.
3. Close Arduino Serial Monitor and Plotter.
4. On macOS, double-click [`Launch Robot Controller.command`](Launch%20Robot%20Controller.command). The first launch needs internet to set up its local runtime.
5. Select the robot's USB port and click **Connect**. Use arrow keys or WASD to drive; release to stop. Space or Esc stops movement.

See [controller setup and troubleshooting](controller/README.md) if the controller does not start or connect.

## Controls and settings

The controller offers manual driving, object following, and right-wall following. Open **Settings** to set maximum PWM, minimum follow PWM, wheel direction, and target distances. Maximum PWM defaults to 255; reduce it to limit overall motor output. Settings last only while the controller server is running.

## Wiring

See [wiring diagrams and pin connections](docs/front-right-wiring.md). The robot uses:

- D5–D7 for the right motor and D8–D10 for the left motor through the L298N.
- A0/A1 for the front sensor and A2/A3 for the right sensor.
- D11 for the green status LED.
- A common ground between the Arduino, motor driver, and sensors.

## Repository contents

- `Robot/` — Arduino firmware; open `Robot.ino` to upload.
- `controller/` — browser interface and USB serial bridge.
- `wiring/` — wiring diagrams.
- `docs/` — setup, motion behavior, protocol, and hardware checks.

The firmware and controller must be used together (protocol 4). The robot's tested motor direction settings are motor A inverted and motor B normal. Maximum PWM is an output limit, not a guarantee of physical speed; battery and load affect motor performance.
