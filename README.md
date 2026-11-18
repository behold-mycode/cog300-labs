# COGS 300 Robot

**Current build: laptop USB control with redesigned line following.** See [USB-Lab-Setup.md](USB-Lab-Setup.md). Line mode uses physical wheel-speed feedback, geometry-aware steering, and live tape capture, with the USB disconnect stop retained. See [Line-Control-Tuning.md](Line-Control-Tuning.md) for the rebuilt behavior, tuning commands, and verification limits. Optional battery auto-start is available by changing `ROBOT_LINE_ONLY` from `0` to `1` in `Robot/Robot.h`; see [Standalone-Line-Setup.md](Standalone-Line-Setup.md).


## Download and set up

1. On GitHub, select **Code → Download ZIP** and extract the folder on your Mac. Keep the folders together.
2. Install Arduino IDE and the **Arduino UNO R4 Boards** package if needed. Open `Robot/Robot.ino`, choose **Arduino UNO R4 WiFi** and its USB port, then upload the sketch. The browser controller requires firmware 7.
3. Close Arduino Serial Monitor and Serial Plotter.
4. Double-click `Launch Robot Controller.command` in the extracted project folder. First launch needs internet to install its private runtime and dependencies.
5. In the browser select the robot's USB port and click **Connect**. Hold arrow keys or WASD to drive; release to stop. Space or Esc stops the robot.

Keep Terminal open while using the controller. For setup help and troubleshooting, see [`controller/README.md`](controller/README.md). Current wiring diagrams are in [`documentation/`](documentation/).

Firmware 7 adds an experimental **wall-maze controller** with encoder-regulated wheel speeds, footprint-aware pivot checks, Bayesian wall/opening estimates, and route recording/backtracking. Start with [Wall-Maze-Setup.md](Wall-Maze-Setup.md) for the new wiring, required measurements, tests and limits. **D11 is now reserved for the left sonar ECHO; disconnect the old LED.** This revision is not uploaded or physically validated.

The controller exposes line following, both Lab 07 tape-stop demonstrations, object search, three-sonar line/wall maze, explicit wheel calibration, and manual driving. The firmware owns configuration. See the current USB guide above; older standalone and observation-first documents describe historical/opt-in behavior.
