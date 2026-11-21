# COGS 300 Robot

This experimental branch focuses on autonomous line following. The intended tournament order is line following → wall maze → object detection. Wall navigation and object search are available as separate modes. Automatic handoff into object search is future work.

See [Wi-Fi control](WiFi-Control.md) for the onboard browser and [robot wiring](Robot-Wiring.md) for sensor connections. The November 18 driving checkpoint is tagged `working-robot-2026-11-18`. Earlier line-controller designs and implementation plans are available at that tag. Floor performance of the new controller is unverified.

## Download and set up

1. On GitHub, select **Code → Download ZIP** and extract the folder on your Mac. Keep the folders together.
2. Install Arduino IDE and the **Arduino UNO R4 Boards** package if needed. Open `Robot/Robot.ino`, choose **Arduino UNO R4 WiFi** and its USB port, then upload the sketch. The browser controller requires firmware 7.
3. Close Arduino Serial Monitor and Serial Plotter.
4. Double-click `Launch Robot Controller.command` in the extracted project folder. First launch needs internet to install its private runtime and dependencies.
5. In the browser select the robot's USB port and click **Connect**. Hold arrow keys or WASD to drive; release to stop. Space or Esc stops the robot.

Keep Terminal open while using the controller. For setup help and troubleshooting, see [`controller/README.md`](controller/README.md). Current wiring diagrams are in [`documentation/`](documentation/).

Firmware 7 adds an experimental **wall-maze controller** with encoder-regulated wheel speeds, footprint-aware pivot checks, Bayesian wall/opening estimates, and route recording/backtracking. Start with [Wall-Maze-Setup.md](Wall-Maze-Setup.md) for the new wiring, required measurements, tests and limits. **D11 is now reserved for the left sonar ECHO; disconnect the old LED.** Wall-maze behavior remains physically unvalidated.

For the servo-mounted front sonar, contour display and arena run controls, see the [object detection guide](Object-Detection-Design.md).

The controller exposes line following, both Lab 07 tape-stop demonstrations, object search, three-sonar line/wall maze, explicit wheel calibration, and manual driving. The firmware owns configuration. See the current USB guide above; older standalone and observation-first documents describe historical/opt-in behavior.

## Wi-Fi control

The UNO R4 can now host a browser for line start/stop, live readings and tuning while retaining USB uploads and the existing desktop controller. For wireless sketch uploads from the Mac, see [the one-time USB setup and Wi-Fi updater](WiFi-Control.md#wireless-firmware-updates-from-the-mac).
