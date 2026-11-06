# COGS 300 Robot

## Download and set up

1. On GitHub, select **Code → Download ZIP** and extract the folder on your Mac. Keep the folders together.
2. Install Arduino IDE and the **Arduino UNO R4 Boards** package if needed. Open `Robot/Robot.ino`, choose **Arduino UNO R4 WiFi** and its USB port, then upload the sketch. The browser controller requires firmware 6.
3. Close Arduino Serial Monitor and Serial Plotter.
4. Double-click `Launch Robot Controller.command` in the extracted project folder. First launch needs internet to install its private runtime and dependencies.
5. In the browser, select the robot's USB port and click **Connect**. Hold arrow keys or WASD to drive; release to stop. Space or Esc stops the robot.

Keep Terminal open while using the controller. For setup help and troubleshooting, see [`controller/README.md`](controller/README.md). Current wiring diagrams are in [`documentation/`](documentation/).

Firmware 6 adds experimental observation-based tape navigation, measured braking calibration, observation capture and route-checkpoint export. Start with the [tape and encoder setup guide](documentation/observation-first.md); encoder type and chassis geometry must be measured before MAZE is available. Basic tape/lab modes work without encoders. [Current verification and limits](documentation/observation-first.md#evidence-and-weaknesses).
