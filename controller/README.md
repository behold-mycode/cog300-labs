# Robot remote control

The app only connects to the robot, selects a mode, starts/stops it, and provides keyboard manual control. Hardware configuration, calibration, route memory and navigation belong to the firmware. There are no calibration forms, tuning panels or settings/navigation web endpoints. Connecting does not overwrite the robot's settings.

1. Upload `Robot/Robot.ino` for Arduino UNO R4 WiFi when ready for hardware testing.
2. Close Arduino Serial Monitor, then open `Launch Robot Controller.command`.
3. Connect to the serial port, choose a mode and press **Start**. Selecting a mode alone stops motion.
4. Hold arrow keys or WASD for manual control; release to stop. Space, Escape and **Stop** stop all modes. Manual keys select manual mode.

Laptop-controlled operation retains its heartbeat stop: leaving the window, losing the browser/USB connection or disconnecting stops motion. The robot's physical button and onboard autonomous start remain separate from this tethered control session. Standalone running does not require the desktop app.

The two modes are **Maze solver** and **Manual driving**. Maze solver starts with line guidance when tape is present, otherwise wall navigation. Sustained tape loss switches to walls; stable tape reacquisition in a corridor switches back to line guidance without clearing the route map. A measured junction manoeuvre finishes before guidance changes. Each solver start creates a fresh route map. The robot reports why it stops; selecting Start does not bypass missing hardware setup or clearance checks.

Run `python3 controller/controller.py --demo` for a UI-only preview with no hardware access. Changes are host-tested; the latest firmware has not been uploaded or physically validated. See [Standalone-Setup.md](../Standalone-Setup.md) for development/hardware status.
