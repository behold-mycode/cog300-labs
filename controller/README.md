# Robot remote control

The app only connects to the robot, selects a mode, starts/stops it, and provides keyboard manual control. Hardware configuration, calibration, route memory and navigation belong to the firmware. There are no calibration forms, tuning panels or settings/navigation web endpoints. Connecting does not overwrite the robot's settings.

1. Upload `Robot/Robot.ino` for Arduino UNO R4 WiFi when ready for hardware testing.
2. Close Arduino Serial Monitor, then open `Launch Robot Controller.command`.
3. Connect to the serial port, choose a mode and press **Start**. Selecting a mode alone stops motion.
4. Hold arrow keys or WASD for manual control; release to stop. Space, Escape and **Stop** stop all modes. Manual keys select manual mode.

Manual keyboard driving stops when the browser loses focus or its heartbeat expires. Autonomous line following continues during browser inactivity. Explicit desktop disconnect and USB serial fault recovery can stop a USB-controlled run. The onboard Wi-Fi controller operates independently.

Modes: Line following, Drive until tape, Stop at tape end, Object search, Line + wall maze (three sonars), Wheel calibration, Manual driving. Selection stops; Start runs. Connecting never overwrites calibration.

See [USB-Lab-Setup.md](../USB-Lab-Setup.md) for first calibration, required measurements, sonar limits and the current lab scope. Run `python3 controller/controller.py --demo` for a UI-only preview with no hardware access. The firmware is not yet physically validated.
