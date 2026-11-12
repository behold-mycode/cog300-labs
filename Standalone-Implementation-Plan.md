# Standalone race and wireless control implementation plan

Goal: battery-powered line-to-wall-maze run with physical start/stop, persistent calibration and wireless setup/start. No firmware upload or power wiring changes during development.

- Implement versioned, checksummed, redundant calibration storage; boot stopped and never restore motion/map state.
- Add debounced D1-to-GND momentary start/stop input with deliberate start and immediate stop semantics; preserve USB watchdog behavior outside explicitly standalone runs.
- Implement a course supervisor for regulated line following, measured line-loss recovery and evidence-based handoff to WALL_MAZE. No goal recognition: user stops and picks up the robot.
- Implement local access-point setup/start interface using WiFiS3, with password configuration. Avoid potentially blocking radio operations while the robot is moving; physical stop remains available throughout a run.
- Add host tests for corrupt/interrupted saves, reboot behavior, button bounce/held-at-boot, USB disconnect during explicitly autonomous vs tethered sessions, transition false positives and finish conditions once specified.
- Compile UNO R4 firmware, run regressions and document battery/pin/setup requirements and unvalidated physical limits.

Resolved: robot-hosted Wi-Fi access point, no goal recognition. Four rechargeable AAs currently power the motor board; independent regulated Arduino power remains a physical setup prerequisite. Existing qa/PR remains the development branch; no merge.
