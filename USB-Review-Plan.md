# USB robot review and implementation plan

Goal: prepare the existing line-to-wall solver for laptop-connected operation and installation of the remaining sensors. Requested scope takes precedence over the former standalone default. Lab 08 object search is explicitly included, with laptop start. The left sonar is deferred; Lab 07 must work with front/right sonars only. Side navigation range is capped at 40 cm.

Design: retain the existing Arduino controller, wheel feedback, graph navigation and desktop UI. Repair the integration that bypasses measured line recovery; require observed corridor evidence before changing guidance. Use a distinct measured floor-sensor offset. Compile standalone entry points out by default. Preserve sensor validity, motor reversal and USB lease enforcement.

Alternatives considered: rewriting navigation would discard working graph/replay coverage; simply deleting all stop conditions would hide missing measurements and broken observations. Repair the actual state transitions and remove obsolete assumptions instead.

- [x] Reproduce false line-to-wall transitions, broad-tape deadlock, and untethered serial movement in host firmware tests.
- [x] Repair supervisor/recovery and USB-only default in Robot/Standalone.cpp, Robot/Race.h, Robot/Robot.ino, Robot/Wireless.cpp and Robot/Protocol.cpp. Preserve opt-in standalone regression coverage.
- [x] Add floor-sensor geometry to HardwareProfile and automatic setup; invalidate saved calibration when changed.
- [x] Reproduce and fix first-edge encoder direction ordering and zero-target motor response.
- [x] Run complete host tests and UNO R4 WiFi compilation. Review resulting diff and report remaining hardware-dependent limits.
- [x] Provide shallow, current USB setup/review documentation and exact measurements needed.

Verification focus: missed sharp corners must search rather than switch guidance; broad tape must not deadlock; missing heartbeat must stop setup and navigation; sensor failures must remain visible; measured turns and braking must use the installed geometry. Synthetic tests do not establish traction, sonar beam behavior or physical course completion.
