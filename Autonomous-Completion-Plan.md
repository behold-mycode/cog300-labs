# Autonomous Startup Completion Implementation Plan

> **For agentic workers:** Use superpowers:executing-plans inline. User has authorized implementation and explicitly rejected further app configuration workflows.

**Goal:** A firmware-owned boot/start path from fixed physical dimensions through bounded automatic motion commissioning to the single maze solver, without app configuration commands.

**Architecture:** A compiled HardwareProfile supplies only unknowable physical dimensions and motor polarity. A nonblocking commissioning state machine runs after explicit Start, checks fresh sonar/IR, measures encoder wheel speeds and coast, derives internal motion settings, and saves the completed profile. Existing solver and graph remain; physical stop and USB lease apply during commissioning.

**Tech Stack:** UNO R4 WiFi Arduino C++14, EEPROM, existing host firmware simulation.

**Spec:** User's current requests: standalone operation, line priority/wall fallback, route memory/dead ends, controlled turn footprint, minimal remote app. No goal detector.

## Global constraints
- No invented wheel/chassis measurements; missing facts explicitly named in firmware status and guide.
- No physical motor commands/upload during development.
- Existing working tree retained; no app configuration UI.
- No claims of physical validation from simulation.

## Review focus
- Stop/disconnect/reset must cancel commissioning; no deferred automatic restart.
- Encoder absent/reversed, chassis drift or stale/blocked sonar must terminate calibration.
- Saved data must be bound to the compiled geometry; changing the car cannot reuse stale calibration.
- Startup measurements must be from distinct fresh samples and bounded travel/time.
- End-to-end test must use setup/start, not manually inject all runtime calibration commands.

## Tasks
- [x] Add HardwareProfile.h: physical dimensions validation/fingerprint and robot-owned initialization. Test absent profile fails without motion, valid profile enables known sensor/encoder wiring and derived tolerances.
- [x] Add AutoSetup.h/.cpp: explicit-start surface check, bounded ramp/measurement/coast, internally derived speed/braking/controller defaults. Test successful synthetic commissioning and failure/stop cases first.
- [x] Integrate setup/start/stop/persistence and solver without resetting autonomous lease or route state unexpectedly. Test fresh EEPROM startup with synthetic geometry and actual motor/encoder feedback.
- [x] Consolidate pin and power wiring in Robot-Wiring.md and SVG. Label the old diagrams superseded; name exact missing measurements and power status.
- [x] Run host suite, Arduino compile and fresh independent review. Fix material findings before reporting status.

## Execution record
- Physical wheel/chassis dimensions were not found in repository documentation; LM393 does not specify disc edge count or wheel diameter. A profile with unset dimensions must remain visibly uncommissioned rather than guess.

- Fresh firmware test now exercises setup/start → automatic measurement → tape → walls → dead end → return, with the production 70ms-or-slower stagger and full synthetic chassis collision checks.
- Independent review found zero-ID persistence bypass and missing motor-polarity fingerprint binding. Both fixed; regression tests reject invalid-profile restore/start and changed-polarity restores.
- Added USB timeout cancellation tests in RAMP, MEASURE and COAST, plus stale individual sonar while the control loop remains timely.
- Ruling: do not add a blind full-PWM kick for low-speed stiction. It could violate the planner's stopping assumption; nonlinear low-speed response must be checked physically before treating calibration as adequate.
- Fixed controller handshake: an early firmware diagnostic must not replace the handshake state before HELLO completes.

- Final verification: 26 C++ executables plus replay variants, 25 Python tests and browser event tests passed. UNO R4 WiFi build: 137252 bytes flash, 16336 bytes static RAM. No upload or physical motion performed.
