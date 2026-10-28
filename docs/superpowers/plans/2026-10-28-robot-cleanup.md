# Robot Cleanup Implementation Plan

> Execute natively with superpowers:executing-plans. User approved direct implementation.

**Goal:** Simplify keyboard control and provide maintainable object/wall following.
**Architecture:** Python loopback HTTP/serial bridge, browser keyboard interface, modular Arduino firmware with a single motor-output owner.
**Tech Stack:** Python 3.9+, pyserial 3.5, vanilla HTML/CSS/JS, Arduino C++ (UNO R4 WiFi).
**Spec:** ../specs/2026-10-28-robot-cleanup-design.md

## Global Constraints
Preserve pins D5–D10, A0–A3, D11; PWM maximum/default 255; separate manual/follow/wall modes; 500 ms firmware watchdog; 2 Hz readable telemetry; no Tk. Preserve untracked backups/diagnostics.

## Review Focus
- Delayed/out-of-order browser requests must not restart motion after stopping.
- Browser disappearance must stop autonomous as well as manual motion.
- Repeated reversal commands must not postpone motion indefinitely.
- Malformed or stale sensor/serial frames must not become valid drive instructions.
- Full-power configuration must not silently retain old lower autonomous caps.

## Task 1: Firmware
- [x] Replace old .ino helpers with Robot.h and Motor.cpp, Sensors.cpp, Navigation.cpp, Protocol.cpp; main .ino only setup/loop.
- [x] Implement protocol 4: HELLO, HEARTBEAT, DRIVE l r, MODE MANUAL/FOLLOW/WALL, CONFIG max min invertL invertR followCm wallCm, TEST l r ms, STOP, STATUS, LOG ON/OFF, HELP.
- [x] Implement 250 ms reversal settling, configurable power/inversion, bounded timed tests, 500 ms expired-session lockout.
- [x] Retain alternating interrupt sensor acquisition; follow deadband/reverse/lost-target stop; right-wall steering/front corner timeout.
- [x] Add host tests for these behaviors, malformed commands, rollover, sensor failure and rate-limited logs; compile for UNO R4 WiFi.

## Task 2: Controller
- [x] Replace Tk/state-machine/presets with controller.py HTTP server, robot_link.py serial worker, index.html and app.js.
- [x] Use authenticated loopback requests, single owner, increasing request sequence, bounded input/log buffers, browser and firmware liveness checks.
- [x] Add keyboard controls, mode buttons, settings, status, expandable log and download; blur/hidden/release stop.
- [x] Update launcher to explain Python/venv/dependency errors and support demo.
- [x] Test fake serial handshake, timeout, sequencing, disconnect, inversion/settings validation and browser UI in demo.

## Task 3: Delivery
- [x] Replace stale documentation with quick start, protocol/tuning and hardware checklist; preserve historical V1 reference.
- [x] Run host/Python/integration tests and Arduino compile, inspect diff, have an independent final review.
- [x] Commit implementation on lab-03/controller-cleanup; report GitHub branch/main divergence and hardware verification limits.

## Execution record

Implemented directly after user approval on 2026-10-28. No wiring changes.

Validation: 11 Python tests pass (including HTTP loopback, fake-serial failures,
and the actual firmware through a C++ host bridge); firmware, protocol and sensor
C++ suites pass; browser event tests pass in Node. UNO R4 WiFi compilation passes.
Safari demo checked connection, live readings, mode selection, Space cancellation,
and wall-follow cancellation when focus moves to the browser address bar.

Independent final review found one sensor timestamp race inherited from the old
acquisition loop. Added a clock-boundary regression (observed failing), then fixed
fresh-sample expiry (observed passing). No other material findings.

Ruling: Remove obsolete protocol/preset tests rather than keeping compatibility
paths; replace them with tests for firmware 4. Historical code remains in Git.
Ruling: Retain Python 3.9+ and auto-install only pyserial; removing Tk solves the
known missing GUI dependency without tying the interface to Web Serial browsers.

Hardware upload, wheel polarity, supply behavior and physical gain tuning remain
unverified. Local untracked backups/diagnostics are preserved.
