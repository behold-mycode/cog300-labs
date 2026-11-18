# Line-control implementation plan

> Use superpowers:executing-plans in this session. User approved implementation and upload.

**Goal:** replace the patched line path with forward-progress control and meaningful encoder feedback.
**Architecture:** line behavior returns explicit wheel ratios; continuous wheel control balances measured travel. Corner approach uses measured sensor geometry; recovery advances between finite sweeps.
**Tech stack:** C++14/Arduino UNO R4, Python controller, vanilla browser UI.
**Spec:** Line-Control-Redesign.md

## Constraints

D4/D2/D3 left/centre/right, LOW tape; 130 mm track, 150/145 mm sensor offsets,
50 mm outer lateral spacing, 5.105 mm/edge. Strong continuous drive, no pulses,
USB stops retained, no sonar requirement or automatic commissioning on Line Start.

## Review focus

- Full-chain motor dynamics and coarse encoder counts, including opposite wheel response.
- Reacquisition followed by immediate loss must not restart the search budget.
- Sensor loss must produce measured translation between repeated search locations.
- Stop/disconnect during pivot and search must cancel every motor demand.
- Missing calibration must not trigger unannounced movement or require wall geometry.

## Tasks

- [x] Add `Robot/LineFollower.h`: `Input` (pose/time/mask), `Command` (signed wheel ratios), `Follower::begin/step/stop`, explicit Follow/Corner/Search/Stopped.
- [x] Add `Robot/LineWheelControl.h`: bounded continuous PWM with per-wheel travel balance, actual zero stays zero. Unit tests for mismatch/turn signs/saturation/reset.
- [x] Build whole-chain replay in `tests/test_line_control_replay.cpp`: use production motor service, quantized encoders, deadband/asymmetry/coast and actual sensor positions. Reproduce old controller shortcomings and exercise both signs of bends, recovery progress and STOP.
- [x] Route LINE and stop modes through new components in Navigation/Line; retire old LineDrive normalization and old LINE-specific Race patches; keep combined RACE/object regression coverage.
- [x] Add current wheel/PWM/phase telemetry to controller; expose explicit line trim configuration and document commissioning without auto-motion.
- [ ] Run full suite and UNO R4 compilation. Independent branch review, fix material findings, commit/push and upload. Verify handshake and reconnect stopped.

## Execution decisions

- Encoder travel balance is chosen over a low-speed PI loop: preserves the known usable continuous torque range and avoids claiming an unattainable slow target. Wheel ratios are explicit; infeasible sharp curvature uses a stopped inner wheel or pivot.
- Fixed high-frequency wheel service and count-based travel balance do not need a low-speed velocity estimate to maintain straightness. Existing reported speed remains a 100 ms estimate; it must be labelled approximate rather than used as precision turn completion.
- Coast stopping is retained; no active brake/current assumptions are introduced.
- Whole-chain tests will determine thresholds and expose any deviation from proposed geometry/search policy before upload.

## Scope change and execution record

User subsequently requested automatic battery-powered line-only operation. This
supersedes the initial USB-only constraint. Default `ROBOT_LINE_ONLY=1` starts
LINE after 1.5 seconds, uses installed LOW tape polarity and no USB lease, and
keeps translating/searching after loss. `=0` retains the laptop/Lab 08 profile.
A real encoder stall and deliberate Stop remain effective. Hardware upload is
pending: no USB serial device is currently enumerated.

Tasks 1–5 implemented. Task 6: host checks and UNO build passed before the battery
scope change; final battery checks/build and GitHub save are in progress.

Ruling: use encoder travel balance rather than low-speed PI — preserves usable
torque with coarse encoder edges; absolute floor speed still varies with load.
Ruling: retain the 250 ms reversal coast interval and use a provisional 400 ms
turn exit / 55 mm approach allowance — no verified active-brake current or floor
coast calibration exists; real inertia outside the model may still overshoot.
Ruling: keep combined RACE isolated and retain its tests — rewriting future wall
behavior would expand scope; its older ideal tests do not validate this follower.
Ruling: provide LINECFG over serial and visible wheel feedback, without adding a
moving friction-search routine — explicit commissioning remains available; the
230 PWM floor is provisional and needs a floor trial.
Ruling: default battery mode continuously searches; laptop mode retains bounded
search — follows the user's new power-cycle workflow; a lost battery-powered car
can keep moving until stopped or powered off.

Fresh review found and fixed: second sweep repeated the first direction under
conflicting cues; turn-exit travel could count as uninterrupted tape confirmation;
no-tape replay had an unrelated downstream success branch. Regression tests now
cover conflicting cues, interrupted confirmation and exclusive no-tape completion.
Battery-scope review found and fixed: HELLO/LINE could reinstate USB lease ownership;
first centre-tape acquisition was ignored during translated search. Both had
observed failing tests before their fixes.

Final verification: 38 C++ test programs, 26 Python tests and browser event checks
passed. UNO R4 compilation passed: 128164 bytes flash, 14768 bytes RAM. No hardware
upload or floor trial occurred; the robot USB port is absent. The desktop serial
integration explicitly builds `ROBOT_LINE_ONLY=0`; dedicated battery tests build
`=1` and check HELLO/start/no-heartbeat/Stop independently. No deferred review bugs.

Latest user steering: keep laptop USB as default until external board power is
available. `ROBOT_LINE_ONLY=0` is the shipped default; `=1` retains the tested
auto-start battery option. USB disconnect/heartbeat stopping remains active in
the default profile. This supersedes the earlier default-battery decision.
