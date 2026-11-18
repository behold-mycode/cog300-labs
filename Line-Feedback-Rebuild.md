# Line feedback rebuild — design and implementation plan

> Execution: use superpowers:executing-plans inline, with test-first changes and a final independent review. User delegated all design decisions and requested implementation without another approval round.

**Goal:** continuous, fast, geometry-aware line following whose measured wheel motion and IR feedback remain coherent through sharp turns.
**Architecture:** one follower owns the motion objective; wheel targets are physical mm/s. A dedicated encoder velocity estimator and feed-forward/PI wheel servo execute them. IR observations remain live during approach, turn capture and exit. Recovery is bounded and moves to a new location. Existing USB Stop, disconnect and wheel-stall handling remain.
**Tech stack:** C++14, UNO R4 WiFi, existing L298 coast/reversal service, host firmware replay.
**Spec:** this document, plus the user's request in the review conversation.

## Decisions and constraints

- Use the existing 130 mm track, 150/145 mm sensor offsets, +/-50 mm lateral spacing, nominal 5.105 mm per edge. Dimensions describe ideal kinematics; IR corrects physical uncertainty.
- Replace dimensionless near-full-power normalization with explicit mm/s targets and feedback. Defaults are provisional, tunable, and require floor commissioning. Zero PWM floor is permitted; no hidden 230 floor.
- Retain bridge interlock initially; expose actual applied motor commands to the wheel controller so it cannot integrate error while waiting for a reversal.
- No unconditional timed exit, stale side latch, or corner approach that ignores a regained continuation. No exact-heading claim from single-channel encoders.
- Persistent sensor evidence and progress determine transitions; independent stop reasons remain observable.
- Keep line-specific code independent of sonar and wall/race controllers. USB mode remains default. Preserve existing uncommitted design-document edits.
- Work on dedicated branch `line-feedback-rebuild` in the existing checkout, keeping the project and review files easy to find.

## Review focus

- Both bend directions, opposite wheel mismatch, doubled coast, increased speed and quantized counts.
- Gap reacquisition, immediate opposite bends, leading-sensor capture followed by loss.
- Wheel servo saturation, changing steering demand, reversal waiting, zero floor, and restarting under friction.
- Progress watchdog versus spinning and missed tape; explicit Stop during every phase.
- Protocol compatibility, battery-only search and default USB disconnect.

## Task 1: Regression evidence

Files: `tests/test_line_feedback.cpp`, `tests/test_line_control_replay.cpp`.
- [x] Add and run failing tests for approach reacquisition, live exit steering, fresh side cues, and coast/speed sensitivity. Assertions describe course outcome and command direction, not exact old PWM.
- [x] Preserve the original replay scenarios and extend the physical model; do not relax course-completion assertions to make the new controller pass.

## Task 2: Measured wheel execution

Files: `Robot/LineWheelControl.h`, `Robot/LineFollower.h`, `Robot/Navigation.cpp`, `tests/test_line_wheel_control.cpp`.
Interface: `Command.left/right` in mm/s; `Input` includes pose, time, tape and measured wheel speeds/applied powers. `WheelControl` estimates velocity from a rolling encoder window and returns signed PWM.
- [x] Write failing tests for speed tracking under mismatched load, zero demand, saturation recovery and blocked reversal.
- [x] Implement continuous feed-forward/PI with anti-windup, no ratio-change resets, and configurable optional minimum. Track actual execution rather than requested actuation.
- [x] Run targeted wheel/firmware tests.

## Task 3: Geometry and live tape control

Files: `Robot/LineFollower.h`, `Robot/Navigation.cpp`, `tests/test_line_feedback.cpp`, `tests/test_line_recovery.cpp`.
Interface: `Follower.begin/step/stop`, observable FOLLOW/CORNER/SEARCH/STOPPED states and reason; one snapshot per navigation update.
- [x] Rebuild tracking, geometric approach, turn capture, and spatial search around current evidence and wheel feedback.
- [x] Use current side evidence, distinguish old incoming tape during a sweep, cancel approach when the continuation is regained, and keep steering active after capture.
- [x] Bound lost-line recovery by elapsed time and movement; do not infer success from a fleeting hit.
- [x] Run behavioral regressions and full physical replay; investigate any failure before tuning.

## Task 4: Integration and verification

Files: `Robot/Protocol.cpp`, `Robot/Robot.h`, line documentation, integration tests.
- [x] Expose speed and motor tuning via explicit serial commands; preserve existing telemetry prefix compatibility and document units.
- [x] Run all host checks, Arduino compilation, and expanded replay. Have an independent reviewer inspect the final change and fix material findings with regressions.
- [x] Record measured simulation outcomes separately from unperformed floor trials. No physical movement is needed to complete software verification.

## Execution ledger

Baseline: the full existing host suite passed in the preceding audit. Temporary audit variants failed 8/27 cases at half coast deceleration and 12/27 at 1.5x motor speed.
Ruling: user explicitly delegated design/implementation, so proceed without plan approval. Keep work in a dedicated branch at the known project path rather than moving their work.

Task 1: complete — all three behavioral probes failed on 6ff11d7 before implementation. Original bend/end scenarios retained; model matrix expanded from 27 to 108.
Task 2: implemented — 120 ms rolling encoder estimator; explicit mm/s targets; feed-forward + PI, conditional integration using applied PWM, tunable zero floor, launch effort starts when execution becomes available. Speed test originally produced 270/330 for demand 180; now tracks 180/180.
Task 3: implemented — no blind exit interval; forward-facing centre reacquisition cancels approach; geometry distinguishes opposite-strip edges from a new opposite bend. Cue expires when acquisition completes. Capture stays responsive at its own physical speed. Recovery has a wall-clock limit in USB mode as well as translation/angle bounds.
Ruling: contact-position uncertainty spans the full tape width because both saved and new contacts may lie on opposite edges. Using 0.7 tape widths admitted the old incoming strip as an outgoing segment.
Ruling: the first cue survives overlap until new contrary evidence lies outside the incoming strip; completion resets it. A short distance latch alone failed acute overlaps, and indefinite geometric anchoring failed the next bend. This is covered by both bend and consecutive-turn replay.
Ruling: keep original turn interlock; no electrical braking change. Capture transitions pivot -> inner-wheel coast -> forward without a blind timer, and wheel PI observes actual output.
Ruling: exact PWM240 assertions were replaced with bounded positive-drive assertions in USB/firmware tests because regulated output must vary. Speed/trajectory tests cover the new physical contract. Old ideal recovery tests now integrate mm/s with dt instead of ratios.
Task 4: in progress — tuning commands implemented; host suite and default UNO R4 build passed once, before final configuration-invariant tests. Independent review and final verification still pending.


Final independent review: four material findings were reproduced with failing tests and fixed: stale ordinary-correction cues under angled starts; confirmation resetting on ordinary outer excursions; permitted PI tuning stuck at zero output; and wrapped heading sign reversal falsely completing alignment. No reviewer edits were applied blindly.
Ruling: a turn cue expires after sensorOffset+tapeWidth of travel regardless of centre-only streaks. Corrective contacts during acquisition do not become the next corner's cue. Completion establishes a new incoming segment.
Ruling: acquisition requires centre hits separated by 50 mm of coherent tracking, permits ordinary outer corrections/small gaps, and resets on genuine tape loss or more than 0.45 rad heading excursion. A fleeting hit cannot replenish recovery.
Additional root-cause finding: coasting inner-wheel speed could exceed the powered outer-wheel target, giving yaw opposite to the requested correction. A failing coasting regression now requires the outer demand to include measured inner speed. This fixed the residual combined long-coast/fast-motor and angled-start course failures without relaxing completion/rotation assertions.
Task 4: complete — 41 C++ programs, 26 Python tests and browser event checks passed. The preserved full-chain replay has 162 bend/end scenarios and 108 mirrored/angled-start course scenarios, all passing. Optimized address/undefined-behavior sanitizer runs of both replay programs passed. Default UNO R4 WiFi build passed (130512 bytes flash, 15052 bytes RAM). Optional battery build also passed (see final record). No upload or physical car motion was performed.
Ruling: keep this rebuild on its dedicated local branch for floor commissioning. No merge of the untested hardware behavior into the prior branch, and no change to the user's pre-existing Line-Control-Redesign.md edit.

Final battery build: UNO R4 WiFi with ROBOT_LINE_ONLY=1 compiled successfully. The tracked default remains ROBOT_LINE_ONLY=0.
