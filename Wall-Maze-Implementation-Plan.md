# Controlled wall-maze navigation implementation plan

Goal: implement the discussed encoder-assisted cardboard-maze navigator, prioritizing chassis clearance and controlled turns over aggressive movement. User authorized implementation after the design discussion.

Architecture: retain legacy tape modes and shared graph storage; add an explicit WALL_MAZE mode, three-sonar acquisition, a host-testable Bayesian wall navigator, and independent wheel velocity control. Calibration values remain unset until measured. Unknown goal/transition markings are not invented.

Constraints: UNO R4 WiFi, L298N, three digital IR modules, two single-output optical encoders. Left sonar TRIG D0/ECHO D11 is opt-in and replaces D11 LED use; the LED function is retired and D11 remains input at boot. Existing motor/front/right sonar wiring stays. No upload or physical movement during development. Full graph retained until capacity, with explicit uncertain loop-closure stops. Coasting stop remains until active-brake current is physically checked.

Review focus: corner swept-volume clearance; mismatched wheel response; coasting and single-channel direction inference; stale/asynchronous ranges and correlated evidence; map preservation across stop/reconnect and invalidation after manual movement.

- [x] Add regression tests for rectangular chassis turn envelope, measured turn convergence/overshoot and wheel synchronization, Bayesian duplicate rejection and contradictory readings, stale/no-return behavior, junction/dead-end/backtracking, map capacity and ambiguous revisit.
- [x] Implement `Robot/WallNavigator.h`: calibration, time/pose-stamped ranges, local wall beliefs, corridor steering, measured junction approach, stopped assessment, graph routing, controlled pivot and verification. Finite echoes provide geometric constraints; no-return remains unknown for clearance. Assume locally straight corridor walls and approximately orthogonal junctions; explicitly stop when those assumptions cannot support a maneuver.
- [x] Implement `Robot/Velocity.h`: acceleration-limited target speeds, wheel feedback with feedforward/PI and anti-windup, stop reset; use existing reversal interlock. Bound turn translation drift and settle before corrections. No timed turn completion.
- [x] Add optional third sonar and adjustable stagger interval; preserve timeout semantics and IRQ independence. Test actual acquisition scheduler with synthetic echo edges.
- [x] Integrate WALL_MAZE in firmware protocol, telemetry, shared map export/reset/confirmation and browser controls/calibration. No automatic goal recognition without a defined observable marker.
- [x] Run regression suite and geometric closed-loop tests with motor dynamics, asymmetric wheels, side openings and dead ends. Compile UNO R4 firmware. Review failure cases and document wiring, calibration, assumptions and physical validation limits at project root.

Implementation scope clarification: WALL_MAZE is an explicit new mode. Automatic course transition, goal recognition, object search and general arbitrary-angle/automatic-loop-closure mapping remain separate work. Synthetic tests do not establish physical completion. See Wall-Maze-Setup.md.
