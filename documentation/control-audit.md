# Firmware 5 rebuild audit (historical)

See [firmware 6 implementation and current limitations](observation-first.md) for the current controller.

## Replaced

- Removed the former AUTO tape-to-wall fallback, timer-based tape search and LineTracker recovery code.
- Removed imposed drive bursts; motor outputs remain continuous at the requested power.
- Replaced recent-route heuristics with a persistent bounded graph, arbitrary exit bearings and shortest-known-route planning to remaining frontiers.
- Replaced assumed turn durations with signed wheel odometry and measured motion targets.
- Added encoder/geometry readiness, explicit endpoint backtracking, uncertain loop confirmation, map export/reset and raw count telemetry.
- Preserved base wiring, observed motor polarity, manual/object modes, stop/watchdog and reversal interlock.

## Decisions and costs

- Full angular surveys provide branch observations, but require sufficient physical rotation space. They are not yet proven on this chassis or cardboard course.
- Unknown loop closures require confirmation rather than automatic merging. This prevents knowingly ambiguous map corruption, but looped courses may need intervention.
- Losing tape does not silently switch to wall following; wall mode remains separate. Blocked tape motion may need intervention instead of an automatic wall detour.
- Graph capacities are 64 nodes/128 edges/6 exits. Overflow stops without forgetting the route. RAM persistence lasts only until reset/power loss.
- Setup values are unfilled until measured. Maze mode cannot run meaningfully with the presently unidentified/unconfigured encoders.

## Verification

Run `python3 tests/run_checks.py` for host C++ components, compiled-firmware/controller integration, controller Python tests and browser-event tests. Arduino target build:

```sh
arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi Robot
```

The target build uses 88,448 bytes of flash (33%) and 12,680 bytes of static RAM (38%), leaving 20,088 bytes for stack/local allocations.

Host coverage includes sharp turns and a close Z detection, dead-end return and remaining-frontier selection, cycles/confirmation, graph exhaustion/capacity, signed-count wraparound, zero-progress motion and per-wheel stall, pause/resume survey, no map reset on STOP/HELLO, manual localization invalidation, calibration invalidation, export locking/validation, line polarity, lab stop modes, wall-loss hold, obstacle stop, continuous PWM, reversal protection and browser/serial watchdogs.

Synthetic observations test state-machine decisions, not real tire traction, IR threshold stability, encoder electrical compatibility, sonar coverage, braking or successful completion of the course. No upload or motor exercise was performed as part of this rebuild.

A fresh reviewer identified the blind departure distance and stale derived calibration; both were fixed with regression coverage. The reviewer subsequently hit a service limit, so its whole-change review did not finish. Remaining integration checks were performed in the main session.
