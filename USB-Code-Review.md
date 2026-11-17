# Robot code review — 2026-11-17

Review scope: production Arduino code, serial/browser control path, calibration and persisted profile, line/wall transition, future object behavior, and host regression coverage. The initial review was performed on `qa` and saved in draft PR #17 on `lab07-usb-review`. No upload or physical movement was performed.

## Fixed findings

- **Recovery bypass:** combined supervisor intercepted tape loss and entered wall navigation before the measured line follower searched. It now delegates recovery first and requires observed side walls before entering a corridor.
- **False ambiguous-pattern fault:** broad tape masks stopped indefinitely. The follower now crosses them slowly; outer-only masks pivot instead of following a fixed-radius arc.
- **Unjustified rear-distance prerequisite:** accumulated path length was treated as evidence of rear clearance and could prevent recovery near a start/bend. Removed from open-floor line recovery; the assumption of physical pivot clearance is explicit.
- **Wrong recovery geometry:** floor sensors inherited the chassis front offset. They now have their own measured offset in the hardware profile/fingerprint.
- **Missing-sensor coupling:** startup always enabled three sonars; line following depended on wall calibration. The installation flag defaults to two sonars, and line/stop/object modes do not require the future left unit.
- **Watchdog bypass:** serial MODE/TEST could start without a live USB session. Both now require the session; standalone and radio paths are disabled by default.
- **Encoder sign race:** first edges after reversal could use the previous commanded direction. Direction is now set before bridge output is enabled, retaining the prior direction during coast.
- **Delayed stop:** zero velocity targets waited behind the 20 ms rate limiter. They now clear motor demand immediately.
- **Unreachable duplicate line code:** removed the old line branches after replacing the production path with the measured follower.
- **All-sensor calibration gap:** IR calibration sketch now reads and checks left/centre/right, not only the centre.
- **Open-space side measurements:** values beyond 40 cm and completed no-return observations normalize to the 40 cm navigation bound; invalid/stale readings remain distinct. Corridor entry no longer depends on the open-space calibration width. An explicit open-beyond-range flag preserves branch detection, wall beliefs and centring even where the old geometric opening threshold exceeds 40 cm.

Independent code review found two further issues, reproduced and fixed: outer-only pivots now check swept front clearance, and object/tape-stop modes no longer trigger forward commissioning before their boundary rules. Explicit Wheel calibration is available and stops on completion.

## Checks retained deliberately

Motor polarity/reversal sequencing, USB heartbeat timeout, bounded serial parsing, endpoint ownership, encoder stall detection, sensor freshness, measured profile validation, and calibration integrity remain. Removing these would conceal failures or permit unintended movement. Historical standalone code remains opt-in rather than deleting the future feature; its alternate-build tests still run.

## New Lab 08 path

`Robot/ObjectSearch.h` owns measured back-and-forth angular sampling, median range selection, alignment/reacquisition, approach and tape-boundary stop. The existing controller mode selector exposes it. There is no automatic wall-to-arena transition without a course cue; explicit laptop start is the chosen current behavior.

## Verification

Final check: **32 C++ test executables plus scenario variants, 25 Python tests, and browser-event checks passed**. Both Robot and IRCalibration compile for UNO R4 WiFi. The measured-profile follow-up also passes simulated commissioning with approximately 5.105 mm encoder edges. `git diff --check` is clean.

Run `python3 tests/run_checks.py` for host checks and `arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi Robot` for the board build. New regressions cover USB ownership, corner recovery, broad tape, immediate stop, encoder first-edge direction, two-sonar commissioning/persistence, side-range saturation, commissioning boundary bypass, and pivot geometry. Geometric line tests include ±45°, ±90°, ±135° and ±150°. Object tests cover target bearings around the car, a farther distractor, no-return scans and boundary stop.

These tests use synthetic sensors/mechanics. Remaining hardware-dependent work is listed in [USB-Lab-Setup.md](USB-Lab-Setup.md). The existing orthogonal wall graph, loop confirmation behavior and legacy serial-only modes were not rewritten. Neither simulated success nor compilation establishes a flawless real course run.

## Measured-profile follow-up

Entered the reported wheel/body/sonar geometry on 2026-11-17. A wide-sensor replay (100 mm outer spacing, assumed centred middle IR and 20 mm tape) reproduced slow repeated wrong-way searches and alternating pivots on a tight bend. Recovery now remembers the successful search direction; outer-only pivots keep their direction until centre detection; saved tape position is updated only when the centre actually detects tape. Expanded angle regressions pass both layouts. Missing-profile tests now explicitly construct an unconfigured robot rather than assuming the real car profile is empty.

## Confirmed tape width

The user confirmed the centre IR is midway between the outer sensors and straight tape is 50 mm wide, with different geometry at turns. Updating the wide replay from 20 mm to 50 mm exposed premature stripe-edge reacquisition and acceptance of a partial incoming stripe at a sweep limit. Recovery now measures both observed edges, aims toward their angular midpoint, accepts centre detection alongside an outer sensor, and discards incomplete bands at sweep limits. The simulation models turn overlaps as the union of joined strips; it does not prove arbitrary physical junction geometry.

Independent review also identified a quantized midpoint edge case. A dedicated regression reproduces a stripe midpoint between encoder counts; centring tolerance now respects half a measurable heading step while still requiring tape detection. Full host checks and the UNO R4 WiFi build pass.
