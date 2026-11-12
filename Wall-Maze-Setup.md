# Wall-maze firmware 7: setup and physical validation

This revision adds **WALL_MAZE / Explore wall maze** for the cardboard-wall section. It is host-tested and compiled for UNO R4 WiFi, but has not been uploaded or physically validated. Existing tape modes remain available as **legacy** modes. Their full-power steering is not replaced by this controller.

## What changed

The new controller expresses motion in wheel millimetres per second. Each wheel has acceleration-limited feedforward plus PI feedback, rather than a choice between full power and zero. Feedforward uses the measured wheel speeds at PWM255; the global maximum PWM still caps output. The old minimum-follow-PWM setting does not clamp this new velocity controller. If the hardware cannot sustain the requested low speeds, calibration or hardware work is necessary; the firmware does not manufacture a reliable low speed with arbitrary timed bursts.

A junction follows this sequence: detect a possible side opening/front wall → measured positioning → coast and settle → gather fresh observations → record/associate a junction → select a route → check pivot clearance → measured pivot → settle and verify the exit. Pivot commands counter-rotate both wheels. Wheel-travel imbalance reduces the leading wheel's demand, and excessive estimated centre drift stops the maneuver. The turn slows toward its measured angle; at most two settled overshoot corrections are permitted. Encoder stalls stop the robot. The old reversal interlock remains.

The footprint's circumscribed radius is `hypot(max(front,rear), halfWidth)`. A pivot requires front/side axle-to-wall clearances beyond this radius plus margin, as well as sufficient recently traversed corridor behind the axle. Front-wall stopping reserves rotation space. The three ranges are compensated conservatively for motion since acquisition; observations with excessive orientation change are held until fresh readings arrive. This is a **local straight-wall/corridor model**, not a proof that every point of the swept disk is observed. A protrusion between sonar beams, diagonal cardboard panel, rear obstacle, wheel slip or bad geometry can invalidate it. The robot holds rather than attempting an unvalidated multi-point turn when clearance is insufficient.

Corridor steering centres between two walls, follows the nominal centre offset when only one wall is visible, and uses heading feedback. It does not chase a side opening as though it were a distant wall. During junction positioning it holds the entry heading. Return along a known corridor slows against remaining recorded distance, coasts when needed, and finishes approaching the endpoint before assessing the junction.

## Bayesian perception and route memory

Three binary wall/opening beliefs are updated from distinct valid sonar acquisitions. An update uses a mixed prior (10% uniform transition mixing) and a range likelihood bounded between 0.2 and 0.8, interpolated near the classification boundary. Nominal corridor width and robot geometry determine that boundary. These are transparent engineering priors, **not fitted probabilities of sensor accuracy**. Duplicate measurement IDs never add evidence; stationary evidence is rate-limited. Pose-stamped raw ranges still constrain every motion, independently of the belief.

A missing echo is not a doorway. No-return, invalid, stale, or excessively rotated measurements hold motion. Fresh contradictory obstacle measurements override prior confidence. Ambiguous junction classification eventually stops with `WALL_AMBIGUOUS_GEOMETRY` rather than waiting forever. This conservative treatment can prevent operation in a large open room beyond sonar range; that is an explicit limitation.

The existing graph stores up to 64 junctions and 128 connections. Unvisited exits are preferred; Dijkstra routes through known connections to another frontier, then back to the start after recorded exits are exhausted. Expected revisits check travel length, location and opening signature. Unexpected nearby matching junctions require `MAP CONFIRM`; ambiguous matches and capacity exhaustion stop without erasing old routes. Export includes edge traversal counts. The graph is not a complete timestamped motion history; use observation CSV for that.

Junction departures currently assume approximately straight corridors and orthogonal exits. This is not a general arbitrary-angle SLAM solver. Candidate exits are based on the three stopped observations plus the traversed incoming corridor; no full 360-degree survey is implemented. Closely spaced junctions within the departure footprint can be indistinguishable. Automatic loop closure is deliberately not claimed. Moving the chassis by hand cannot be detected reliably: reset the map after repositioning.

`WALL_MAP_EXPLORED_NOT_GOAL` means the recorded exploration is exhausted and the robot has returned near its start estimate. It is **not goal-room recognition**. A long corridor provides little longitudinal localization information; return accuracy depends on odometry and the configured association tolerance. The program does not promise an exact return position.

## Wiring

Use the verified regulated module supply and common ground. Do not connect modules to the motor battery rail. Pin labels on the actual modules determine connector order.

| Signal | UNO R4 WiFi pin |
|---|---|
| Motor A enable / IN1 / IN2 | D5 / D6 / D7 (unchanged, physical right wheel) |
| Motor B IN3 / IN4 / enable | D8 / D9 / D10 (unchanged, physical left wheel) |
| Front HC-SR04 TRIG / ECHO | A0 / A1 (unchanged) |
| Right HC-SR04 TRIG / ECHO | A2 / A3 (unchanged) |
| **Left HC-SR04 TRIG / ECHO** | **D0 / D11** |
| Left / centre / right floor IR OUT | D3 / D2 / D4 |
| Left LM393 slotted encoder OUT | D12 |
| Right LM393 slotted encoder OUT | A4 used as digital GPIO |

**Disconnect the previous D11 LED connection.** Firmware 7 leaves D11 as an input even at boot and when left ranging is disabled, so it cannot drive against a connected ECHO output. The old external green LED function is retired. D0 is no longer available as UART RX; USB serial remains the controller link. A4 is occupied by the encoder and cannot simultaneously be used for I2C SDA. The existing optional quadrature B assignments D13/A5 are unused by these single-output encoders.

The three sonar ECHO interrupts and two encoder interrupts use distinct R4 channels. Left sonar is enabled explicitly with `SONAR 1 <intervalMs>`. The configurable stagger is 25–100 ms; default is 70 ms. **25 ms is an experimental option, not a demonstrated crosstalk-free setting.** Three valid short echoes at that stagger give roughly 75 ms per direction; timeout cases take longer. Only one echo acquisition is armed at a time. Shorter software timeouts do not eliminate reflected sound.

## Calibration order

All dimensions are millimetres, wheel speeds mm/s, acceleration/deceleration mm/s², and angles radians internally. No physical measurements below are filled in automatically. The app no longer has setup forms. The command sequence below is a legacy engineering/debug reference; current robot-owned startup is described in [Robot-Wiring.md](Robot-Wiring.md). Configuration stops motion. Wall-only calibration initially lives in RAM. The standalone extension can SAVE a complete race calibration and restore it at boot; graph and navigation state remain RAM-only. See [Standalone-Setup.md](Standalone-Setup.md).

1. Verify motor directions with wheels lifted. The defaults retain the prior working channel polarities. Changing motor polarity invalidates wall calibration. Changing encoder mode or wheel scale also invalidates wall calibration; unchanged reconnect settings preserve it.
2. Enable the left sonar after wiring: `SONAR 1 70`. Check each distance separately, including a cardboard corner. Compare single-sensor and multi-sensor recordings before reducing spacing.
3. Select `ENC 1 0 0` for the two single-channel encoders. Turn each wheel independently and check COUNTS. The driver counts **both rising and falling edges**. Count actual transitions per wheel revolution; do not substitute a guessed slot count. Direction is inferred from motor output, so hand-turning counts do not establish sign. Verify signs under powered movement. Quantization, coasting and backlash remain limitations.
4. Set `ODOM <leftMmPerCount> <rightMmPerCount> <effectiveTrackMm>`. Measure travel per count and refine the effective track from physical turns. Counts measure wheel rotation, not traction. Coarse counts can make small-angle turns infeasible; startup rejects excessively coarse angle/footprint tolerances.
5. Set `BODY <front> <rear> <halfWidth> <frontSonarX> <sideSonarX> <sideSonarY>`. Measure the entire footprint including wheels, boards and protrusions, relative to axle midpoint. Side sonars are assumed symmetric in lateral offset and share a forward offset; mount them accordingly. The front sonar is assumed on the centreline and all three face the nominal forward/right/left directions.
6. Set `SPEEDCFG <leftSpeedAt255> <rightSpeedAt255> <Kp> <Ki> <acceleration>`. Determine full-speed measurements on the floor under actual load. Gains are user-supplied; they must be tuned on the robot. Confirm that requested slow travel and slow pivots are sustainable. The current motor STOP is **coast**, not active braking. Do not claim low-speed control merely because encoder counts are readable.
7. Set `WALLCFG <cruise> <turnWheelSpeed> <margin> <coastDeceleration> <reactionSeconds> <matchRadius> <nominalCorridorWidth>`. Reaction must be at least 0.1 s to cover the velocity measurement window; choose conservatively from observed behavior. Measure coast deceleration and allow for floor/battery variation. Match radius must accommodate observed drift but be smaller than separations between distinct junctions. Corridor width is an explicit classifier assumption, not a discovered universal maze dimension.
8. Reset the map at a straight, clear corridor entrance. Select **Explore wall maze** / `MODE WALL_MAZE`. The robot needs a traversed corridor behind it before its first pivot; do not start pressed into a junction or dead end.

Useful commands:

```text
STOP
STATUS
LOG ON
TRACE 1
TRACE 0
MODE WALL_MAZE
MAP DUMP
MAP CONFIRM <candidateNodeId>
MAP RESET
```

`MAP CONFIRM` leaves the robot stopped; select the mode again to resume. A manual driving command invalidates wall-map localization. STOP, map export and an unchanged reconnect preserve it. Non-wall autonomous modes invalidate wall localization. Reset before switching between a tape graph and a wall graph.

## Diagnostics and verification

The firmware exposes left range, three wall probabilities, requested/estimated wheel speeds and map information in its diagnostic serial protocol. The simplified app does not expose diagnostic panels. `WALLNAV` telemetry carries those values; `OBS` CSV includes left range/age, wall beliefs, wheel targets/estimated speeds and numeric wall state in addition to existing pose/raw tape/front/right/PWM fields. Encoder speeds update over 100 ms windows; telemetry values are estimates, not ground truth. Start observation recording before selecting the mode, then download the CSV before closing the controller. The ordinary recent log is bounded and is not a full run archive.

Run `python3 tests/run_checks.py`. New tests cover corner sweep rejection, unequal wheel feedback, settle/overshoot logic, invalid/stale ranges, duplicate evidence, actual three-sonar echo scheduling, calibration gates, controller actions, route export, uncertain loop confirmation, capacity/ambiguity failures and wider corridors. Firmware-in-loop simulations include the real motor reversal interlock and coast dynamics, staggered synthetic range readings, rectangular-body collision checks, a dead-end return, a T-junction with two explored branches, 2% unobserved wheel slip, faster travel/weaker deceleration, and large reaction allowance with tight node tolerance.

These simulations do **not** model sonar beam width, multipath/crosstalk, real encoder interrupt noise, uneven traction, drivetrain stiction/backlash or irregular cardboard. They are regression evidence, not a physical performance claim. Physical validation remains: lifted wheel direction/counts → straight speed tracking/coast measurement → isolated pivot in open space → corridor/dead end → one junction → full course. Record failures before increasing speed.

## Remaining tournament work

The standalone extension now provides combined `RACE` navigation and a laptop **Run line → maze** button; see [Standalone-Setup.md](Standalone-Setup.md). It searches for lost tape and checks corridor evidence before handing off to this wall controller. Validate the transition on the real course. Goal recognition is intentionally omitted, and Lab 08 object-search behavior is not implemented by this mode. Separate line and wall modes remain useful for initial tests. Legacy tape modes retain their previous limitations.

### Verification recorded for this revision

On 2026-11-11, `python3 tests/run_checks.py` passed 13 C++ test executables (plus three additional dead-end replay parameter runs), 21 Python tests and browser-event tests. The T-junction replay also passed AddressSanitizer and UndefinedBehaviorSanitizer. UNO R4 WiFi compilation passed with 105,032 bytes flash (40%) and 13,600 bytes static RAM (41%). Independent review identified and prompted fixes for return braking, ambiguous classification, calibration invalidation and collision-test coverage. No firmware upload, physical motor command, commit, push or merge was performed.
