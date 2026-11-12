# Maze navigation rebuild implementation plan

**Goal:** Replace provisional time-driven navigation with sensor-led local control, measured motion, a persistent bounded maze graph and exploration planning.
**Architecture:** Hardware adapters produce observations and signed wheel counts. Pure C++ odometry, motion, graph and exploration components consume them. Arduino navigation executes their decisions; a versioned serial protocol and controller expose readiness, state and graph export.
**Tech stack:** Arduino UNO R4 WiFi / C++14, existing Python HTTP controller and browser JavaScript, host C++ tests.
**Spec:** Requirements and implementation contract below, derived from the user's maze/encoder discussion.

## Contract

- Preserve tested motor polarity, D5–D10 motor wiring, front/right sonar and IR centre D2 / left D3 / right D4.
- Continuous configured motor power (255 default); no imposed burst scheduling or fabricated odometry.
- Distinguish motion completion (measured travel/tape) from failure deadlines (stall/watchdog).
- Persistent graph for the entire run; explicit capacity overflow and explicit reset. No map clear on STOP, handshake or mode change.
- Represent arbitrary branch bearings, not only cardinal 90-degree turns.
- Prefer untried exits; use graph traversal to reach remaining frontiers. Incoming branch may be chosen only as explicit backtracking.
- Never merge junctions purely because estimated positions are close. Halt for explicit confirmation of uncertain loop closure.
- Encoder driver is disabled until board pins/type and scale are configured. No default encoder wiring is energized. Physical voltage compatibility remains a user setup requirement.
- Maze motion requires measured wheel scale, track width, sensor offset, tape width and turn clearance. No timed substitute for absent counts.
- Line-only/basic lab stop modes remain available without encoders; loss that requires measured recovery reports missing feedback instead of spinning blindly.
- Wall fallback must not silently corrupt a tape edge in the graph: wall detours are explicit localization holds in mapped mode; independent wall mode is separate.
- Collision guard, stale-sensor holds, STOP, watchdog and existing motor reversal protection remain. Sonar does not guarantee full-body clearance.
- No hardware upload or motor exercise without a connected board and verified calibration. No physical performance claims from host tests.

## Files and tasks

1. `Robot/MazeGraph.h`: 64 nodes, 128 undirected edges, up to 6 angular ports/node; transactional updates, Dijkstra path to nearest frontier, exploration status, export. Tests: dead-end backtrack, branching, cycle with confirmed identities, exhaustion and invalid updates.
2. `Robot/Odometry.h`, `Robot/Motion.h`, `Robot/Encoders.cpp`: signed count adapter, differential pose and path distance, measured translation/turn targets, per-wheel stall checks; zero-count motion never completes. Tests: calibration, rotation/translation, wraparound and feedback failure.
3. `Robot/Explorer.h`: segment following, measured axle positioning, centre-sensor angular survey, graph arrival, planned alignment/departure and backtracking. Incoming direction excluded from unexplored exits; nodes close to known positions require confirmation. Tests: synthetic corner/junction/dead end sequences and no automatic map reset.
4. Rewrite `Robot/Navigation.cpp`, `Robot/Line.cpp`, header/protocol: integrate components, keep manual/object control, add maze and lab demonstrations; version protocol. Delete obsolete recovery heuristic rather than keeping two authorities.
5. Controller: readiness/encoder/graph status, measured calibration controls, map download/reset/new-run actions, explicit error state handling. Tests: protocol versions, config validation, stop/connect/map semantics.
6. Run full host suite, Arduino build with SRAM check; fresh code review. Replace outdated documentation with setup, architecture, constraints and verified/pending status.

## Review focus

- A wheel stops producing counts while the other moves: abort, not an endless turn.
- STOP mid-edge then manual motion: preserve graph but invalidate localization; do not resume a fabricated node.
- Unknown loop closure or map full: do not corrupt existing nodes or silently forget history.
- Repeated sensor samples/serial commands: do not restart targets or duplicate graph arrivals.
- Missing calibration, stale sonar and disconnection: no maze motion; explain the specific reason.

## Execution notes

Work in the existing dirty checkout; preserve user files and previous calibration sketch. No automatic commits, resets, or destructive cleanup of unrelated files. Implement and test components in sequence; user has explicitly authorized the rebuild. Hardware model/geometry remain missing inputs, not guessed defaults.


## Execution record

- Graph, odometry, measured motion, encoder adapter, explorer, protocol 5 and controller integration implemented.
- Host regressions pass; final target compilation recorded in the task response.
- Review found blind departure travel and stale calibration tolerances; removed departure suppression and invalidated dependent calibration, with tests.
- Ruling: keep this dirty checkout and do not commit — preserves pre-existing user work; changes remain local and uncommitted.
- Ruling: keep unknown loop closure as an explicit hold — three binary IR sensors plus odometry cannot guarantee unique node identity; costs autonomous completion on ambiguous loops.
- Ruling: keep wall mode separate from mapped tape traversal — an unobserved wall detour cannot be assigned to the same tape edge; costs automatic escape from blocked tape.
- Ruling: host tests use C++14 — matches aggregate initialization supported by the R4 build; older host compilers need a newer language mode.
- Fresh review stopped at an external service usage limit after reporting the two fixed findings; do not describe the full review as clean/completed.
- No board upload, reset or floor trial performed. Encoder electrical identity and physical measurements remain setup requirements.

Final verification: five host C++ executables, 19 Python tests (including production-firmware bridge), browser-event suite and UNO R4 WiFi compile passed. Target: 88,448 bytes flash / 12,680 bytes static RAM. git diff --check passed. Files remain uncommitted.
