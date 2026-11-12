# Tape maze navigation — firmware 5 (historical)

**Superseded navigation:** see [firmware 6 observation-first navigation](observation-first.md). The wiring table below remains applicable.

This rebuild separates motor control, wheel odometry, tape observations, local maneuvers and graph planning. It is compiled and host-tested; it has **not been uploaded or validated on the course**. The encoder modules are still unidentified. No encoder scale or chassis dimensions are prefilled.

## What the robot does

- `LINE`: follows tape using the three binary sensors. Centre means forward; left means stop the left wheel while powering the right, and right means the reverse. Tape loss or an ambiguous junction stops motion. This is the encoder-free lab demonstration.
- `STOP_ON_TAPE`: starts off tape, drives until any sensor sees tape, then stops.
- `STOP_AT_END`: starts on tape, follows until all sensors leave tape, then stops.
- `MAZE`: requires encoder and geometry calibration. Tracks tape, positions the axle at a detected feature, makes a measured full angular survey, records tape branch bearings, chooses an exit, and aligns with that exit. The centre sensor must actually see tape before departure. Feature detection resumes immediately, including closely spaced Z corners.
- `WALL`: independent right-wall following, target **10 cm**. Front detection at **20 cm** initiates a measured left turn when calibrated feedback and clearance permit. The turn ends when the front observation clears, not at an assumed 90 degrees. Losing the right wall stops; it does not justify a blind right turn.
- `FOLLOW`: retains the existing object-distance follower, target 20 cm. This mode can intentionally drive backward to maintain distance. It is separate from MAZE.

MAZE does not switch to wall following merely because tape disappears. A wall detour would break the assumption that the robot is still traversing its recorded tape edge. Obstacle guards can stop tape motion; they do not secretly choose a different route.

Motor commands use continuous configured PWM, **255 by default**. There is no periodic burst schedule. A measured maneuver finishes on encoder travel or tape observation. The 1.5-second no-count deadline reports a stalled commanded wheel; it never declares motion successful. The existing 250 ms motor reversal interlock and 500 ms desktop watchdog remain.

## Whole-route graph

The graph stores up to **64 nodes, 128 edges and 6 angular exits per node**, including corners, junctions and observed endpoints. These are explicit storage capacities, not a rolling recent-route window. Reaching capacity stops with `MAP_FULL`; older route information is never discarded to make space. Branch bearings are arbitrary angles, so a turn need not be 90 degrees.

The planner selects an untried exit at the current node. If none remains, Dijkstra's algorithm finds a route through known edges to another node with an untried exit. Returning along an incoming branch is a deliberate backtracking decision, not acceptance of the first tape found while spinning. Once every reachable recorded exit has been explored, the robot reports `MAP_EXPLORED`. No special goal marker is assumed.

An arrival via a known edge must match the expected node's location and branch bearings. An unknown edge ending near a known node pauses at `CONFIRM_LOOP_CLOSURE`; confirm only after checking that it really is the same physical junction. Conflicting observations stop with `RELOCALIZE_REQUIRED`. Encoders drift and three binary IR sensors cannot uniquely identify every visually identical junction. This implementation does not claim fully autonomous loop closure under that ambiguity.

STOP, HELLO and controller disconnect preserve the graph in RAM. Manual driving or selecting another autonomous mode invalidates map localization. Moving the robot by hand without wheel rotation cannot be detected: reset the map after repositioning. A power cycle/upload clears RAM. Export JSON before reset; there is no map-import or flash-persistence feature.

## Wiring additions

Keep the working battery and motor wiring. The existing wiring drawings cover the base robot; this table is the added signal plan.

| Device | Signal | UNO R4 WiFi pin |
|---|---|---|
| Left IR | digital OUT | D3 |
| Middle IR | digital OUT | D2 |
| Right IR | digital OUT | D4 |
| Left encoder | output/A | D12 |
| Right encoder | output/A | A4 used as digital GPIO |
| Left quadrature encoder only | B | D13 |
| Right quadrature encoder only | B | A5 used as digital GPIO |

```text
                Front of robot
          [Left IR] [Middle IR] [Right IR]
             OUT       OUT        OUT
              |         |          |
             D3        D2         D4
                    UNO R4 WiFi
Left encoder A/OUT ---- D12    A4 ---- Right encoder A/OUT
Left encoder B ------- D13    A5 ---- Right encoder B
                    (B only for quadrature)
Module GND -------- common robot GND -------- Module GND
Module VCC -------- supply verified for that specific module
```

Do not connect encoder VCC based on this signal diagram: verify the actual module's supply voltage, output voltage, labels and output type first. “LM393” names a comparator, not a complete encoder specification. The driver uses INPUT without pull-ups; an open-collector output may need a suitable external pull-up. Keep IR VCC on its verified regulated supply, not the motor battery rail, with common ground.

The proposed encoder A pins use different R4 interrupt channels from the existing front/right sonar echoes on A1/A3. Quadrature B is sampled on changes of A, giving **x2 decoding**, not x4. A5 does not receive an interrupt because it shares an IRQ channel with A3. No motor or sonar rewiring is required for this pin plan if the modules prove compatible.

Single-output encoders can count travel but cannot sense direction. Their sign is inferred from the actual motor command and retained during coasting. Quadrature A/B can measure direction. Neither type detects tire slip or guarantees chassis heading.

## Calibration and first run

Use the browser's **Maze setup** panel. Opening setup stops motion. Serial Monitor alternative: **115200 baud**, newline; close it before connecting the controller. Never have both applications own the serial port.

1. Mount all three IR sensors at the same height in a straight row ahead of the axle. Adjust each potentiometer so white tape and floor give distinct outputs. Observe the raw readings; the module LED does not establish whether OUT is HIGH or LOW. All three must use the same tape polarity, then select that polarity in Settings.
2. Demonstrate LINE and the two stop modes first. Verify left/right corrections with the wheels lifted before floor tests.
3. Identify and wire the encoders using the signal plan after checking module electrical specifications. Select single-output or quadrature. Check raw left/right counts by rotating each wheel separately. With quadrature, forward wheel rotation must increase that wheel's count; flip its encoder inversion option if needed. Encoder inversion has no effect in single-output mode because direction is inferred.
4. Measure millimetres per reported count for each wheel: known travel divided by absolute count change. Match the actual decoding used here. For quadrature, rotate the wheel by hand; for single-output, count magnitude is usable, but direction needs powered-command validation. Enter both scales and the effective wheel track width. Verify straight travel and measured rotation on the actual surface; refine effective track width from observed rotation.
5. Enter the forward distance from axle midpoint to the centre IR sensing spot, tape width, node-match radius and turn clearance, all in millimetres. Node-match radius must accommodate measured odometry error while remaining smaller than separation between distinct features. If no such radius exists, this hardware cannot reliably identify nodes from these measurements. Turn clearance must cover the chassis sweep and stopping margin as observed from the front/right sensor origins. The firmware checks those two rays, not the entire swept area.
6. Changing encoder mode invalidates wheel and geometry calibration. Changing wheel scale or track invalidates geometry calibration. Complete setup in that order. Changing geometry or direction/polarity with an existing map requires an explicit new-map reset.
7. Start on a straight section with only the centre sensor detecting tape, facing the intended course direction. Reset the map to set the origin, then select MAZE. Test a single corner, a sharp corner, a close Z, a dead end, and a branch/return sequence before a full run. Check stopping distance at PWM255, including battery and floor changes.

The geometry validity checks require tape width smaller than forward sensor offset, distance resolution finer than tape width, and angular resolution finer than the tape's apparent width. These checks reject unusable input; they do not prove physical accuracy.

## Serial interface

Commands below use placeholders, not suggested measurements:

```text
CONFIG 255 255 1 0 20 10 <tapeLevel: 0 or 1>
ENC <0 disabled, 1 single-output, 2 quadrature> <invertLeft: 0 or 1> <invertRight: 0 or 1>
ODOM <leftMmPerCount> <rightMmPerCount> <trackMm>
GEOMETRY <sensorOffsetMm> <tapeWidthMm> <matchRadiusMm> <turnClearanceMm>
MODE STOP_ON_TAPE
MODE STOP_AT_END
MODE LINE
MODE MAZE
STOP
MAP DUMP
MAP CONFIRM <candidateNodeId>
MAP RESET
```

`MAP DUMP` stops motion and streams a consistent graph snapshot; changes are locked until `MAP_END`. The browser downloads JSON. `MAP CONFIRM` records a verified revisit but leaves the robot stopped; select MAZE to resume. `MAP RESET` discards the graph and resets the odometry origin while stopped.

`HELLO` starts the 500 ms heartbeat watchdog session. The controller sends heartbeats automatically. Direct Serial Monitor autonomous commands without HELLO retain the pre-existing untethered behavior and have no desktop watchdog; STOP still works. After a session expires, send HELLO again before movement commands. Reconnecting to a live map reports its existing settings so a fresh desktop process does not overwrite tape polarity.

Telemetry:

- `DATA,ms,mode,state,frontCm,rightCm,leftPWM,rightPWM`
- `IR,leftRaw,centreRaw,rightRaw,tapeLevel`
- `COUNTS,leftSigned,rightSigned,encoderKind`
- `NAV,encoderEnabled,odometryConfigured,geometryValid,localized,nodes,edges,currentNode,candidateNode,xMm,yMm,headingRad`
- `MAP_BEGIN`, `MAP_NODE`, `MAP_EDGE`, `MAP_END`: export records. Angles are radians; edge lengths are estimated millimetres.

## Physical limits to resolve on the robot

A full survey requires room to rotate. Front/right sonar cannot observe the entire left/rear chassis sweep, and a completed no-return sonar pulse is not proof of free space. Invalid/stale readings stop motion. A corner close to a wall may stop at `FRONT_BLOCKED` or `RIGHT_BLOCKED` instead of being traversable. The robot does not mark such a stop as a successful turn or explored edge.

The survey assumes positioning by the measured forward sensor offset puts the axle sufficiently near the feature. It must be tested with actual tape spacing, sensor spacing, footprint and stopping overshoot. Very close bends can be physically indistinguishable within the sensor footprint. If the incoming branch is not observed, it stops rather than inventing one. Stopping during a survey discards the partial scan and starts a fresh measured sweep on resume.

Full PWM can provide starting torque but may overshoot a narrow tape branch. Encoder feedback measures that motion; it does not add braking torque or remove inertia. A missed alignment reports `EXIT_NOT_ON_TAPE`. Address observed overshoot with measured motor/braking behavior or hardware changes, rather than assuming an arbitrary burst duration will work.

Bayesian filtering is not included: there are no measured sensor error probabilities yet, and it would not resolve an unobservable junction identity. Collect labelled course observations before adding probabilistic inference.
