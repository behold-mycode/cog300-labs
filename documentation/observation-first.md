# Observation-first navigation — firmware 6

This is an experimental local-navigation implementation, not a physically validated maze solver. It replaces firmware 5's fixed-offset positioning and mandatory full-circle survey. The existing motor/encoder/sonar drivers, lab modes and graph storage remain. Nothing has been uploaded in this revision.

## One feedback loop

```text
20 ms timestamped observations ──→ local Bayesian tape belief + wheel speeds
          ↑                                      ↓
       physical robot ← motor guard ← proposed local action ← route suggestion
                                                      ↓
                                           verified route checkpoint
```

`WorldModel.h` holds the latest observation and a five-state Bayesian filter: tape left, centred, right, broad, or missing. Prediction mixes the previous distribution with a transition prior and shifts lateral probability according to observed yaw. The measurement likelihood is computed from the binary IR bits; posterior probabilities are normalized. Left/right and broad hypotheses marginalize alternative sensor patterns. These are local sensor-configuration hypotheses—not probabilities of an entire corner, dead end, or global robot pose.

The default hit probability 0.85, false-positive probability 0.15 and transition mixing 0.12 are explicit **engineering priors**, not measured accuracy. The model assumes common sensor quality and conditionally independent bits. Actual adjacent IR readings are correlated; this approximation must be checked with labelled data. Repeated stationary readings refresh sensor age and speed but do not repeatedly increase confidence. Duplicate/out-of-order observations are rejected. A fresh contradictory reading can stop motion even when the prior is confident.

`Explorer.h` owns local actions. Centre tape permits forward tracking. Side tape provides steering evidence. Tape loss or a broad pattern first stops and waits for three fresh observations with no measured wheel movement. A measured angular search starts toward the direction supported by recent tape evidence. It can accept a continuation before finishing its search, but excludes the estimated incoming direction. A candidate must survive wheel settling and measured forward travel across one tape width; a transient hit does not create a map node.

If the first search finds nothing, the robot can align and approach the last observed centre-tape location. This point comes from a previous sensor pose, not an assumed junction. Travel is bounded by that observation and the measured braking model, and the obstacle guard checks every command. It then reassesses from the new location. If no onward tape is found, an explicit return maneuver checks for incoming tape before following it. Search failure alone is not classified as proof of a dead end.

`guardMotion` evaluates the **proposed command**, including the very first pivot command, using sensor age and estimated wheel speed. Its stopping allowance is:

`speed × (reaction latency + observation age) + speed² / (2 × deceleration) + margin`

The front must clear that allowance and the applicable configured minimum. Turns also require a valid right observation and right clearance. Unknown/no-return sonar is not assumed clear in MAZE. Autonomous backward translation is rejected because there is no rear sensor. This is still only a two-ray guard; it cannot certify the full chassis sweep. Existing MANUAL/FOLLOW/WALL/basic LINE modes keep their previous behavior and do not use this new probabilistic controller.

## What the graph means now

The persistent graph stores **observed route checkpoints and traversed connections**, not a claimed exhaustive map of junction exits. A checkpoint is committed only after observing and following a continuation. A return along a known edge may match its expected endpoint; an unexpected nearby checkpoint requires confirmation. Conflicts preserve the existing graph and stop. STOP preserves memory; manual movement invalidates localization.

The graph can suggest a search direction when near a recorded checkpoint. Current tape evidence, incoming-direction exclusion and collision checks can reject that suggestion. The graph cannot mark space clear, force a turn or manufacture an unobserved branch.

**Remaining graph limitation:** local exploration does not yet guarantee that every exit at a complex junction has been observed. The existing Dijkstra frontier planner is available, but incomplete exit discovery means this revision does not guarantee autonomous completion of an arbitrary branching/cyclic maze. That requires physical junction observations and further route-association validation. Graph capacity is still 64 nodes/128 edges; it is not a rolling history.

## Setup and observation recording

Keep the existing [IR/encoder signal wiring plan](line-following.md#wiring-additions). Its firmware-5 navigation description is historical; this document defines current MAZE behavior.

1. Identify and electrically verify the encoder modules. Verify wheel sign/counts and calibrate wheel scales/track. IR tape polarity must also be set.
2. Enter geometry as before. Encoder changes invalidate odometry, geometry and dynamics; odometry changes invalidate geometry and dynamics.
3. Measure stopping behavior at the actual motor power, load and floor. Enter deceleration (mm/s²), reaction latency (seconds) and stopping margin (mm) under Maze setup. Use conservative measurements, not the simulation constants. MAZE refuses to start without these values.
4. Start an observation recording in the controller. This stops current motion; select the desired mode afterward. Stop recording and download CSV before starting a new capture. A capture stops at 20,000 rows (about 400 seconds at 50 Hz), preserving all collected rows rather than overwriting the beginning. Capture is held in the controller process's memory; save before closing it.
5. First record straight tracking, one corner, a sharper-than-90-degree turn, a Z, and an endpoint individually. Compare requested/actual behavior before a maze run. The console's belief display is diagnostic, not a statement of calibrated confidence.

New serial commands:

```text
DYNAMICS <decelerationMmS2> <reactionSeconds> <marginMm>
MODEL <hitProbability> <falsePositiveProbability> <transitionMix>
TRACE 1
TRACE 0
```

`MODEL` parameters must remain strictly inside their supported ranges. MAP RESET clears estimates and route history but preserves configured model parameters. The handshake is firmware 6; use the matching controller.

`OBS` CSV fields are milliseconds, sample sequence, tape mask, x/y/heading, signed left/right wheel travel, front/right ranges, front/right range ages, actual left/right PWM, and odometry-configured flag. Invalid ranges are -1. Observations record the motor output at sampling time, before that sample's decision. A tape mask is meaningful only after IR polarity calibration. Pose is estimated from wheel rotation, not external ground truth. `BELIEF` telemetry adds dynamics readiness, five probabilities, wheel speeds, a heuristic heading-uncertainty allowance and informative-update count.

## Evidence and weaknesses

Run `python3 tests/run_checks.py`. The suite includes a closed-loop differential-drive replay: sensor readings are calculated from sensor positions over finite-width tape, motor velocity changes gradually, and the controller receives observations rather than scripted state transitions. Cases include a Z, a turn greater than 90 degrees, isolated false tape bits, unobserved 2% wheel slip, and intentional endpoint return. Unit/integration checks cover duplicate/stationary evidence, rejected transient candidates, stale/unknown observations, speed-dependent stopping, first-pivot veto, calibration gates, capture and the existing watchdogs.

The synthetic wheel speed, braking, tape width and traction values are test fixtures. Passing these cases does not establish real-world accuracy. The simulation does not yet reproduce sonar beam geometry/crosstalk, full-body collisions, the motor driver's real reversal/coast dynamics, irregular floor reflection or a full branching maze. Probabilities and uncertainty growth are provisional. The stopping estimator uses wheel speeds, which can differ from chassis speed during slip. Coarse encoders can report zero counts while the robot is still moving slowly. No gyro, camera or rear/left obstacle sensor is present.

The next hardware deliverable is a recorded, labelled trace and measured braking/encoder parameters. Use those to fit the sensor likelihoods, evaluate recovery failures and then strengthen junction/graph association. Adding Bayesian arithmetic alone cannot resolve physically unobservable cases.

Final build verification: seven host C++ test executables, 20 Python tests (including the compiled production-firmware bridge), browser-event tests, and the UNO R4 WiFi Arduino build passed. Firmware uses 93,716 bytes of flash (35%) and 12,908 bytes static RAM (39%). No physical run is included in these results.
