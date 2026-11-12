# Lab 07–08 firmware review

Reviewed 2026-11-11 at `46b42f0` on `qa`. This is an audit and proposed direction, not a completed implementation or evidence of a physical run.

## Requirements and current coverage

Sources: [Lab 07](https://cogs-300.github.io/labs/lab07), [Lab 08](https://cogs-300.github.io/labs/lab08).

| Requirement | Current code | Work needed |
|---|---|---|
| Tune two or more binary IR sensors to white tape | Three inputs on D3/D2/D4, one shared polarity; separate calibration sketch reads D2 only | Confirm installed modules; calibrate every installed input against both surfaces; prevent unused inputs contributing observations |
| Drive until tape, stop at tape end | STOP_ON_TAPE and STOP_AT_END exist | Verify real stopping distance, surface noise and start conditions |
| Correct left/right and follow the line course | LINE steers on simple masks but stops at loss or broad masks | Recovery through sharp bends and ambiguous masks, measured motion when feedback is available |
| Navigate the previously discussed branches/dead ends | MAZE has local search, return and a checkpoint graph | Reliable exit discovery, route selection and branching/cyclic-course tests; current checkpoint graph is not an exhaustive junction map |
| Recognize transitions between tournament sections | No tournament supervisor | Define observable section markers from the actual course, then implement explicit transition states and rearm conditions |
| Sweep both ways, estimate object bearing, approach closest object | FOLLOW maintains distance to an already frontal object | Add angle-indexed scanning, fresh range acquisition at each bearing, candidate selection, alignment and reacquisition |
| Find object from any initial orientation | Absent | Full measured angular coverage and tests with targets throughout 360 degrees |
| Remain within white arena lines | Absent | Boundary override during search and approach; establish sensor placement and braking margin for the entire chassis |

The lab pages do not specify a universal transition marker or arena geometry. Tape loss alone cannot distinguish a missed bend, dead end, section exit and sensor fault. An ultrasonic echo alone cannot distinguish an object from an arena wall. These ambiguities must be resolved with the course layout and explicit assumptions.

## Recommendation

Substantially redesign navigation while retaining the useful motor interlock, nonblocking sonar acquisition, serial watchdog, controller transport, recording and tested graph utilities. Retain odometry only if compatible encoders are actually installed and calibrated.

Separate responsibilities into sensor acquisition/calibration, measured movement, line/wall/object behaviors, and a tournament supervisor. Apply boundary and obstacle constraints to proposed motion consistently. Preserve actual sensor evidence and acquisition timestamps rather than interpreting repeated control-loop reads as new observations.

The present MAZE estimator assumes selected sensor probabilities; those values are engineering priors, not measured performance. More probability calculations cannot supply missing branch observations or heading measurements. Keep useful filtering, but evaluate improvement through complete behavior tests rather than estimator complexity.

Object search should stop and settle at measured angular samples, acquire fresh echoes, reject isolated readings, record bearing and range, select a candidate, align and reacquire before approach. Sweep coverage and target angular resolution depend on the actual sonar and course dimensions. A measured turn uses wheel feedback or another verified heading source; an LM393 floor sensor is not such a source. Do not substitute arbitrary turn durations for measured angles.

Preserve the requested PWM255 operating assumption pending physical measurement. Encoder feedback does not itself reduce inertia or create braking torque. Confirm braking and turning on the floor before claiming precision. Motor power-stage changes or added sensors require a wiring discussion.

## Specific weaknesses in the inspected code

- `Line.cpp` reads all three inputs immediately, assumes one common polarity, and has no installed-sensor mask or multi-sensor calibration workflow.
- `Navigation.cpp::basicLine` stops at masks 0, 5 and 7; it cannot independently complete all sharp-turn courses.
- `Explorer.h` finds and follows a candidate continuation without exhaustively registering a junction's other exits. Nearby unexpected checkpoints require manual confirmation.
- `Navigation.cpp::followWall` stops on a lost right wall. During a corner turn, a valid clear front reading terminates rotation, but a NO_RETURN observation does not; this can continue to the turn bound instead of recognizing the opening.
- Basic modes and MAZE interpret sonar no-return differently. MAZE requires finite front/right echoes for turns, so an otherwise open arena may be unusable for that guard. Missing or stuck echoes must remain distinguishable from completed no-return pulses.
- Existing turn guards inspect front/right rays, not the full chassis footprint. Floor sensors mounted only forward cannot guarantee rear-corner containment while rotating at an arena boundary.
- Tests for firmware navigation directly inject range readings. They do not validate real echo interrupts, crosstalk, sensor electrical behavior, or wheel count acquisition.
- The existing geometric replay exercises the Explorer core with synthetic motion and constant clear ranges. It does not include the production motor reversal interlock, full-body walls, arbitrary branches or tournament transitions.

## Wiring facts and questions

The existing code assigns left/centre/right floor sensors to D3/D2/D4. Historical confirmation covers only the centre on D2; current physical wiring must be checked. Preserve the working motor and sonar wiring.

LM393 identifies a comparator used in many modules. The needed information is the complete module function and pin labels, its digital tape/floor outputs, quantity, placement, and supply compatibility. If all three compatible digital floor modules already occupy the planned pins, integrating them does not inherently require a motor or sonar wiring change.

Encoder code proposes D12 and A4 for the two wheel A/output signals, with D13/A5 for quadrature B if present. These are proposed assignments, not confirmation of installed encoders or permission to wire unidentified modules. Confirm hardware before relying on this path.

Before finalizing the implementation architecture, establish:

1. Number and positions of installed floor sensors, their signal pins and tape/floor readings.
2. Whether both wheels have encoders; if yes, module labels, output channels and wiring.
3. Course section order and observable transition markers, arena boundary layout and target objects.

## Verification baseline

`python3 tests/run_checks.py` passes: seven C++ executables, twenty Python tests and browser-event checks. This establishes the current regression baseline, not lab completion or real-world performance. No firmware upload or motor command was issued during the review.

Implementation verification should cover complete line courses (including Z turns, branches and returns), actual motor-interlock timing in simulation, noisy/disconnected sensors, encoder stalls, transition false positives, 360-degree object acquisition, target loss and boundary encounters. Finish with a staged physical calibration and course run; report simulated and physical results separately.
