# Line-control redesign — proposed 2026-11-17

## Outcome and scope

Replace the Lab 07 line-driving path with one coherent controller that normally
keeps moving forward, has enough torque to move on the actual floor, and uses
encoder measurements to control travel and turns. Recovery must gather new
information by changing position rather than alternating indefinitely at one spot.

This proposal preserves the laptop/USB workflow, IR polarity (0=tape), installed
left/centre/right pins D4/D2/D3, and measured geometry. It requires no new sensors.
Line-only operation does not depend on sonar. Lab 07 drive-until-tape and
stop-at-end remain explicit modes. Lab 08 object search remains manually started;
wall/object behavior stays separate and available for later integration. No third
sonar, standalone startup, course map, or automatic section transition is added.

The current uploaded firmware is the saved straight-power trim at `dfa9be0`.
This document is a proposed replacement, not a claim that new code is running.

## What the audit found

1. `Navigation.cpp` sends line planner speed requests through `LineDrive.h`, which
normalizes them to near-full PWM. A tiny requested turn speed becomes full power.
The planner's deceleration and wheel-balance calculations therefore do not have
the physical effect they assume. Equal forward requests also all become PWM240.
2. `wall::Turn` is a stationary, precise pivot primitive with drift checks and
settling/reversal corrections. It is reused inside line recovery despite the
line driver's open-loop high power. The observed RACE_TURN_DRIFT is consistent
with that mismatch; it is not proof that the encoder hardware is bad.
3. Wheel speed is estimated in 100 ms bins. With 5.105 mm per edge, one extra edge
changes the estimate by 51.05 mm/s. Low-speed PI control needs a more suitable
estimator, not a claim that every zero bin means the wheel has stopped.
4. Motor reversal waits 250 ms. Recovery also waits for speed/settling. Frequent
pivot reversals stack these waits. Ordinary corrections should not reverse wheels.
5. Motor zero currently disables the L298 enable and coasts. It is not active
braking. Reaching an encoder target does not instantly stop the chassis.
6. The sharp-angle replay advances ideal wheel distances using requested speeds.
It bypasses the production PWM mapping, motor dynamics, reversal timing and coarse
encoder observations. Passing it does not validate the uploaded full-power car.
7. Net encoder counts at the last tape contact do not record the full trajectory.
Independently returning each wheel to an old count is not generally an exact path
reversal, and it can repeatedly return to the same unhelpful pose.

I added these patches during the lab session; the resulting mismatch is a design
problem to fix, not a reason to keep adding exceptions to the existing state machine.

## Hardware facts and what the encoders can tell us

Track=130 mm, wheel diameter approximately 65 mm, 20 slots per wheel, both edges
counted. Initial scale is 5.105 mm/edge. A nominal 90-degree in-place turn needs
102.1 mm per wheel in opposite directions, about 20 edges each. A single
left/right differential edge changes estimated heading by 2.25 degrees. At the
150 mm centre-IR offset, that is approximately 5.9 mm of lateral sensor motion.

Heading estimate is (right travel - left travel)/130, in radians. This measures
wheel-derived yaw, not absolute chassis orientation. These are single-output
encoders: firmware infers direction from its command. Slip, a wheel being pushed,
coast during reversal, diameter error and effective track all affect that estimate.
The three binary IR readings describe where tape is under three points; they do
not directly report a corner angle. Geometry makes a requested angle executable;
IR observations must determine which direction the course actually takes.

## Approaches considered

- Simple binary-IR proportional steering at fixed PWM: small, keeps moving, but
cannot compensate for unequal wheel response or stopping travel. Useful baseline,
not sufficient for the reported overshoot and recovery failures.
- Encoder-balanced forward follower plus explicit corner/recovery behavior:
recommended engineering choice. Continuous high-torque motion on ordinary tape,
measured distance/heading where they matter, IR confirmation of reacquisition.
- Rebuild around a global map: rejected for this Lab 07 scope. It adds uncertainty
and state without fixing actuator behavior or sparse tape measurements.

## Proposed control structure

Two line-specific components replace the current LINE-to-Race-to-LineDrive path:
`LineFollower` chooses forward speed, curvature and a finite motion objective;
`WheelControl` executes that request using actual wheel feedback. Neither knows
about wall graphs, sonar beliefs, or object-search states. Hardware/USB services
remain shared. The old combined RACE path is isolated from this replacement.

The line behavior has four observable phases: Follow, Corner, Search, Stopped.
Internal progress is expressed as measured distance/heading, not nested waits or
repeated restarts of the same search. One owner produces the wheel command per tick.

### Follow

Use the binary IR pattern as a coarse lateral error with the last reliable side
remembered. Use modest proportional steering with yaw-rate damping from encoders;
do not pretend three bits are a high-resolution analog line-position estimate.
Both wheels normally move forward. Return smoothly toward straight motion when
centre tape is restored; do not immediately pivot or stop for an outer-sensor event.

A short all-floor gap may continue the previous forward arc for a bounded distance
(initial hypothesis: half the 50 mm tape width). The bound is distance-based and
must be exercised in the complete simulation. Persistent loss or a persistent
strong outer signal enters Corner/Search, not an unbounded remembered turn.

### Corner

Store incoming heading and a small rolling window of sensor-contact positions
using the actual 150/145 mm longitudinal offsets and +/-50 mm lateral offsets.
Contacts indicate a tape region, not an exact vertex. Project a bounded approach
waypoint toward that region so the axle moves toward the bend before a tight turn.
Do not blindly drive an additional 150 mm after every outer-sensor detection.

Try a forward arc first. If the inferred bend cannot be followed with both wheels
moving, commit to one turn direction and use a one-wheel turn or opposing-wheel
pivot as needed. Use differential encoder progress and the measured coast response
to release drive before passing the desired region. IR detection confirms the
outgoing segment. No unconditional 90-degree turn: courses include sharper bends.

Allow one controlled reversal when geometry requires it. Keep the existing bridge
interlock until physical data supports a replacement. Avoid it during normal
tracking. Stop/settle only when reversing or capturing a sharp turn genuinely needs
it, not as a standard response to every deviation.

### Search with spatial progress

Remember the last reliable side, incoming heading and recent sensor-contact region.
First continue a short forward arc toward the expected continuation. If it fails,
perform one measured local sweep toward the remembered side and across the nearby
heading range. If that location yields no line, advance toward a new bounded
waypoint in the last observed tape region and search there. There must be measurable
translation between repeated search passes, not infinite left/right oscillation.

A recovery episode has an explicit finite distance/angle budget and never resets
that budget from a fleeting sensor hit. Provisional bounds: no more than two search
locations, each new location at least 25 mm from the previous one; total unobserved
translation no greater than two sensor offsets (300 mm). Exact scan extents and
corner classification thresholds are builder choices to be chosen in the full
simulation and reported before floor testing. Reacquisition must lead to forward
progress along tape; one transient centre hit is not proof of success.

If it cannot find the course after the finite search, report `LINE_NOT_FOUND` and
stop. This is preferable to indefinitely driving away or thinking in place.
Stop-at-end bypasses all search and remains a deliberate stop on tape loss.

### Wheel feedback and motor calibration

Use a fixed control cadence and an encoder estimate that combines edge timing
with a rolling count window. Do not differentiate single raw binary tape samples,
or reset feedback state at each control update. Timestamp counters atomically;
recognize encoder quantization and loss of direction certainty at reversals.

Use a measured continuous drive feed-forward map with a per-wheel sustainable-power
floor, plus feedback to balance actual wheel travel/speed. Starting power and
running power are separate measurements. No 35 ms on/off pulse scheme is proposed.
No request for an unachievable crawling speed is silently normalized to full PWM.
If a demanded curvature cannot be achieved within the usable continuous motor
range, explicitly choose a one-wheel turn/pivot rather than silently clipping the
steering and hoping it follows the line. Feedback saturation must be visible.

Begin from the observed high-power range (roughly PWM230–255), not the failed
80 mm/s target. The sustainable floor, relative wheel response and coast angle
must be measured on the actual floor; 230 is provisional, not a calibration fact.
The first implementation may use encoder travel balance over windows rather than
a fully tuned velocity PI loop if that is more stable at this coarse resolution.
This choice belongs to the builder and must be justified by end-to-end tests.

Retain coast as the initial stopping policy and predict release from measured
coast distance. Do not enable L298 active braking blindly: the datasheet requires
its braking current to remain within 2 A. A braking-mode change needs the motor/
supply information and a separate measured check; it is not a prerequisite to
writing or testing this controller.

Motor characterization is an explicit laptop action on clear floor, not a surprise
motion hidden in Line Start. Keep existing geometric measurements. Do not require
sonar just to initialize the new line mode. Separate line calibration validity
from the wall navigator's corridor and sonar configuration.

## Verification that would count

1. Whole-chain simulation runs the actual line behavior, wheel controller, motor
interlock/output and quantized encoder input together. Motor model includes an
explicit start/run deadband, unequal left/right response and coast. The model
parameters are assumptions until measured. Cover both turn directions, 45/90/135/
150-degree bends, joined/widened tape, small gaps and initial lateral offsets.
2. Assert course progress, bounded recovery path, correct steering, and absence of
repeating stationary search cycles. Compare against the saved firmware on the same
model. Record failures instead of replacing assertions with the new behavior.
3. Tests must show low-level commands actually implement the requested wheel ratio
and that a slower target does not turn into larger PWM accidentally. Verify the
minimum-torque constraint and report when it prevents a desired turn.
4. Verify USB stop/disconnect and encoder-stall stops during every phase. Verify
Lab 07 stop modes. Re-run wall/object regressions to detect shared-service changes.
5. Physical commissioning: observe each wheel's direction/counting, straight travel
and left/right turns on the current floor; record IR, counts, estimated speed,
commanded PWM, phase and target. Validate approximate wheel scale and effective
track using distance and angle, not just nominal dimensions.
6. Trial the troublesome corner in both directions. Measure uninterrupted progress,
recovery success and overshoot; distinguish actual floor evidence from simulation.
Do not claim flawless performance or an exact known heading from encoders alone.

## Deliverables and rollout

Preserve `dfa9be0` and the older working milestone. Implement on a new branch after
the design is reviewed. Delete the old line-specific normalization/retrace patches
from the active LINE path rather than layering a new mode on top of them. Keep
Lab 08/maze scope separate. Update the controller to show phase, both wheel speeds,
PWM and IR readings so physical tuning is visible. Provide a short commissioning
sequence with explicit movement commands the operator can start.

This design is ready for review; no replacement firmware has been uploaded.

## Primary references

- Lab requirements: https://cogs-300.github.io/labs/lab07 and https://cogs-300.github.io/labs/lab08
- Continuous line steering example (uses richer sensors than this car): https://www.pololu.com/docs/0J21/7.c
- Wheel-based odometry limitations: https://docs.wpilib.org/en/latest/docs/software/kinematics-and-odometry/differential-drive-odometry.html
- L298 stop modes and current limits: https://www.st.com/resource/en/datasheet/l298.pdf
