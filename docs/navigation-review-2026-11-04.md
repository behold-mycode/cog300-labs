# Navigation review — 2026-11-04

Reviewed the current uncommitted adaptive implementation on `lab-03/adaptive-wall-approach`, not just Checkpoint 2. No production code or board firmware changed during this review.

## Priority findings

1. **High — ALIGN rejects the correction needed after an over-turn.** `Robot/Navigation.cpp:204–210` suppresses every positive PD output for 560 ms. A deterministic sequence through CORNER into ALIGN with right distance 30 cm and target 15 cm produced `PD=450`, yet wheel commands were `255,255`. This is a control-policy conflict, not a gain problem. Remove the unconditional directional veto; any transition limiting should preserve the sign of necessary feedback.

2. **High — computed wall geometry is not identifiable from arbitrary front/right echoes.** `Robot/Navigation.cpp:159–179` uses range peaks to infer same-wall returns and then computes bearing and normal clearance. Those equations need common-origin measurements of one plane; the actual sensors have different origins and can see different walls. A falling side reading plus rising front reading cannot establish that premise. The two readings are also normally separated by 70 ms. Turn completion at lines 140–145 uses the distance ratio without even the reacquisition gate. Remove this inferred heading from control unless the measurement geometry is established; use side-distance feedback and front clearance as distinct signals with the present arrangement.

3. **High — an open front field stops otherwise valid side-wall tracking.** `Robot/Sensors.cpp:67–69` maps no return to invalid, and `Robot/Navigation.cpp:117` stops after cached data expires. Reproduced `front=-1, right=15 -> FRONT_INVALID, 0,0`. A sensor fault and an obstacle beyond ultrasonic range currently have the same representation. This does not justify treating every failed echo as clear; it requires distinguishing acquisition outcomes and specifying behavior for sustained no-return versus hardware failure. The current requirement implicitly assumes the front always sees a reflecting surface within range.

4. **High — positive PWM is mistaken for physical motion.** Navigation counts powered time from motor command signs (lines 106–110), and uses inside-wheel commands as low as 64 in tracking and 90 in corners at full outer power. No measured starting/running PWM or wheel-speed mapping supports those choices. The user has already observed poor motion at lower PWM. Physical inner-wheel stall can produce the prohibited one-wheel turn and a different angle/radius, while the controller believes its normal turn succeeded. Calibrate each wheel's loaded response before further steering-gain tuning; retain the calibrated motor pin mapping.

5. **Medium — simulation does not validate the reported failure.** `tests/test_corner_motion.cpp:36–44` checks commanded PWM and heading only when the car reaches y=100, after subsequent correction. It does not constrain maximum corner overshoot or verify both physical wheel speeds stay positive. In a temporary model with wheel speed zero below PWM 150, all nine cases still passed. That threshold is a hypothetical sensitivity test, not a measured property of this car. The model also caps rays at 400 cm and supplies valid returns instead of simulating no echo. Add a corner-exit angle/overshoot measurement, wall-distance bounds over the path, motor deadband/asymmetry, and no-return cases.

6. **Medium — derivative sensitivity and duplicate estimators complicate tuning.** `Observation::update` and `trackCorrection` separately estimate side rate with different caps (80 vs 50 cm/s); corner bearing has another filtered derivative. A 1 cm right-distance increase over 140 ms produced a 48 PWM wheel differential. This is reproducible sensitivity, not proof the gain is wrong on hardware. Use one timestamped rate estimate per signal and tune it against stationary sensor noise and real motion. Repeated clipping is not a substitute for calibration.

## Assessment

Correctness: major navigation-policy and observation-model concerns. Maintainability: navigation needs simplification; basic sensor/motor/protocol separation is worth keeping. Performance: no CPU bottleneck found; measurement cadence, smoothing delay and motor response are more material. Security: no new security issue identified in the inspected controller/serial paths; this was not a comprehensive security audit.

Useful foundations: nonblocking alternating ultrasonic acquisition, explicit motor mapping, actual sample timestamps for PD, protocol framing, Stop, and connection watchdog. Keep these. Consolidate control around one side-distance loop and a clearly defined front-obstacle transition. Remove speculative wall geometry and the fixed-duration directional veto rather than adding more fallback branches. Keep genuine input validity, motor reversal and stop behavior.

## External comparison

- [F1TENTH official wall-following lab](https://github.com/f1tenth/f1tenth_lab3_template): estimates orientation from two beams directed at the same wall, projects distance forward, and adjusts speed with steering. The useful lesson is the measurement geometry and separation of speed/steering; its Ackermann/LiDAR controller cannot be copied directly into this differential-drive ultrasonic robot.
- [WPILib PID documentation](https://docs.wpilib.org/en/latest/docs/software/advanced-controls/controllers/pidcontroller.html): emphasizes consistency between controller update timing and configured period. Our use of distinct acquisition timestamps is worth preserving.

The earlier nine-case simulation result established a narrow idealized regression result, not accurate 90-degree hardware turning or robust 15 cm tracking. The implementation accumulated heuristics before these assumptions were validated.
