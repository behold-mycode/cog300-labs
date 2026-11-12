# Observation-first redesign implementation record

User approved the observation → uncertain estimate → feasible action → feedback architecture and explicitly authorized replacing rigid code. Implemented locally without commits or uploads, preserving the dirty checkout.

- Replaced Explorer's POSITION/full-360-SURVEY/ALIGN sequence with evidence-led tracking, measured settling, directional bounded search, candidate verification, guarded approach to a previously observed tape point and explicit return checking.
- Added WorldModel: timestamp/order handling, Bayesian prediction/likelihood/normalization, correlated-stationary-sample rejection, measured wheel speed and explicit heuristic uncertainty allowances.
- Added Dynamics calibration and command-level stopping-distance guard. Checks apply to the first proposed turn, not only after the behavior has entered a turning state.
- Retained graph storage as verified route-checkpoint memory. Unknown exits are not invented; exhaustive junction discovery and arbitrary cyclic-maze completion remain unproven and are documented, not claimed complete.
- Added protocol 6, controller model/dynamics readiness, belief telemetry, and explicit 20,000-row CSV capture without rolling overwrite.
- Closed-loop geometric tape-course tests exposed too-tight alignment tolerance and rejecting a return line with an adjacent sensor also on tape. Both were corrected against sensor footprint geometry.
- Validation: host checks and target build; final results recorded in user response. No physical test or upload.

Design references discussed with user: roboticsbook.org/S44_logistics_perception.html (Bayes prediction/update); Pololu QTR sensor/PD examples (feedback from calibrated reflectance, with our binary-sensor limitation); Nav2 collision monitor (separate fresh-observation intervention). These motivate architecture, not calibration values or a claim of reproducing those systems.

Final verification: 7 C++ executables, 20 Python tests, browser events, target compile and git diff --check passed. Target: 93,716 bytes flash / 12,908 bytes RAM. Remaining: measured encoder/braking/IR traces and complete junction-exit discovery; no claim of race readiness.
