# Line control: rebuilt behavior and floor tuning

This branch contains the feedback rebuild. It has host replay coverage and an UNO R4 build; it has not been uploaded or validated on the physical car in this session.

## What changed

The follower requests wheel velocity in **mm/s**. The wheel controller estimates velocity over a rolling 120 ms encoder window, applies a feed-forward motor model, and corrects measured error with PI feedback. The optional running PWM floor defaults to **0**. There is no hidden 230-PWM floor and no timed pulse-driving scheme.

The 130 mm track converts requested yaw to wheel-speed difference. The 150/145 mm sensor offsets and +/-50 mm lateral spacing locate tape contacts and guide corner approach. These are ideal kinematics, corrected continuously by the IR observations; wheel slip and inferred encoder direction still limit absolute pose accuracy.

- A regained forward-facing centre line cancels a corner approach.
- Tape feedback remains active throughout turn capture. There is no 400 ms blind exit.
- Turn cues describe a local region and expire with travel. Geometry rejects overlapping incoming-strip edges; finishing acquisition establishes a new incoming segment.
- Acquisition confirms forward progress between centre hits, allowing ordinary outer-sensor corrections and small gaps. It does not demand 50 mm of uninterrupted centre-only readings.
- During an outer-only correction, the inner wheel coasts. The outer speed request includes the measured inner-wheel speed so residual momentum does not initially turn the car in the wrong direction. Power saturation remains visible; impossible wheel speeds are not guaranteed.
- Search alignment uses an unwrapped heading target, so crossing +/-180 degrees cannot masquerade as reaching the forward direction.
- PI does not accumulate error while a reversal command is waiting to execute. Launch effort is timed from available actuation. Lower-rail saturation can recover even with zero launch power/offset.

Normal line following remains independent of sonar. Explicit Stop, USB heartbeat timeout, and actual encoder-stall detection remain. The motor layer still uses its 250 ms reversal interval and coast stopping; electrical braking was not changed. USB search is bounded by travel, sweep count and a 20-second episode deadline. The optional battery profile retains continuous search as before.

## Default operating points

| Setting | Default | Meaning |
|---|---:|---|
| Straight target | 200 mm/s | Desired wheel speed during centred tracking |
| Pivot wheel target | 90 mm/s | Opposing wheel speeds; ideal yaw about 1.38 rad/s |
| Corrective speed difference | 180 mm/s | Added to the coasting inner speed for outer-only steering |
| Capture speed | 130 mm/s | Lower physical target while establishing the outgoing line; IR steering stays live |
| Feed-forward reference | PWM 240 at 200 mm/s | Initial motor estimate, before feedback |
| Feed-forward offset | PWM 160 | Intercept of the moving-motor model, **not** an enforced floor |
| Optional running floor | PWM 0 | Disabled until you choose a measured value |
| Launch effort | PWM 220 for up to 120 ms | Applies while initial measured speed is below 30 mm/s; not repeated drive/rest pulses |
| PI gains | Kp 0.12, Ki 0.8 | PWM per speed error, and PWM per integrated speed error |

All motor-model numbers are initial estimates, not measurements of your car. A running floor that forces the motors above a requested turn speed removes the controller's ability to achieve that speed. Tune attainable turn/capture speeds together with the floor, rather than forcing every movement to near-full PWM.

## Serial tuning

These commands stop the car before changing settings. They are RAM-only and reset on firmware restart. Defaults live in `Robot/LineFollower.h` (`Config`) and `Robot/LineWheelControl.h` (`WheelControl`) if you want them retained across power cycles. The controller's existing general minimum-power setting is separate from this line-specific floor.

```text
STOP
LINECFG 240 0
LINESPEED 200 90 180 130
LINEMOTOR 200 160 0.12 0.8 220
```

- `LINECFG reference_pwm minimum_pwm`: optional line floor; zero disables it.
- `LINESPEED cruise turn_wheel correction capture`: all four values are mm/s.
- `LINEMOTOR reference_mm_s offset_pwm kp ki start_pwm`: the feed-forward model is `offset + (reference_pwm - offset) * target_speed / reference_mm_s`. The offset must remain below the reference PWM.
- `HELLO` reports `LINETUNE` alongside the existing configuration. In USB mode it also establishes the heartbeat-controlled session; use the desktop controller for driving rather than starting motion from a serial monitor without heartbeat service.

The new commands are available in the firmware serial protocol; dedicated tuning controls have not been added to the browser UI. Do not open Serial Monitor and the desktop controller on the same port simultaneously.

For observation, `LINECTRL` retains its eight-field format: phase, reference PWM, minimum PWM, saturation, heading target, remaining distance/angle, measured left/right speed. `DATA` reports applied PWM and state; `IR` reports the raw sensors. Speed estimates are quantized, not instantaneous precision measurements.

## First floor session

1. Confirm the installed polarity (LOW tape), left/centre/right D4/D2/D3, wheel direction and roughly correct encoder travel. The code already contains the reported dimensions.
2. Measure each wheel's **starting** and **sustained-running** requirements under the car's load. Increase launch power if it struggles only when starting; choose the optional running floor from sustained movement. Keep the global maximum sufficient for steering corrections.
3. On a straight, compare measured wheel speeds with the 200 mm/s target. Tune the feed-forward reference first. A large persistent PI correction indicates a poor motor model; sustained saturation indicates an unattainable target or unsuitable floor.
4. Exercise one left and one right sharp turn. Check that the coasting inner wheel actually slows, the powered outer wheel supplies corrective yaw, and capture speed is sustainable. Then test the troublesome acute bend, gaps and closely spaced opposite bends.
5. Raise cruise speed only after repeated complete-course runs remain stable. Log failures by phase, IR and applied PWM; distinguish a powered wheel failing to move from a logical Stop or a reversal interval.

## Verification scope

The preserved replay contains **162 bend/end-of-line scenarios**: +/-45, 90, 135 and 150 degrees plus no continuation; three motor gains; nominal behavior, doubled coast distance, faster motors, reversed wheel mismatch, higher starting/running friction, and combined faster motors with long coast.

A further **108 course scenarios** cover a 40 mm gap, opposite bends 350 or 220 mm apart, both course orientations, valid on-tape starts with heading errors through +/-0.12 radians and lateral offsets through +/-15 mm, and combined long coast with 3% encoder scale error and 135 mm actual track. They must finish and must not make an extra full rotation. In the final run their completion times ranged below 5.8 seconds for these particular short courses.

Separate regressions cover blind exit, approach reacquisition, stale cues, coasting-wheel curvature, interrupted confirmation, heading wrap, launch timing and PI lower-rail recovery. The complete host suite also checks existing USB, battery, wall/object and browser behavior.

These models do not establish performance on the real floor, arbitrary intersections, arbitrary motor response, or sensor electrical noise. Three binary sensors cannot uniquely identify every junction or absolute chassis heading. The next evidence needed is repeated physical course runs with the tuned motor model.
