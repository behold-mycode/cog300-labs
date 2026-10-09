# Lab 3 test procedure

Use the Mac controller described in [controller/README.md](controller/README.md). This framework demonstrates the lab concepts, but physical results must be observed and recorded.

## Bench checks

With power disconnected, secure the previously disconnected motor wire and check all terminals. Raise the wheels. Upload firmware 2, close Serial Monitor, connect the GUI and enable controls. Start with left/right trim 100/100. Use **Wheels (F1)** to test left and right separately. Confirm that each would propel the chassis forward. Correct an individual direction with its Reverse direction checkbox, Apply, and enable again.

Use Space to stop during a Wheels test; verify no later step starts. Changing settings, disconnecting, closing the app or switching to another application should stop/cancel and disable control. Verify this first with raised wheels. The firmware also enforces a 500 ms controller heartbeat timeout, tested in simulation; any physical connection-loss demonstration should keep the wheels raised and preserve board power only through the already-validated supply arrangement.

## Required movements

After directions are confirmed, demonstrate these on the floor with enough room for the USB lead:

| Preset / hotkey | Expected observations |
| --- | --- |
| Straight / F2 | Forward then reverse; record drift |
| Turns / F3 | Left and right moving curves; inner wheel has half nominal PWM |
| Spins / F4 | Left and right in-place rotation with opposite wheel directions |
| Full lab / F5 | Wheels, Straight, Turns and Spins in sequence |

Each step uses displayed PWM/duration. Initial values are PWM 255 and 500 ms, with a 1-second stopped gap and a 250 ms settling period when beginning a new movement. Full lab has eight steps. Stop and cancellation must work at every stage. Function keys may require Fn on a Mac; buttons perform the same tests.

## Repeatability

Perform three trials with the same power source, surface, PWM, trim, inversion flags and test duration. Record conditions and actual behavior in `test-results.csv`; use the name column to match GUI log descriptions. The existing sheet also has isolated reverse-wheel rows from the earlier ten-step suite; mark those NOT TESTED unless run separately. Do not replace already-recorded measurements with assumed outcomes.

Save the serial log using Save log. Each test has a request ID and board start/completion acknowledgment. Completion means the command ran to its deadline, not that wheels rotated or distance/angle targets were met. Pick a straight-line drift tolerance before measuring repeated forward runs against a marked line.

## Calibration

The user identified a disconnected wire as the likely cause of left-wheel failure. Retest after fixing it before applying compensation. At PWM 255 a weak right motor cannot be boosted further by software. Check friction, wiring and supply if either is weak alone. If both run well but differ in speed, reduce the faster wheel's trim in small increments; too much reduction may fall below its starting threshold.

Use the visible settings and log to record actual PWM and reversal flags. Apply stops the car and requires re-enabling controls. Settings are sent on connection but not saved across app restarts. Recheck on the floor under load. These are open-loop PWM commands, not measured wheel speed; no encoders are used.

## Demonstration and submission

Show the controller and robot, name the test, and show observed outcomes. Include a cancellation and the calibration performed. Explain relevant limits such as USB motor power, drift and coasting stop. Complete the video, repository link/usage instructions, BOM, circuit diagram and team agreement according to [Lab 3](https://cogs-300.github.io/labs/lab03#7-robust-testing-homework).
