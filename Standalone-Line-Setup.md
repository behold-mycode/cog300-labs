# Battery-powered line following

Optional build: `Robot/Robot.ino`, UNO R4 WiFi. The default is laptop USB control.
Change `ROBOT_LINE_ONLY` from `0` to `1` in
`Robot/Robot.h` starts line following 1.5 seconds after power-up or reset.
It does not wait for a laptop, a button, sonar, or moving wheel calibration.
LOW means tape. Physical left/centre/right IR inputs are D4/D2/D3.

## Power

The confirmed pack contains four 1.2 V rechargeable AA cells: 4.8 V nominal.
That is below the UNO R4 WiFi's specified 6–24 V VIN range. Four alkaline AAs
provide 6 V nominal with little headroom as they discharge or the motors load them.
Do not mix alkaline and rechargeable cells in one pack.

Recommended existing-wiring arrangement: keep the AA pack connected to the motor
driver; connect a USB power bank to the Arduino USB-C port. Keep the driver,
Arduino and sensor grounds common. The Arduino powers sensor/driver logic as in
the USB setup. Do not reconnect the old Arduino-5V-to-motor-VS bridge.

A raw AA pack is not a regulated 5 V supply. Do not connect it directly to the
Arduino 5V rail. A suitably rated regulated 5 V buck-boost converter with USB
output is an alternative to a power bank; its capacity and output must be checked
before wiring. Removing only the motor pack leaves the Arduino powered by the
power bank; switch off both supplies for a full shutdown/restart.

Sources: [Arduino R4 power](https://docs.arduino.cc/tutorials/uno-r4-wifi/cheat-sheet),
[Arduino power pins](https://docs.arduino.cc/learn/electronics/power-pins/),
[Energizer battery care](https://energizer.com/about-batteries/battery-care/).

## Upload and run

1. Set `ROBOT_LINE_ONLY=1` in `Robot/Robot.h`. Connect the laptop USB cable and upload `Robot/Robot.ino` for UNO R4 WiFi.
   This build starts automatically after reset, including the reset following upload.
2. Place the car with centre IR over tape, facing along the course. It can also
   search when starting off tape, but centred placement provides a known direction.
3. Replace laptop USB power with power-bank USB power. The board restarts and
   begins again after 1.5 seconds. Keep the motor battery connected for driving.
4. Switch off the motor battery to stop motion; switch off the power bank too
   to shut down the board. Power-up starts a fresh run.

No controller app is required. A deliberate serial `STOP` or controller Stop
cancels the run until reset or explicit `MODE LINE`. USB heartbeat loss does not
cancel a battery LINE run. The desktop controller sends Stop/configuration
commands when connecting, so do not connect it just to keep this build running.
A serial `HELLO` alone is observational in the battery profile.

## Behavior

- Continuous drive, nominal straight PWM 240; moving wheels normally stay between
  PWM 230 and 255. These are provisional duty-cycle settings, not measured speeds.
- Encoder travel balances the wheels. Ordinary corrections move forward, using
  a coasting inner wheel when a tighter curve is needed; pivots are for loss/search.
- Corner memory includes the first side cue and last centre-tape location. The
  150 mm sensor offset guides the approach. The leading outer sensor releases a
  pivot early to allow for motor coast and the existing reversal interval.
- Line loss keeps searching in this battery profile. It changes position between
  sweeps and does not switch into the wall maze or object search.
- A real encoder stall still stops the run. Explicit Stop stays stopped. These are
  faults/operator commands, not routine tape-loss pauses.

Geometry is an estimate from wheel travel, not an absolute angle measurement;
wheel slip and the approximate wheel diameter still require a physical course run.
The 55 mm approach allowance and 400 ms turn-exit interval are model-tested
starting values, not measured stopping calibration for this floor.

## Saved alternatives and verification

Set `ROBOT_LINE_ONLY=0` and rebuild to restore the laptop-controlled profile,
including Lab 07 tape-stop demonstrations and manually started Lab 08 search.
That profile retains its USB watchdog and finite unsuccessful-search stop.
The previous working firmware remains in Git at `dfa9be0`; the earlier milestone
is tagged `lab07-working-high-power-2026-11-17`.

Host replay exercises the production line navigation, PWM mapping, motor reversal
service and signed single-channel encoder interrupts. The plant includes friction,
coast, motor mismatch and quantized 5.105 mm edges. It covers ±45°, ±90°, ±135° and
±150° bends at three motor gain settings, plus finite no-tape recovery in USB mode.
Battery tests cover auto-start, missing sonar, no heartbeat, HELLO, explicit restart,
Stop, first tape acquisition during search and repeated translated search sweeps.
These checks do not certify the real car or arbitrary course layouts.
