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

- Physical wheel-speed control: 200 mm/s straight target, 90 mm/s pivot wheel target, and 130 mm/s capture target. The optional line PWM floor defaults to zero.
- Encoder feedback balances speed; the outer-wheel target accounts for the coasting inner wheel during a strong correction.
- Geometry guides approach and rejects old incoming tape. Reacquisition cancels obsolete approach, and steering remains responsive throughout capture.
- Line loss keeps searching in this battery profile. It changes position between sweeps and does not switch into the wall maze or object search.
- Encoder stall and explicit Stop still stop the run.

See [Line-Control-Tuning.md](Line-Control-Tuning.md) for settings and floor tuning. The 250 ms motor reversal interlock remains; the former 400 ms blind turn-exit interval is removed. Geometry is wheel-derived, not an absolute heading measurement.

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
