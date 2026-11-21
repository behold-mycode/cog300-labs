# Current robot wiring and startup

**2026-11-17 update:** today’s build uses laptop USB, front/right sonars, three floor sensors and two encoders. The left sonar/button below are future connections. Standalone is disabled by default. Follow [USB-Lab-Setup.md](USB-Lab-Setup.md) for current startup and measurements; the older startup narrative below is superseded.

This is the consolidated connection plan for the **UNO R4 WiFi, L298N, three HC-SR04 sonars, three LM393 floor sensors, two single-output LM393 slot encoders and a start/stop button**. It supersedes the older USB-bench drawings in `documentation/`. Signal assignments match the current firmware; installation has not been physically inspected.

Open [Robot-Wiring.svg](Robot-Wiring.svg) for the complete overview.

## Connections

| Device terminal | Arduino / destination |
|---|---|
| L298N ENA | D5; remove ENA jumper |
| L298N IN1 / IN2 | D6 / D7 |
| L298N OUT1 / OUT2 | Physical right motor (channel A) |
| L298N IN3 / IN4 | D8 / D9 |
| L298N ENB | D10; remove ENB jumper |
| L298N OUT3 / OUT4 | Physical left motor (channel B) |
| L298N logic 5V | Arduino regulated 5V rail; **5V-EN removed** |
| L298N VS / terminal labelled +12V | Four-AA pack positive; **not Arduino 5V** |
| L298N GND | AA pack negative and common Arduino/sensor GND |
| Front HC-SR04 TRIG / ECHO | A0 / A1 |
| Right HC-SR04 TRIG / ECHO | A2 / A3 |
| Left HC-SR04 TRIG / ECHO | D0 / D11; remove old D11 LED |
| All HC-SR04 VCC / GND | Regulated 5V / common GND |
| Left / centre / right floor LM393 DO or OUT | D4 / D2 / D3 |
| Left slotted encoder DO or OUT | D12 |
| Right slotted encoder DO or OUT | A4, used as a digital input |
| LM393 module VCC / GND | Regulated 5V / common GND for the planned 5V-compatible modules; follow the actual labelled terminals, not connector order |
| Momentary normally-open button | Between D1 and GND; internal pull-up, no external voltage |

AO, if present, is unused. Single-output encoders use **one OUT wire each**, not quadrature A/B wiring. D13 and A5 are free. The firmware counts both rising and falling edges. D0/D1 are therefore unavailable for a separate UART; USB serial is unaffected. A4 is unavailable for I2C SDA.

The old diagrams show Arduino 5V also connected to L298N VS/+12V in the USB-only bench arrangement. **Remove that bridge when the AA pack supplies VS.** Keep the documented 5V-EN jumper removed; the L298N logic terminal is an input in this arrangement. Do not put the motor pack on the sensor 5V rail, and do not join independent regulator outputs.

## Power

Current tethered tests: laptop USB-C → Arduino → regulated sensor/logic rail; four rechargeable AAs → L298N VS → motors; common ground joins them. Confirm the original Arduino-5V-to-VS bridge is removed before connecting the pack.

For a single-pack standalone build, the additional proposed component is a suitable **regulated 5V buck-boost converter** feeding the Arduino through a proper USB power connection. Input comes from the same AA pack, separately from the motor-driver branch. This converter is **not installed or selected**. Four typical 1.2V NiMH cells cannot reliably supply the R4's specified 6V-minimum VIN input directly. The user rejected a separate power bank because of weight.

## Facts needed in the firmware hardware profile

`Robot/HardwareProfile.h` owns the physical profile; there are no app forms. Nine measurements are currently unknown, and zero means unknown rather than an assumed kit size:

1. Left and right millimetres per encoder edge: wheel circumference divided by counted transitions per revolution, or measured travel divided by counts. Both signal edges count; a disc with N openings usually gives 2N transitions, but verify the disc.
2. Wheel track: lateral distance between the wheel contact centre lines.
3. Axle centre to front, axle centre to rear, and half the total chassis width, including protrusions.
4. Front sonar forward offset from the axle centre; side-sonar forward offset and lateral offset from the centreline. Side sonars currently share symmetric mounting offsets.

LM393 identifies the comparator, not wheel diameter, disc pattern or chassis dimensions. These values were not present in the saved drawings. Incorrect values directly affect turning clearance and odometry. The firmware reports `HARDWARE_DIMENSIONS_REQUIRED` until the profile is provided; it must not guess them.

## Robot-owned first start

With a valid compiled physical profile, boot enables the three sonars and both single-channel encoders and tries to restore matching saved calibration. Changing dimensions or the compiled motor polarity invalidates that calibration. Boot never starts motion.

When calibration is absent, the first explicit Start runs a bounded forward measurement in **clear straight space**, followed by a coast-to-stop measurement. It needs all sensors working, at least 650mm of measured clearance ahead of the chassis at the initial check, and side clearance beyond the chassis turning radius plus 30mm. Place the centre floor sensor over a narrow line, or put all three on bare floor; a broad tape patch is not an acceptable bare-floor reference. The observed starting pattern determines tape polarity under that placement assumption.

Single-output encoder direction is inferred from motor commands; it cannot independently prove physical direction or detect slipping. The profile retains the previously established A/right reversed, B/left normal polarity. Verify that unchanged wiring before the first floor run.

The routine ramps motor output, balances forward travel with encoder feedback, measures each wheel's response, measures coasting, chooses conservative speed/braking values and saves them. It has travel, time, sensor freshness, encoder-count consistency and drift checks. Stop, disconnect in a tethered session, or the physical button cancels it. The check itself moves the robot after Start; it is not a stationary sensor test, and its synthetic tests are not a physical safety guarantee.

Afterward the same Start enters the maze solver. It follows usable lines, changes to wall navigation on sustained line loss, and reacquires lines in corridors. Dead ends, route recording and backtracking use the onboard graph. Ambiguous localization or insufficient clearance still stops rather than claiming a successful route. The user stops and picks up the robot at the goal; there is no goal detector.

## What has actually been verified

Host tests exercise fresh startup without app calibration commands, calibration persistence tied to dimensions, cancel/failure cases, and a synthetic complete startup → line → wall → dead-end → return sequence with full chassis collision assertions. The mechanics, sonar rays and dimensions in that test are synthetic. The test now staggers sonar acquisitions by at least 70ms, rather than supplying every ray on every loop. Interrupt tests cover cancellation in ramp, measurement and coasting. No new firmware has been uploaded and no real run is claimed. High breakaway PWM, nonlinear motor response, sonar beam effects and real traction still require physical validation; full-speed response alone does not establish controllable low-speed turns.
