# V2 distance logging and wall-following milestones

MyRobot-V2 is the working version. Keep V1 as the previous version.

## Milestone 1: readable distance log

1. Close the Python controller so Arduino IDE can use the serial port.
2. Open `src/MyRobot-V2/MyRobot-V2.ino`, with its Motor, Serial and Safety tabs.
3. Select UNO R4 WiFi and the robot's USB port, then upload.
4. Open Serial Monitor at **9600 baud**, select **Newline**, and send `LOG ON`.
5. Send `LOG OFF` to stop logging. Logging defaults to off after reset; connecting the Python controller also turns it off.

Example (illustrative, not a hardware measurement):

```text
Distance log ON (front/right cm, every 500 ms)
t=1.0s | front=30.2 cm OK | right=20.1 cm OK
t=1.5s | front=30.0 cm OK | right=20.0 cm OK
t=2.0s | front=-- INVALID | right=20.2 cm OK
```

Time is elapsed since reset. Distance is the latest raw echo converted with the existing `microseconds / 58` formula, rounded to one decimal place. Decimal resolution does not establish accuracy. Invalid or stale readings display `--`, never zero or a previous valid distance. Sensor-validity messages and command responses remain visible alongside the optional stream. `OK` describes a valid measurement.

The sensors alternate one ping every 70 ms, so each updates every 140 ms. Samples expire after 350 ms independently. Logging shows one snapshot every 500 ms and does not perform extra measurements. Sensor-based motion blocking is disabled. Near objects, missing echoes and invalid/stale readings do not stop motors or cancel tests. Green indicates a valid reading, including close readings; off indicates no valid reading. Manual STOP, finite movement durations and controller timeout still stop motion. Logging neither starts nor stops motion; use `STOP` to stop the robot.

## Milestone 2: establish measurement accuracy

With the robot stationary, use a broad flat target perpendicular to the sensor. Measure from the sensor face at 10, 20, 30, 50 and 100 cm. Collect at least ten displayed readings per position. Record reference distance, mean reading, minimum/maximum and invalid count. Repeat selected distances to check consistency before changing the conversion formula or adding filtering. Test angled surfaces and missing echoes separately. Hardware accuracy is not verified by software tests.

## Later milestones: wall following and P control

The objective is wall following, not following a person. Before implementing it, confirm wall side, sensor orientation, desired wall distance and how forward obstacle detection will be preserved if the sensor is repositioned.

Then validate any filtering, implement an explicitly enabled wall-following mode with manual stop and a stop on lost measurements, and tune proportional steering at a fixed low base speed. Define distance error as measured minus desired wall distance; verify the steering sign for the chosen wall side before tuning gain. Increase gain gradually, observe correction and oscillation, and record distance error, gain and wheel commands. Keep speed, target distance and test course fixed while comparing gains. This tuning is a separate milestone, not implemented here.

## Version control

Use one focused branch and regular GitHub PR per milestone. PR descriptions record behavior, software validation and outstanding hardware checks. Merge after review; use new branches for subsequent milestones rather than duplicating V2 into V3 for each change.

## Software checks

Run from the repository root:

```sh
python3 -m unittest discover -s labs/lab-03/controller -v
for name in robot obstacle distance_log dual_sensor; do
  c++ -std=c++11 -Wall -Wextra -Werror -I labs/lab-03/tests labs/lab-03/tests/test_${name}.cpp -o /tmp/cog300-test-${name} && /tmp/cog300-test-${name} || exit 1
done
arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi labs/lab-03/src/MyRobot-V2
```

Both sensors are active: front TRIG/ECHO A0/A1, right TRIG/ECHO A2/A3. Connect both VCC pins to regulated 5V and both grounds to common GND. Each log column independently shows INVALID for a missing, out-of-range or stale reading. Test each against a ruler separately, then together to check cross-talk. Green still indicates front validity only.
