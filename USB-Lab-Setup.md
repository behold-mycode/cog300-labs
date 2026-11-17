# USB robot: Lab 07 first, Lab 08 prepared

Updated 2026-11-17. Current target: UNO R4 WiFi, laptop USB, L298N, three floor IR modules, two single-output encoders, front/right sonars. The left sonar is **not installed**. No firmware was uploaded and no physical run was performed during this review.

## What to install today

Follow [Robot-Wiring.md](Robot-Wiring.md) for the existing motor/power connections. Floor sensor outputs: left **D3**, centre **D2**, right **D4**. Encoder outputs: left **D12**, right **A4**. Keep front/right sonar assignments unchanged. D0/D11 are reserved for the future left sonar; no button or Wi-Fi is needed for USB operation.

USB supplies the Arduino and sensor/driver logic. The motor pack supplies L298N VS. The old Arduino-5V-to-VS bridge must be absent when the pack is connected. This review did not change the physical wiring.

## Measurements to send

Use millimetres, measured from the axle centre unless stated otherwise:

- Left/right wheel circumference and encoder transitions for one revolution, or measured travel/counts. Firmware counts rising **and** falling edges.
- Wheel contact-centre spacing (track).
- Front and rear extents, and overall width including protrusions.
- Axle to the floor sensors' detection row; also send left–centre/right–centre spacing, mounting height and tape width to assess whether the layout can detect the course corners.
- Front sonar forward offset; right sonar forward and lateral offsets.
- Each IR module's raw tape and floor levels, plus confirmation of forward motor direction.

`Robot/HardwareProfile.h` contains the compiled physical measurements. `leftSonarInstalled=false` is deliberate. Adding the left sonar later requires its installation flag and symmetric mounting offsets to be checked. A changed physical profile invalidates saved calibration.

Measurements reported 2026-11-17: wheel spacing 130 mm, wheel diameter approximately 65 mm, 20 encoder slots, overall width 160 mm, and right sonar 15 mm inward from the rightmost edge. These give an initial wheel scale of approximately 5.105 mm per counted edge and a right-sonar lateral position 65 mm from the centreline. These values are entered; wheel scale still needs verification with actual travel/counts.

Final body measurements: length approximately 210 mm, axle 140 mm behind the body front, front sonar and centre IR 10 mm ahead of the body, outer IR sensors approximately 5 mm ahead. The right sonar midpoint is 95 mm behind the body front. Therefore the entered axle-based profile is: front envelope 150 mm (including the protrusion), rear 70 mm, centre IR 150 mm, front sonar 150 mm, right sonar forward 45 mm and lateral 65 mm. Required profile dimensions are now populated.

Confirmed IR positions are 0, 50 and 100 mm across the front. The course tape is 50 mm wide on straight sections, with different geometry at turns. The expanded simulation uses 50 mm-wide joined strips with overlap at bends; the actual turn shapes still require a floor run. The centre IR is 5 mm ahead of the outer pair. Firmware recovery uses the centre detection position and now preserves the selected pivot direction until the centre reacquires tape, avoiding opposite outer sensors reversing each other on a tight bend. Recovery sweeps observe both tape edges and aim back toward the middle before resuming; incomplete bands at sweep limits are discarded. Both narrow and wide synthetic layouts complete ±45°, ±90°, ±135° and ±150° cases. This is not physical validation.

## Sensor check and calibration

1. With wheels off the ground, upload `IRCalibration/IRCalibration.ino` if the new floor modules need tuning. It keeps motors disabled and reports all three pins. Put all three over white tape and send **T**, then over floor and send **G**. Every sensor must change state; the current firmware expects the same tape polarity across all three modules. Tune at the installed height.
2. Once the measured profile is filled, upload `Robot/Robot.ino`. Close Serial Monitor before using the desktop controller.
3. Launch `Launch Robot Controller.command`, connect, and choose **Wheel calibration · clear floor**. Start only in clear straight space, with at least 650 mm ahead of the chassis and side/pivot clearance. This routine drives forward up to 300 mm, measures wheel response and coasting, saves calibration, and stops. Initial placement is centre sensor on a narrow tape line, or all sensors on bare floor. A wide patch of tape is not a bare-floor reference.
4. For the first **Line following** start, missing calibration also invokes that same moving setup before following the line. Prefer explicit calibration first. **Object search** and the two tape-stop demonstrations require completed calibration; they never silently run that forward setup.
5. Test low-speed forward motion and measured turns on the actual floor. Full-speed wheel response does not prove low-speed breakaway behavior. If it stalls, overshoots or reports a fault, capture the reported state and counts before changing parameters.

## Laptop modes

| Mode | Behavior |
|---|---|
| Line following · Lab 07 | Encoder-regulated line tracking; outer sensor pivots; bounded measured recovery at loss; no dependency on the missing left sonar; stops after an unsuccessful search rather than switching to walls. |
| Drive until tape · Lab 07 | Drives forward and stops on any floor sensor detecting tape. |
| Stop at tape end · Lab 07 | Starts on tape, tracks it, stops at the first all-off reading. Use Line following for bends requiring recovery. |
| Object search · Lab 08 | Explicit laptop start. Samples forward through +180°, then back through −180°, chooses the nearest median range, aligns, verifies, and approaches to 20 cm ahead of the chassis. Any tape detection stops the run. |
| Line + wall maze | Requires the third sonar. Exhausts line recovery before checking for a corridor. Preserves wall graph across temporary line guidance. |
| Wheel calibration · clear floor | Forward measurement and coast, saves and stops. |
| Manual driving | Hold arrows/WASD; release to stop. |

Selecting a mode stops motion; **Start** runs it. Space/Escape stops. Leaving the controller window or losing browser/USB heartbeat stops the car. Standalone button/Wi-Fi entry points are disabled in the default build (`ROBOT_ENABLE_STANDALONE=0`).

## Sonar range and open space

Side navigation distances are capped at **400 mm / 40 cm**. A farther finite echo or a completed no-return pulse means open beyond this bound, not a sensor failure. Close distances remain exact. The raw sensor telemetry is retained for diagnosis; it can therefore display values beyond 40 cm. Front range is separate and retains the existing approximately 4 m measurement limit for obstacle control and object search.

The driver is nonblocking, has a 45 ms echo deadline, and staggers pings. It does not wait forever in open space. A completed long no-return echo is distinguished from a missing/stuck signal. Different HC-SR04 clones can behave differently with no target; verify the actual modules' open-space output. Stale/broken front observations still stop motion. Lab 07 line tracking ignores side ranges; the future wall solver uses them.

## Requirements checked and remaining limits

Sources retrieved 2026-11-17: [Lab 07](https://cogs-300.github.io/labs/lab07), [Lab 08](https://cogs-300.github.io/labs/lab08).

Lab 07 asks for multiple tuned IR sensors, tape start/stop, corrective turning, and completion of the line course. These behaviors are now exposed in the app. Sharp-corner geometry is tested synthetically through 150° left and right; that does not guarantee every physical tape layout or angle. Broad tape is crossed slowly; LINE is a follower, not an exhaustive branch-searching tape-maze planner. The existing separate MAZE/graph code remains for that older purpose.

Lab 08 asks for transitions, an angle-resolved sweep toward the nearest object, and acquisition from arbitrary orientation. Object-search start is manual as requested; automatic wall-to-object transition remains undefined until the actual course cue is supplied. The extra boundary behavior stops on tape. This is not proof that every chassis corner stays inside an arena: initial pivot clearance, sensor placement and stopping distance must be verified. A front sonar cannot distinguish a target from an arena wall; the selected target is the nearest detected reflector. Thin targets between angular samples or weak echoes need physical validation.

The line and object modes assume an open floor area with pivot clearance. Their front ray cannot certify unobserved rear/side space. Single-channel encoders infer direction from commanded motor polarity and cannot independently detect slip or being pushed. Tight wall turns require enough physical room for the chassis, regardless of code.
