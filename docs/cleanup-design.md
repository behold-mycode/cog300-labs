# Robot controller and firmware cleanup

## Purpose

Make the existing UNO R4 WiFi car easy for teammates to connect and drive, and
make its firmware understandable enough to debug and tune during lab. Preserve
the existing wiring. Replace accumulated compatibility paths and overlapping
motion state with a single documented implementation.

## Agreed requirements from the request

- Keyboard driving: forward, reverse, moving turns, and in-place turns.
- Independently reversible wheel direction for the current wiring.
- Default available motor output of PWM 255; easily reduce power for batteries.
- Separate straight-line object-distance and right-wall-following modes.
- Use the existing front and right ultrasonic sensors.
- Readable, rate-limited serial logging.
- Minimal interface with connection and driving as the primary workflow.
- Resolve missing Mac GUI dependencies and establish GitHub sync status.

## Implementation choices proposed by the assistant

Use a local browser page served by Python rather than Tk. Retain only pyserial
as a Python package dependency, installed by the launcher into a local virtual
environment. Python 3 remains required; report missing Python or installation
failures with actionable instructions. Support Safari, Chrome, and Firefox via
the local Python serial bridge; do not depend on browser Web Serial support.
Bind the server to loopback, validate requests, and allow one controlling page.

The main page contains a port selector and Connect, a large keyboard-control
area with a stop button and concise status, and side buttons for Manual,
Follow object, and Follow right wall. Show front/right distances. Put power,
target distances, and independent wheel inversion under Settings. Default power
is 255, left inversion false, right inversion true. Settings changes stop motion.
No enable step, preset dashboard, trim multipliers, or permanent log console.
Provide log download and a small expandable diagnostics view.

Arrow keys and WASD drive only while the page is focused and no settings field
is being edited. Release stops manual movement. Space/Escape stops and exits
autonomous mode. Focus loss, hidden page, disconnection, stale browser updates,
or stale serial communication stop motion. A fresh key press or mode selection
is required afterward; stale held inputs cannot restart the car. Connection
checks firmware compatibility before accepting motion commands.

## Firmware organization

Keep Robot as the working sketch. Use clearly named modules for configuration,
motor output, sensors, navigation, protocol, and telemetry. Keep main setup/loop
short. Remove old single-letter command aliases, duplicated test scheduling,
obsolete obstacle-stop code, and legacy GUI protocol paths. Replace affected
tests and documentation together. Retain V1 as explicitly historical code;
leave untracked backups and diagnostics untouched.

Pins remain left ENA/IN1/IN2 D5/D6/D7; right IN3/IN4/ENB D8/D9/D10;
front TRIG/ECHO A0/A1; right TRIG/ECHO A2/A3; green LED D11.

One owner controls motion at a time: manual, object follow, or wall follow.
Motor output clamps to configured maximum and applies independent inversions.
Use an unpowered settling interval on direction reversal without restarting
that interval on every repeated command. Stop cancels pending output as well
as current output. Startup always stops.

Retain alternating nonblocking ultrasonic acquisition to avoid sensor crosstalk.
Reject missing/out-of-range/stale readings; missing echo must never mean zero
distance or open space. Manual driving remains independent of sensor readings.

Object follow uses the front sensor only, an adjustable target (default 20 cm),
and a small deadband. Proposed behavior: advance when too far, reverse when too
close, stop within the deadband or when target is lost. Both wheels receive the
same logical command. Use bounded proportional control with a configurable
minimum moving PWM for weak supply; default minimum equals maximum for full
output when movement is required. With batteries, lowering the minimum permits
smoother proportional approach. Document that distance regulation at full output
requires physical tuning and may oscillate.

Wall follow uses right-distance error for steering and front distance for blocked
path handling. Default right target is 20 cm. At maximum speed, slow the inside
wheel to steer rather than requesting output above 255. A front obstruction
triggers a bounded left turn; stop if front clearance cannot be recovered before
the turn timeout. Stop on invalid front/right readings or lost wall rather than
blindly searching. This is a local wall follower, not a complete maze solver.

Use one documented line-based protocol with version handshake, watchdog,
manual drive, mode, settings, stop, status, logging, and timed individual-wheel
test commands. Keep timed wheel tests in Serial Monitor for repeatable Lab 3
checks without adding dashboard buttons. An expired GUI session rejects delayed
motion until a new handshake. Serial Monitor supports standalone operation.

Telemetry prints timestamp, mode/state, front/right cm (or explicit invalid),
and motor commands at 2 Hz by default. Logging can be turned off; status requests
produce one snapshot. Do not print each control tick, ping, or heartbeat as
human-readable log traffic. Bound buffers and serial work per loop.

## Verification and delivery

Test real firmware logic with host Arduino stubs: pin mapping, inversion, output
limits, reversal settling, stop, watchdog, parser errors, stale sensor readings,
distance deadband, wall steering direction, and bounded corner turns. Test the
Python serial bridge with fake hardware, including page loss and connection
failure. Compile for arduino:renesas_uno:unor4wifi with the installed core.
Exercise the browser interface in simulation when tooling allows.

Update quick-start instructions, protocol, tuning notes, and hardware test steps.
Distinguish software verification from real-world tests. No firmware upload or
physical motor movement is part of this cleanup without a connected test setup.
Implement on a new branch based on the latest working code. GitHub review can
be prepared without merging the outstanding branches into main.

## Repository findings (2026-10-28)

Fetched origin successfully. Local HEAD a046510 exactly matches
origin/lab-03/proportional-following. Seven branch commits are absent from main.
Open PRs #3 through #7 contain the accumulated revisions; #7 is the latest.
Untracked backups/ and diagnostics/ are local only. Thus the previous working
changes were pushed but were not merged into the default branch.

## Course scope

Lab 3 requires forward/reverse, moving turns, spins, calibration, and repeatable
testing: https://cogs-300.github.io/labs/lab03 . Lab 4 asks for encoders and readable
telemetry: https://cogs-300.github.io/labs/lab04 . Encoder implementation is outside
this request because the specified installed sensors are ultrasonic; this cleanup
does not claim to complete all Lab 4 requirements.
