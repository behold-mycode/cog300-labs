# Serial protocol 4

115200 baud, ASCII lines, Newline. Case-insensitive command words. Boot is stopped.
This intentionally replaces protocol 3 and its legacy aliases; update firmware and controller together.

| Command | Effect |
| --- | --- |
| `STOP` | Cancel mode, timed test and pending output; coast stop |
| `HELLO` | Stop and open/reopen a desktop watchdog session; replies `ROBOT_READY 4` |
| `HEARTBEAT` | Refresh active session; replies `ALIVE` |
| `DRIVE left right` | Desktop-only manual command; signed integers −255…255, limited by power |
| `MODE MANUAL` | Stop and return to keyboard control |
| `MODE FOLLOW` | Straight-line front-distance control |
| `MODE WALL` | Right-wall tracking with front clearance |
| `CONFIG power minimum invertA invertB followCm wallCm` | Stop, validate and apply all settings atomically; replies `OK CONFIG` |
| `TEST left right ms` | Standalone bounded wheel test; duration 1…5000 ms, signed wheel values −255…255 |
| `STATUS` | One telemetry snapshot |
| `LOG ON` / `LOG OFF` | Enable/disable periodic telemetry (on by default) |
| `HELP` | Print command reference without changing motion |

Configuration example: `CONFIG 255 255 1 0 20 20`.
Power: 1…255; minimum: 0…power; inversion: 0/1; object target: 5…100 cm; wall target: 5…80 cm. Minimum applies only to object following; wall steering may slow the inside wheel below it. Configuration changes enforce a 250 ms unpowered settling interval before later motion.

`TEST` duration begins when motor output actually starts (after any reversal pause). It emits `TEST_START` and `TEST_DONE`; these mean commanded execution, not physical success. A zero/zero test stops. The GUI uses live drive commands, not timed tests.

Desktop sessions expire after 500 ms without HEARTBEAT or valid DRIVE. Expiry cancels everything and emits `CONTROL_TIMEOUT`. Subsequent motion/config commands are rejected until HELLO. A desktop heartbeat must continue during autonomous modes as well. Serial Monitor may use MODE/TEST without a desktop session after board reset. Reset after disconnecting the GUI before standalone testing; do not send HELLO unless you will maintain heartbeats.

Malformed/oversized commands stop the robot and emit ERROR. Partial input expires after 1 second and is discarded through the next newline. `!` immediately stops while discarding the rest of its line. Input processing is bounded to 32 bytes per loop.

## Telemetry

Default cadence: one line per 500 ms; no per-ping/per-control-tick prints.

```text
DATA,ms,mode,state,front_cm,right_cm,left_pwm,right_pwm
DATA,12500,FOLLOW,APPROACHING,35.00,20.00,255,255
DATA,13000,FOLLOW,FRONT_INVALID,-1.00,20.00,0,0
```

The header above documents fields (it is not automatically printed). PWM is the signed logical output after the power cap, before electrical wheel inversion. Missing, out-of-range and stale readings are `-1`. `STATUS` works with periodic logging disabled. Desktop ALIVE acknowledgments are protocol traffic and are omitted from the GUI's human-readable log.
