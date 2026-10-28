# Distance logging

Both front and right sensors are measured independently. See [current wiring](front-right-wiring.md).

Open Serial Monitor at **115200 baud / Newline**. Periodic logging is on by default, once every 500 ms. Send `LOG OFF` for silence, `STATUS` for one snapshot, or `LOG ON` to resume. Each CSV line includes timestamp, mode, control state, both distances and signed logical wheel outputs. See [field definitions](protocol.md#telemetry).

A distance of `-1` means unavailable: missing echo, outside 2–400 cm, or older than 350 ms. It never means an obstacle is at zero distance or the path is clear. The green LED indicates valid front measurements. Manual motion does not depend on sensors; [autonomous modes](autonomous-modes.md) hold stopped on invalid required readings.

The browser shows distances and a bounded recent log. Expand Diagnostics to view/download it. A saved log records commanded outputs and measured echoes, not proof that wheels moved correctly.
