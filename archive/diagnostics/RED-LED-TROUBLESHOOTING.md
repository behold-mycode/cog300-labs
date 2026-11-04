# Red LED investigation — 2026-10-15

## Preserved code

- Original: `../src/MyRobot-V1/` (unchanged).
- Additional backup: `../backups/2026-10-15-original/MyRobot-V1/`, with SHA-256 checksums alongside it.
- Working copy: `../src/MyRobot-V2/MyRobot-V2.ino`. All four tabs were copied exactly; only the main filename changed to match the folder. Make future robot changes here.

## Findings

`Safety.ino` configures D12 as OUTPUT and writes HIGH at startup. `applySafetyState()` writes HIGH whenever `frontBlocked` is true and LOW when clear. No other sketch code uses D12. The existing host obstacle test passes, including red-on at startup, obstacle detection, and reverse while blocked. This verifies commanded output, not voltage or physical illumination.

The software defect is not reproduced. A wiring fault, reversed or failed LED, wrong physical pin, or different firmware on the board remains possible. Red is normally OFF with a clear path, even when the robot is stationary. Startup red may last only about 0.15 seconds before three clear readings turn it off.

## Check the current firmware first

With wheels raised, close the controller and open Serial Monitor at 9600 baud with Newline. Place a broad flat target 5–10 cm in front of the sensor and send `STATUS`. `SAFETY OBSTACLE` (or `SAFETY UNKNOWN`) should mean green off and red on. `SAFETY CLEAR` means red off is expected; verify target distance and sensor aim. The 10–20 cm band retains the previous state.

## Isolate the LED circuit

1. With power off, check the documented path: **D12 → 1 kΩ resistor → red anode (+); red cathode (flat side/short leg) → GND**. Check breadboard row connections and any split ground rails. Each LED needs its own series resistor.
2. If the cause remains unclear, open `RedLedCheck/RedLedCheck.ino`, select UNO R4 WiFi, and upload it. This deliberately replaces the firmware on the board until you re-upload the robot sketch; it does not modify any source backup. It holds red on continuously, green off, and motor enables low. No upload was performed during this investigation.
3. If red now lights, the LED and D12 path can work: re-upload V2, repeat the target/STATUS check, and compare the reported state.
4. If red stays dark, switch power off and exchange only the red and known-working green LED components, retaining their resistor branches and polarity. Power on: if green now lights in the D12 branch, the red LED or its seating/polarity is suspect. If neither lights in D12, inspect the D12 wire, resistor, breadboard contacts, and ground return.
5. If available, measure D12 relative to Arduino GND with the diagnostic loaded: it should be near 5 V. A HIGH software command alone cannot establish this. Avoid shorting adjacent pins with the probe.
6. Restore wiring with power off and upload `../src/MyRobot-V2/MyRobot-V2.ino` to resume normal operation.

Do not change LED polarity in software or move pin assignments without confirming the actual circuit. Hardware cause remains unconfirmed until these bench checks are observed.

## Follow-up: replacing the red LED did not help

The user reports that a replacement LED also remains dark. That makes a single failed LED less likely, but does not rule out repeated reversed polarity or a shared wiring fault. The installed UNO R4 WiFi core maps numeric pin 12 to header D12; the sketch does not initialize SPI or otherwise reuse D12. No Arduino USB serial device was present during this follow-up, so live firmware status and pin voltage could not be checked.

A useful next isolation test needs no code change: while the normal robot firmware reports CLEAR and the green LED is lit, power off and replace only the green LED component with the red LED, observing polarity and keeping the known-working D11 resistor/ground path. Power on with the sensor clear. If red lights there, the red LED works; restore wiring with power off. Next, with a confirmed OBSTACLE/UNKNOWN state, move the known-working complete LED/resistor/ground branch from D11 to D12 with power off. If that branch lights, the original red branch has a wiring/resistor/ground issue. If it stays dark, verify the loaded firmware and measure D12; do not conclude the pin is damaged without checking its output.
