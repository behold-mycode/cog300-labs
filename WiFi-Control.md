> Current build: direct-rollback-v1. Continuous-live-v3 was withdrawn after the user's failed physical startup tests. The earlier direct driving stack is restored; the three live sliders, path drawing and labelled run export remain. Historical deployment records below are not current behavior.

**Working checkpoint: `direct-rollback-ir-v1` (2026-11-18).** After correcting the centre IR sensor, which had been reporting floor even over tape, the user reports that the installed robot now works very well. This checkpoint matches that installed direct follower and preserves the GUI, live IR indicators, tuning and path tracing. The later encoder yaw-damping candidate was never uploaded and is excluded. Earlier startup failures occurred with the misadjusted centre sensor, so they cannot establish that the driving algorithm caused those failures.

# Wi-Fi and USB control

The UNO R4 WiFi hosts its own password-protected network. Join the SSID configured in `Robot/WirelessSecrets.h`, then open **http://192.168.4.1**. No router or internet is needed. Credentials remain in that ignored local file; do not commit it.

1. For battery operation, switch on the shared supply: switched battery positive branches to Arduino **VIN** and L298N **VS/+12V**, with common ground. USB is not required. Keep the L298N **5V-EN jumper removed** and Arduino regulated **5V → L298N logic 5V**. Confirm the battery voltage is suitable for both the Arduino and motors before powering up. USB power remains an option for bench use.
2. Disconnect the desktop controller's serial session if it is connected. The USB cable may stay plugged in for power.
3. Open the page, choose **Take Wi-Fi control**, place the centre sensor over the tape, then choose **Start line following**.
4. Keep the page visible. **STOP** works regardless of who owns control. After Stop, explicitly take control again before starting.
5. To use USB again, release Wi-Fi control (or Stop), then connect the existing desktop controller. USB uploads and serial diagnostics are unchanged.

The page shows normalized tape contacts, measured wheel speeds, applied PWM, navigation state and line-controller phase (current source: 0 stopped, 1 following tape, 3 search). Speed sliders send the four speeds when released, or through **Apply speeds**. A valid speed change from the owning Wi-Fi page applies while line following continues; changing speeds while stopped does not start the car. **Take Wi-Fi control** enables the sliders. Stop, release and connection loss disable them. The page shows the values acknowledged by the robot. Advanced motor calibration/PWM changes still stop the car. Tuning is RAM-only and resets on reboot. An optional PWM floor remains available, default zero.

The four speeds are **Straight speed**, **Lost-line outside wheel**, **Correction outside wheel**, and **Gentle correction inside wheel**. Keep the gentle inside speed below the correction outside speed to produce a turn. These retain the existing `LINESPEED` wire format; serial/USB speed changes still stop the robot.

Wi-Fi heartbeat expiry stops the motors within 1500 ms of the last accepted heartbeat. USB retains its 500 ms timeout. A hidden page sends Stop and ceases heartbeats. Expiry never falls back automatically to another connection or resumes motion. Delayed commands carrying an expired token are rejected. A Wi-Fi Stop also expires a USB session so buffered USB drive commands cannot restart the car; reconnect that desktop controller when ready.

The firmware boots stopped and starts its Wi-Fi access point automatically, including when powered through VIN with no USB cable. Keep `ROBOT_ENABLE_WIFI=1`, `ROBOT_LINE_ONLY=0` and `ROBOT_ENABLE_STANDALONE=0` for this mode. `ROBOT_LINE_ONLY=1` means automatic line-following after boot; it is not needed for battery-powered Wi-Fi control. The USB watchdog applies only to an active USB control session; Wi-Fi has its own heartbeat timeout.

For a fresh untethered start, disconnect USB, switch the battery supply off and on, join the robot network, and open **http://192.168.4.1**. Choose **Take Wi-Fi control**, then **Start line following** when ready. Keep the page visible and use **STOP** to stop. The current Wi-Fi page controls line following and its tuning; it does not provide manual steering or wall-maze commands. OTA firmware uploading is not included; firmware uploads still use USB.

## Live path and recording

The **Live path** panel draws the estimated position and heading as telemetry arrives; it does not wait for the run to end. Polls are scheduled 250 ms after each response (up to four updates/second; actual rate depends on Wi-Fi response time). The scale fits the current trace automatically; the grid label shows its spacing. Position uses encoder odometry, not a camera or measured tape map. Wheel slip, wheel dimensions and inferred encoder direction affect accuracy.

Recording starts when the page opens. **Pause trail** pauses storage while the robot marker keeps updating. **Clear trail** clears this page's samples and places its display origin at the current position; it does not reset robot navigation. **Download run** saves JSON containing robot/browser timestamps, estimated pose, IR readings, PWM, measured speeds and tuning values. Recording is held in browser memory (up to 20,000 samples); download before reloading or closing the page. The page reports when the recording limit is reached.

Connection gaps and recording pauses break the plotted line. Robot restarts and odometry reconfiguration start new coordinate frames; old frames remain in the download, but only the current frame is drawn. A stale connection greys the robot marker and shows a waiting/lost status. Keep the page visible while driving.

## Building the page

Edit `Robot/WirelessPage.h`, then run `python3 build_wifi_page.py` before compiling. The generated `Robot/WirelessPageGzip.h` reduces page transfer from 14,277 to 5,632 bytes so a page load occupies the radio for less time. `tests/run_checks.py` checks that the compressed page matches the source. Firmware HTTP tests verify the complete compressed body, and the browser check loads the gzip response. This reduces single-client transfer latency; it does not guarantee timing on a congested physical radio link.

The current build restores the geometric follower. Its four known course-model failures are documented in [Line-Control-Tuning.md](Line-Control-Tuning.md); live tuning and plotting do not establish reliable physical cornering. OTA updating is not implemented; installation of this page requires a USB firmware upload. Subsequent slider adjustments do not.

## Implementation and validation plan

- Keep the existing line controller and USB protocol intact; add explicit ownership around mutations.
- Use WiFiS3 only at stopped startup to configure the AP and server. Its synchronous modem waits are unsuitable inside the driving loop.
- During operation use the installed ESP bridge AT protocol incrementally: at most 16 UART transmit bytes and 128 buffered receive bytes per service call, and at most 256 HTTP payload bytes per modem transaction. UART transmission is approximately 1.4 ms for 16 bytes at 115200 baud; this is not a measured whole-loop deadline.
- Preserve encoder interrupts and the normal navigation loop. Network responses never gate the next control iteration. Radio replies have a 2-second failure deadline; the independent Wi-Fi control lease expires sooner. A radio failure disables Wi-Fi until reset and leaves USB available.
- Verify parser framing (including binary payloads and bare send acknowledgements), ownership, stale commands, Stop, time rollover, stalled modem, fragmented HTTP and page transfer. Run the existing line replay matrix and full repository suite. Compile the UNO target, review independently, then upload stopped and check physical AP/USB coexistence. No automatic motor test.

Transport format was cross-checked against the installed Arduino renesas_uno 1.6.0 WiFiS3 sources and Arduino's [ESP bridge implementation](https://github.com/arduino/uno-r4-wifi-usb-bridge/blob/main/UNOR4USBBridge/cmds_wifi_netif.h). This does not change or reflash the ESP bridge firmware.

## Work record

- Parser and ownership tests first failed before implementation, then passed.
- Cross-checking the official bridge caught the bare `OK` send acknowledgement; corrected and covered by a failing-then-passing parser test.
- Wi-Fi Stop now invalidates a USB lease; a regression test confirmed buffered drive rejection.
- Existing unrelated edits in `Line-Control-Redesign.md` are preserved.
- Independent review: fixed case-insensitive HTTP header handling, claim cancelling battery startup, physical Start respecting Wi-Fi ownership, observer Stop messaging, and serial backpressure. Regression tests pass. Tuning fields refresh on each new claim.
- USB output now uses a bounded queue of complete records and drains at most 16 UART bytes per loop. No change to the USB wire format. The review raised native USB backpressure, but hardware inspection established that UNO R4 WiFi is built with `NO_USB`: its USB connection uses the ESP bridge UART. Its UART does not implement `availableForWrite`; testing caught and corrected that distinction. The queue bounds telemetry work rather than waiting for a host acknowledgement.
- 46 C++ test programs, 26 Python tests, existing USB browser events and the optional real-Chrome Wi-Fi page checks pass. Default UNO build passes (about 147 KB flash, 20 KB global RAM).
- Final firmware uploaded stopped; USB `WIFI` reports `WIFI_READY`, USB `STATUS` confirms zero motor outputs. Observed maximum radio-service time was 2202 microseconds while stopped. This is not a moving-car latency measurement.
- User confirmed the real phone/browser page loads. No physical driving was initiated by the upload/verification workflow. Track performance remains unvalidated.
- Serial telemetry yields when its queue is busy; map export waits for room instead of dropping map records. Queue and real bridge UART behavior have dedicated host tests.

## Live-controls delivery — 2026-11-18

Uploaded `direct-live-v1` to the UNO R4 WiFi after USB reconnection. Verified `WIFI_READY http://192.168.4.1 build=direct-live-v1` and `MANUAL,STOPPED` with both outputs zero. Serial connection closed afterward; no motion was commanded.

The final UNO build uses 144180 bytes flash and 20016 bytes global RAM. Real Chrome checks pass for the gzip-served page, sliders, live trail, pause/resume, recording download, coordinate-frame changes, Stop, connection loss, and phone layout. Firmware tests cover live speed tuning, rejected stale owners, motor-tuning Stop, and complete binary HTTP transmission. The odometry reconfiguration epoch bug was reproduced with a failing test and corrected. The complete host sweep passed 49 C++ programs, 28 Python tests and USB browser tests; the same three prior course-model failures remain (`test_line_control_replay`, `test_line_eight_cell`, `test_line_eight_cell_course`). No physical driving or phone Wi-Fi trial was performed for this build.

## Restored controller deployment — 2026-11-18

Uploaded geometry-live-v2 over USB. Serial verified WIFI_READY at http://192.168.4.1 with that build identifier and MANUAL,STOPPED with both motor outputs zero. No movement commanded. Build: 148076 bytes flash, 20168 bytes RAM. Browser checks (three sliders, live path, labelled export, ownership and stop), 28 Python tests and USB browser-event checks pass. C++ sweep: 48 files pass, four course suites fail as listed in Line-Control-Tuning.md. Physical turning remains to be tested.
