# Wi-Fi and USB control

The UNO R4 WiFi hosts its own password-protected network. Join the SSID configured in `Robot/WirelessSecrets.h`, then open **http://192.168.4.1**. No router or internet is needed. Credentials remain in that ignored local file; do not commit it.

1. Power the Arduino through USB and the motor driver from its motor supply.
2. Disconnect the desktop controller's serial session if it is connected. The USB cable may stay plugged in for power.
3. Open the page, choose **Take Wi-Fi control**, place the centre sensor over the tape, then choose **Start line following**.
4. Keep the page visible. **STOP** works regardless of who owns control. After Stop, explicitly take control again before starting.
5. To use USB again, release Wi-Fi control (or Stop), then connect the existing desktop controller. USB uploads and serial diagnostics are unchanged.

The page shows normalized tape contacts, measured wheel speeds, applied PWM, navigation state and line-controller phase (0 stopped, 1 follow, 2 corner, 3 search). It exposes the three line-tuning command groups. Applying a group stops the car; restart explicitly. Tuning is RAM-only and resets on reboot. An optional PWM floor remains available, default zero.

Wi-Fi heartbeat expiry stops the motors within 1500 ms of the last accepted heartbeat. USB retains its 500 ms timeout. A hidden page sends Stop and ceases heartbeats. Expiry never falls back automatically to another connection or resumes motion. Delayed commands carrying an expired token are rejected. A Wi-Fi Stop also expires a USB session so buffered USB drive commands cannot restart the car; reconnect that desktop controller when ready.

The firmware boots stopped. Wi-Fi does not supply power. For untethered use the Arduino needs an appropriate USB power source in addition to the motor supply. OTA firmware uploading is not included.

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
