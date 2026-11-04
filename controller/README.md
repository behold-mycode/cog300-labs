# Robot controller

A small local browser interface for keyboard driving and two autonomous modes. Python talks to USB serial; Safari/Chrome/Firefox display the interface. No Tk, browser extension, Web Serial support, global keyboard hook or Accessibility permission is required.

## Mac setup

1. Download and extract the repository into a writable folder on your Mac (Intel or Apple Silicon). Do not copy `.venv` or `.runtime` from another computer.
2. If needed, install Arduino IDE and **Arduino UNO R4 Boards** through Boards Manager, and upload the complete `../Robot` sketch to UNO R4 WiFi. This controller requires **firmware 4**.
3. Close Arduino Serial Monitor and Serial Plotter.
4. Double-click **Launch Robot Controller.command** in the project root. It downloads uv 0.13.6 into `.runtime`, provisions Python 3.12 if no working project environment exists, installs `pyserial==3.5` into `.venv`, starts a loopback server, and opens your browser. **Python, pip, Homebrew, Xcode, and Node need not be preinstalled.** First-time setup needs internet and a macOS version supported by the downloaded runtime. macOS 13+ is the recommended baseline; older releases are not validated.
5. Select the `/dev/cu.usbmodem…` port and click Connect. Wait about two seconds for reset/handshake, then drive.

If macOS prevents double-click launch, open Terminal, type `zsh `, drag the launcher into Terminal, and press Return. The launcher does not require administrator access or change shell profiles. Downloads use [Astral's official installer](https://docs.astral.sh/uv/reference/installer/) and [managed Python distributions](https://docs.astral.sh/uv/guides/install-python/). A working environment is reused without reinstalling dependencies. An unusable environment is preserved as `.venv.old.*` before rebuilding. Generated runtime folders are excluded from Git.

Keep Terminal open while using the controller; Ctrl+C shuts it down and stops the robot. If the browser does not open, use the full local URL printed in Terminal, including its `#token=…` fragment. Setup failures leave an error in Terminal; check internet access and that the extracted folder is writable, then rerun. To try the interface without a robot, run the launcher with `--demo` from Terminal.

Manual setup, also usable on Windows/Linux:

```sh
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements.txt
.venv/bin/python controller.py
```

On Windows use `.venv\Scripts\python` for the last two commands. Linux users need a Python installation with venv support and permission to open their serial device.

## Driving

Hold arrows or WASD. Left/right alone spins; combine with forward/reverse to curve. Release to stop. Opposing directions stop. Space/Esc cancels any mode. Keyboard input only drives while this page is focused, and never while editing a settings field. Switching windows/tabs stops; held-key repeat cannot restart motion after stopping.

The side buttons choose **Follow object** or **Follow right wall**. Any new driving key returns to Manual. [Algorithm behavior and tuning](../docs/autonomous-modes.md).

Settings are collapsed by default. Opening/editing them stops motion. Apply power, minimum follow PWM, independent wheel reversal and target distances there. The server retains applied settings during reconnects; closing it restores defaults on the next launch. A firmware reset also restores firmware defaults until the controller reconnects and applies its settings. Settings may be adjusted only when connected.

Diagnostics shows a bounded recent log (600 lines); Download log saves that recent history. Front/right telemetry is reported ten times per second (each sensor measures about seven times per second). A dash means invalid/unavailable, not zero distance.

## Connection behavior

One page owns each connection. Other pages cannot take over. Closing or freezing the page expires its 600 ms lease; Python stops and closes serial. A frozen Python process is covered by the firmware's independent 500 ms watchdog. Reconnect after a timeout or reset. A normal window change stops immediately; if the browser suspends background timers, reconnect may also be required.

Requests use increasing sequence numbers so delayed requests cannot overwrite a newer stop. The HTTP server listens only on `127.0.0.1`, requires the launcher token for APIs, and validates Host/Origin. No cloud service is involved.

## Troubleshooting

- **No ports:** check USB cable/board power and Refresh. Charge-only cables do not carry serial data.
- **Port busy:** close Serial Monitor/Plotter and other controller instances.
- **No firmware 4 response:** upload the current complete sketch. Old Tk/firmware 3 code is incompatible.
- **Browser stopped responding:** return to the page and reconnect. No automatic motion resume.
- **Weak/no movement:** verify individual wheel direction with the [hardware checklist](../docs/test-plan.md); software PWM cannot fix a loose wire or inadequate supply.

Demo without dependencies or hardware:

```sh
python3 controller.py --demo
```

Demo simulates protocol responses and telemetry only; it does not simulate robot physics or validate the autonomous algorithms.

Motor direction calibration: the former “Reverse left wheel” checkbox controlled
channel A, which drives this car’s right wheel; the former right checkbox controlled
B, which drives its left wheel. Settings now name both the channel and wheel.
Defaults preserve the observed working polarity: A reversed, B normal. Firmware
maps logical left/right drive requests to B/A, so spins, curves and autonomous
steering use the same physical directions. Restart the controller and upload the
updated firmware together after this calibration change.
