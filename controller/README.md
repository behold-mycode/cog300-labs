# Robot controller

A small local browser interface for keyboard driving and two autonomous modes. Python talks to USB serial; Safari/Chrome/Firefox display the interface. No Tk, browser extension, Web Serial support, global keyboard hook or Accessibility permission is required.

## Mac setup

1. Install **Python 3.9+** from [python.org](https://www.python.org/downloads/macos/) if `python3 --version` is missing or older.
2. Upload the complete `../Robot` sketch to UNO R4 WiFi. This controller requires **firmware 4**.
3. Close Arduino Serial Monitor and Serial Plotter.
4. Double-click **Launch Robot Controller.command** in the project root. It creates `.venv`, installs `pyserial==3.5` on first use, starts a loopback server, and opens your browser. First-time dependency installation needs internet.
5. Select the `/dev/cu.usbmodem…` port and click Connect. Wait about two seconds for reset/handshake, then drive.

If macOS prevents double-click launch, run `zsh '../Launch Robot Controller.command'` from this folder in Terminal. If a copied `.venv` fails, rename/remove that generated `.venv` folder and rerun the launcher; virtual environments should not be copied between computers. Keep Terminal open while using the controller; Ctrl+C shuts it down and stops the robot. If the browser does not open, use the full local URL printed in Terminal, including its `#token=…` fragment.

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

Diagnostics shows a bounded recent log (600 lines); Download log saves that recent history. Front/right distances update twice per second. A dash means invalid/unavailable, not zero distance.

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
