# Standalone operation

The current authoritative hardware/startup guide is [Robot-Wiring.md](Robot-Wiring.md), with [Robot-Wiring.svg](Robot-Wiring.svg). The old graphical diagrams describe a superseded bench setup.

The firmware now owns startup. With the physical dimensions entered once in `Robot/HardwareProfile.h`, it enables the planned sensors/encoders, loads calibration matching that profile, or runs a bounded automatic wheel/coast measurement after an explicit Start. It saves derived operating values and enters the maze solver. Boot itself remains stopped. The compiled dimensions are still unset because they were not supplied; the firmware does not invent them. Those missing facts and the exact first-start placement/clearance assumptions are listed in the wiring guide.

The two desktop modes are Maze solver and Manual driving. Selecting a mode stops; Start runs it; Space/Escape/Stop stop it. Keyboard manual control remains. The app neither configures the robot nor supplies navigation decisions. A USB-controlled run stops on lost heartbeat or window focus; a physical/onboard autonomous start does not require the desktop or USB lease.

The robot's access point is `COGS-Robot`, page `http://192.168.4.1`. The private build password lives in ignored `Robot/WirelessSecrets.h`; copy the example header and choose a private password for another checkout. Missing password disables the AP, not the physical button. The onboard page starts autonomous running while stopped. It is not a real-time wireless joystick/STOP interface: WiFiS3 calls are suspended during movement, and the physical button stops the run. This communication limitation remains explicit rather than promising remote stop that is not implemented.

The normally-open D1-to-GND button uses a deliberate hold (at least 0.8s) and release to start; a press during motion stops. A stop press cannot immediately restart. On the first uncalibrated start, that command authorizes the bounded forward commissioning movement before maze navigation; use the clear-lane arrangement in the wiring guide.

Maze solver follows usable tape, switches to walls after sustained tape loss and a stop, and reacquires stable tape in corridors without erasing the map. A measured junction manoeuvre finishes before switching guidance. The graph records traversed connections and supports dead-end return. Ambiguous localization or missing clearance can still stop the robot; there is no guarantee for arbitrary maze geometry. No goal detector is required: stop and pick up the robot at the goal.

For single-pack standalone power, a suitable regulated 5V converter is still needed; the user rejected a separate power bank. The software changes have not been uploaded or physically validated. Passing synthetic commissioning/course tests is not a physical performance claim.

Legacy serial calibration commands remain engineering/debug interfaces; they are not an app workflow or the normal startup path. Autonomous save/load/start requires a valid compiled physical profile, and saved calibration must match its dimensions and polarity.
