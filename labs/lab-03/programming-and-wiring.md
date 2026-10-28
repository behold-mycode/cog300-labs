# Programming and power reference

The working sketch is [MyRobot-V2](src/MyRobot-V2/), protocol 4. The signal pin mapping is documented in the [current wiring guide](front-right-wiring.md), with motors on D5–D10 and sensors on A0–A3. No signal pins change in the controller cleanup.

## Power: course USB bench-test arrangement

For the common L298N module with a removable regulator jumper, and only after confirming that the module matches:

```text
Laptop USB ──> Arduino USB socket
Arduino 5V ──┬──> L298N motor supply terminal (often labeled +12V / VS)
             └──> L298N logic supply terminal (5V)
Arduino GND ─────> L298N GND

5V-EN regulator jumper: removed for externally supplied 5V logic
ENA and ENB jumpers: removed for Arduino PWM control
Battery: disconnected in this arrangement
```

This follows the lab's suggested 5V bench supply. The terminal label “12V” does not mean that 12V is being supplied here. The regulator jumper must be removed when the module's 5V terminal is used as a logic power input. Do not connect the module's regulated 5V output against the Arduino's 5V supply.

USB current limits and the L298 voltage drop can make two motors weak or cause resets, especially when starting or stalled. Start with one motor and wheels raised, and do not leave a stalled motor energized. For an external motor supply, replace the Arduino-to-VS connection with a correctly rated motor supply, retain common ground, and check the actual motor/module ratings and regulator configuration. Do not connect battery positive to Arduino 5V or select a battery solely from the driver's maximum voltage.

Sources: [course power instructions](https://cogs-300.github.io/labs/lab03#2-prototyping-the-control-circuit), [module regulator instructions](https://robu-prod-media.s3.ap-south-1.amazonaws.com/uploads/2017/12/L298N-Module-Pinouts-ROBU.IN_.pdf), [ST L298 datasheet](https://www.st.com/resource/en/datasheet/l298.pdf).

## Software calibration

The [browser controller](controller/README.md) exposes independent left/right inversion and maximum PWM. Default power is 255, left inversion off, right inversion on. Confirm the actual wheel direction using bounded individual-wheel tests; see the [hardware test checklist](test-plan.md). Use [protocol.md](protocol.md) for current commands. The old proposed pin map and starter-code review are available in Git history.
