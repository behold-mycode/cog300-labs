# Ultrasonic sensors — current behavior

The previous single-sensor obstacle-stop implementation was retired. **Use the [front/right wiring guide](front-right-wiring.md) and its complete diagrams**: front TRIG/ECHO A0/A1, right TRIG/ECHO A2/A3, green LED D11. The red LED is not used.

`Sensors.cpp` measures and validates readings. `Navigation.cpp` decides how autonomous modes respond. Manual keyboard driving is independent of sensor readings; missing/close readings do not block manual motion. Follow modes stop when their required sensor readings are invalid. Right-wall mode uses front clearance for bounded left turns.

See [autonomous modes](autonomous-modes.md), [logging](distance-logging.md), and [test checklist](test-plan.md). Upload the entire `Robot` folder; firmware 4 uses 115200 baud and the browser controller.
