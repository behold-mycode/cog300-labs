# Wi-Fi control

The robot hosts its own Wi-Fi network. Join the network configured in the ignored local file `Robot/WirelessSecrets.h` and open **http://192.168.4.1**. Choose **Take Wi-Fi control** and **Start line following**. The default firmware boots stopped.

Line following runs on the Arduino. Closing the page, losing Wi-Fi, or choosing **Release control** leaves the run active. Five seconds without an accepted heartbeat expires the operator token. After reconnecting, choose **Take Wi-Fi control** to adjust the running car. **STOP** works from any connected page. When radio communication is unavailable the page cannot deliver a stop command.

The speed slider adjusts the current run. The live readings show the IR mask, encoder speeds, motor PWM and longest steering interval. The firmware retries transient modem transactions in the background. A board reset interrupts both driving and Wi-Fi.

## Live path and recording

The **Live path** panel draws the estimated position and heading as telemetry arrives; it does not wait for the run to end. Polls are scheduled 250 ms after each response (up to four updates/second; actual rate depends on Wi-Fi response time). The scale fits the current trace automatically; the grid label shows its spacing. Position uses encoder odometry, not a camera or measured tape map. Wheel slip, wheel dimensions and inferred encoder direction affect accuracy.

Recording starts when the page opens. **Pause trail** pauses storage while the robot marker keeps updating. **Clear trail** clears this page's samples and places its display origin at the current position; it does not reset robot navigation. **Download run** saves JSON containing robot/browser timestamps, estimated pose, IR readings, PWM, measured speeds and tuning values. Recording is held in browser memory (up to 20,000 samples); download before reloading or closing the page. The page reports when the recording limit is reached.

Connection gaps and recording pauses break the plotted line. Robot restarts and odometry reconfiguration start new coordinate frames; old frames remain in the download, but only the current frame is drawn. A stale connection greys the robot marker and shows a waiting/lost status. Keep the page visible for continuous recording.

## Building the page

Edit `Robot/WirelessPage.h` and run `python3 build_wifi_page.py`. The Arduino serves the generated gzip file. Install firmware updates over USB.
