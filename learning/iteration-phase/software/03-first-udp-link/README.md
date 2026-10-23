# ArUco Marker → Raspberry Pi Camera → Python → UDP → MATLAB/Simulink

1) pi_gimbal_udp_stream.py (detects the marker and sends X/Y)

This file runs on the Raspberry Pi.

what it does?

1. Opens the Raspberry Pi camera.
2. Detects the ArUco marker in each camera frame.
3. Uses `solvePnP` to calculate the marker's position relative to the camera.
4. Extracts the X and Y coordinates.
5. Sends `[X, Y]` to the computer running Simulink using UDP.

2) GimbalUDPReceiver.m (receives X/Y from the Raspberry Pi)

This file runs on the computer running MATLAB/Simulink.

It acts as the UDP receiver for the data sent by the Raspberry Pi.

what it does?

1. Listens for incoming UDP packets.
2. Reads the X and Y values sent by the Raspberry Pi.
3. Converts the received data into a signal that can be used inside Simulink.
4. Holds the last received X and Y values if no new packet has arrived.

3) build_gimbal_model.m (automatically creates a Simulink model to view and test the received X/Y data)

This is a MATLAB script that automatically creates the Simulink model used for testing the UDP connection.

Instead of manually creating and connecting the blocks, this script

1. Creates the GimbalUDPReceiver block.
2. Receives the X and Y signals.
3. Separates X and Y into individual signals.
4. Adds scopes so the received values can be monitored.

Running this script generates gimbal_model.slx


---

## Where this stands now

This folder is the first working Raspberry Pi → Simulink link, written before
the gimbal runtime existed. It was superseded by `final-product/raspberry-pi/servo.py`
and `final-product/simulink/GimbalUDPReceiver.m`.

Things to know if you study it:

- Its packet is `[x, y, z, pitch, yaw, distance]`, where pitch and yaw are the
  **marker's own orientation** from `cv2.RQDecomp3x3`. The final sender puts
  different values in those slots (see `final-product/known-issues.md`).
- With several markers in view it sends the **last** one detected in the frame.
- `MARKER_SIZE` is hard-coded to 0.05 m and the host IP is hard-coded in the script.
- `GimbalUDPReceiver.m` here is an older version of the receiver. Do not put this
  folder and `final-product/simulink` on the MATLAB path at the same time, or
  MATLAB may pick up the wrong class.
