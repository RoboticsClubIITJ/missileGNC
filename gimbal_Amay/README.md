# Gimbal Amay: ArUco, Servo, and Simulink Integration

This folder contains the complete Raspberry Pi gimbal pipeline. It detects an ArUco marker with the Pi camera, calculates the marker pose, points the pan/tilt gimbal toward it, and sends the pose to MATLAB/Simulink over UDP.

## System Flow

```text
Pi Camera
   |
   v
aruco_gimbal_picamera2.py
   |
   | marker pose: [x, y, z, pitch, yaw, distance]
   +----------------------+
   |                      |
   v                      v
servo.py              UDP packet
   |                      |
   v                      v
Pan/tilt servos       GimbalUDPReceiver.m
                              |
                              v
                       anjaneya_V7.slx
```

`servo.py` is the normal runtime entry point. It performs detection, servo control, and UDP transmission in one process.

## Files

- `servo.py`: Main runtime program. Reads camera frames, calls the ArUco detector, calculates servo angles, drives the servos, and sends UDP packets.
- `aruco_gimbal_picamera2.py`: Reusable camera and ArUco detection functions. The important public functions are `create_aruco_detector()` and `detect_marker_pose()`.
- `gimbal_config.py`: Shared camera calibration, marker size, screen distance, servo settings, and Simulink network settings.
- `calibrate_camera_picamera2.py`: Captures ChArUco calibration images and creates `camera_calibration.npz`.
- `camera_calibration.npz`: Camera intrinsic calibration file used by the detector.
- `GimbalUDPReceiver.m`: MATLAB System object that receives the UDP pose packets.
- `build_udp_monitor_model.m`: Creates a small Simulink test model for checking the UDP connection.
- `README.md`: This guide.

## Hardware and Software

The runtime is intended for a Raspberry Pi with:

- Pi Camera supported by Picamera2/libcamera
- Two servos connected to the configured GPIO pins
- `pigpio` and a running `pigpiod` service
- OpenCV with ArUco support (`opencv-contrib`)
- NumPy
- Picamera2

MATLAB requires:

- Simulink
- Instrument Control Toolbox for `udpport`
- The folder containing `GimbalUDPReceiver.m` on the MATLAB path

## Python Setup

Run these commands on the Raspberry Pi. Package names can differ depending on the Raspberry Pi OS image, so use the distribution packages where available.

```bash
sudo apt update
sudo apt install python3-opencv python3-numpy python3-picamera2 pigpio
sudo systemctl enable --now pigpiod
```

Check that the camera and Python dependencies work before connecting servos.

## Configuration

Edit `gimbal_config.py` before running the system.

### Simulink network settings

```python
SIMULINK_HOST = "192.168.1.50"
SIMULINK_PORT = 25000
```

Set `SIMULINK_HOST` to the IP address of the computer running MATLAB. The MATLAB receiver listens on port `25000` by default. Both computers must be on the same network, and the MATLAB computer firewall must allow incoming UDP traffic on that port.

### Camera and marker settings

`camera_calibration.npz` must contain:

- `camera_matrix` or `mtx`
- `dist_coeffs` or `dist`

The marker printed size is configured here:

```python
MARKER_SIZE = 0.05
```

This value is in meters and must match the actual printed marker edge length. An incorrect value produces an incorrect estimated distance and position.

### Servo settings

The following values must match the physical wiring and servo calibration:

```python
PAN_GPIO = 18
TILT_GPIO = 19
PAN_CENTER_US = 1500
TILT_CENTER_US = 1500
US_PER_DEGREE = 10.0
PAN_LIMIT_DEG = (-90.0, 90.0)
TILT_LIMIT_DEG = (-45.0, 45.0)
YAW_OFFSET_DEG = 0.0
PITCH_OFFSET_DEG = 0.0
```

Check the wiring and limits before attaching the mechanical load. A wrong GPIO or pulse range can move a servo unexpectedly.

## Camera Calibration

Run this when the camera is first installed or its position/lens changes:

```bash
python3 calibrate_camera_picamera2.py
```

Move the ChArUco board through different positions, distances, and angles. Capture at least 15 varied views if possible. The script saves:

```text
camera_calibration.npz
```

The detector loads this file automatically through `gimbal_config.py`.

## Screen Distance Calibration

The servo angle calculation uses a fixed camera-to-screen distance instead of the per-frame ArUco `z` value.

Place the marker at the center of the screen or board and run:

```bash
python3 gimbal_config.py --calibrate
```

This saves the measured value in `z_distance.json`. If the file does not exist, the code uses the placeholder value `0.40` meters. For accurate pointing, perform this calibration first.

## Running the Pi Program

1. Start MATLAB and configure the Simulink receiver as described below.
2. Confirm `SIMULINK_HOST` in `gimbal_config.py`.
3. Start `pigpiod` on the Raspberry Pi.
4. From this folder, run:

```bash
cd /path/to/gimbal_Amay
python3 servo.py
```

The program displays the camera image and prints values similar to:

```text
x=+0.020m y=-0.010m | delta yaw=+2.86 deg, delta pitch=-1.43 deg | yaw=+2.86 deg, pitch=-1.43 deg | screen distance=0.400m
```

Press `q` in the camera window to stop. The program releases the servo outputs, closes the UDP socket, stops the camera, and closes the display window.

## Servo Angle Calculation

The detector returns marker coordinates in the camera coordinate system:

- `x`: horizontal offset in meters; positive is camera-right
- `y`: vertical offset in meters; positive is camera-down
- `z`: forward distance in meters

`servo.py` uses the configured fixed screen distance:

```python
yaw = degrees(atan2(x, SCREEN_DISTANCE_M)) + YAW_OFFSET_DEG
pitch = degrees(atan2(y, SCREEN_DISTANCE_M)) + PITCH_OFFSET_DEG
```

The controller keeps the current servo angle, calculates the required change, clamps the result to the configured mechanical limits, and converts the angle to a PWM pulse width.

The current controller accumulates the detected `x` and `y` offsets over frames. This assumes each new detection is a camera-relative correction produced after the gimbal moves. If the target is stationary and the camera is not moving with the gimbal, this accumulation behavior should be changed before use.

## UDP Packet Format

For every valid marker detection, `servo.py` sends one UDP datagram to `SIMULINK_HOST:SIMULINK_PORT`.

The payload is:

```text
[x, y, z, pitch, yaw, distance, sequence_number]
```

Binary format:

```python
struct.pack("<6dI", *marker_array, sequence_number)
```

This is:

- six little-endian 64-bit floating-point values: 48 bytes
- one little-endian 32-bit unsigned sequence number: 4 bytes
- total packet size: 52 bytes

The sequence number increments for each packet and wraps after `0xFFFFFFFF`.

## MATLAB and Simulink Setup

The Simulink computer is the UDP receiver. The Raspberry Pi running `servo.py` is the UDP sender. Start the receiver model before moving the marker or enabling the servos.

### 1. Check MATLAB requirements

Install or enable:

- MATLAB
- Simulink
- Instrument Control Toolbox, which provides `udpport`

The MATLAB release must support `udpport` and MATLAB System blocks. `GimbalUDPReceiver.m` is a `matlab.System` object and cannot be used as a normal MATLAB Function block.

### 2. Add the receiver folder to the MATLAB path

Run this in MATLAB, replacing the path with the actual location on the Simulink computer:

```matlab
gimbalFolder = "path/to/missileGNC/gimbal_Amay";
addpath(gimbalFolder);
savepath; % Optional: make the path persistent
which GimbalUDPReceiver
```

The final command must print the path to `GimbalUDPReceiver.m`. If it does not, the MATLAB path is wrong.

### 3. Configure the network

Find the MATLAB computer's local IP address and put it in `gimbal_config.py` on the Raspberry Pi:

```python
SIMULINK_HOST = "192.168.1.50"
SIMULINK_PORT = 25000
```

The MATLAB receiver listens on:

```text
LocalIP: 0.0.0.0
Port: 25000
```

Allow incoming UDP traffic on port `25000` in the MATLAB computer's firewall. Confirm that the Raspberry Pi and MATLAB computer are on the same network. UDP does not establish a connection, so a successful Python run alone does not prove that MATLAB is receiving packets.

### 4. Test the UDP link first

From MATLAB, run:

```matlab
cd(gimbalFolder)
build_udp_monitor_model
```

This creates and opens `gimbal_Amay_udp_monitor.slx`. Start that model, then run `servo.py` on the Raspberry Pi:

```bash
cd /path/to/gimbal_Amay
python3 servo.py
```

The monitor scopes should receive these six values:

1. `X`: marker horizontal position, meters
2. `Y`: marker vertical position, meters
3. `Z`: marker forward position, meters
4. `Pitch`: marker rotation, degrees
5. `Yaw`: marker rotation, degrees
6. `Distance`: straight-line camera-to-marker distance, meters

The receiver also outputs `seq`, the packet sequence number, and `isNew`, which is true only when a new packet arrived. If the marker disappears, the receiver holds the last pose and sets `isNew` to false.

### 5. Add the receiver to `anjaneya_V7.slx`

Open the final model and perform these steps:

1. Add a **MATLAB System** block from the Simulink Library Browser. The block is usually under **User-Defined Functions > MATLAB System**.
2. Open the block parameters.
3. Set **System object name** to:

   ```text
   GimbalUDPReceiver
   ```

4. Set `Port` to `25000`.
5. Set `LocalIP` to `0.0.0.0`.
6. Add a **Demux** block from **Signal Routing**.
7. Set the Demux number of outputs to `6`.
8. Connect output 1 of the receiver to the Demux input.
9. Connect the six Demux outputs to signal labels, Scopes, or the appropriate guidance inputs.

The receiver has three outputs:

```text
Output 1: pose = [x y z pitch yaw distance]
Output 2: seq  = uint32 packet counter
Output 3: isNew = boolean fresh-packet flag
```

Do not connect the six-element pose vector directly to a scalar model input. Demux it first, or use a Selector block to select the required elements.

### 6. Map the pose to the missile model

The UDP receiver only transports measurements. It does not automatically know which `anjaneya_V7` input represents a target position or line-of-sight measurement.

Before connecting signals, check the existing model input labels and units:

- Use `x` and `y` if the model expects marker position in meters.
- Use `pitch` and `yaw` if the model expects measured angles in degrees, converting to radians if required.
- Use `distance` if the model needs range.
- Use `z` only if the model needs the marker's forward camera coordinate.

If the model expects radians, insert a **Gain** block with gain `pi/180` on the degree-valued pitch or yaw signals. If it expects a different coordinate direction, insert a **Gain** block with `-1` where necessary and document that convention in the model.

Use the `isNew` output to prevent stale measurements from updating the guidance logic. A common arrangement is:

```text
GimbalUDPReceiver -> Demux -> pose conversion -> guidance input
                |
                +-> isNew -> Enabled Subsystem or validity logic
```

### 7. Configure model timing

Set a discrete or fixed-step solver for the real-time receiver model. The UDP receiver is polled once per Simulink sample hit. Choose a sample time appropriate for the camera update rate, for example `0.02` seconds for approximately 50 Hz, and ensure the model can execute faster than that.

Avoid using a very large step size: the receiver intentionally drains queued datagrams and keeps only the newest packet, so a slow model will skip intermediate camera frames.

### 8. Validate inside `anjaneya_V7`

Run the model with the receiver connected but with the guidance or actuator subsystem disabled first. Add Scopes or `To Workspace` blocks to verify:

- X and Y change when the marker moves horizontally and vertically.
- Distance remains physically reasonable.
- `seq` increases when packets arrive.
- `isNew` becomes false when the marker is hidden.
- No degree/radian conversion or sign inversion is missing.

Only after these checks should the pose signals be connected to the active guidance and control path.

## Connection Troubleshooting

### No marker detected

- Confirm the marker uses the `DICT_6X6_250` dictionary.
- Confirm `MARKER_SIZE` matches the printed marker.
- Improve lighting and keep the complete marker visible.
- Check that OpenCV includes the `aruco` module.

### Camera calibration error

- Confirm `camera_calibration.npz` is in this folder.
- Confirm it contains the expected matrix and distortion keys.
- Re-run `calibrate_camera_picamera2.py` if the camera or resolution changed.

### Servo does not move

- Confirm `pigpiod` is running: `systemctl status pigpiod`.
- Check GPIO wiring and power supply.
- Check `PAN_GPIO` and `TILT_GPIO`.
- Test with the servos disconnected from the mechanical assembly first.

### MATLAB receives nothing

- Confirm `SIMULINK_HOST` is the MATLAB computer's IP address, not the Raspberry Pi's IP address.
- Confirm both sides use UDP port `25000`.
- Add `gimbal_Amay` to the MATLAB path.
- Start the Simulink model before or while running `servo.py`.
- Check the MATLAB computer firewall.
- Run `build_udp_monitor_model` to isolate the network link from the missile model.
- Do not run multiple Python UDP senders at the same time unless they intentionally share the same receiver.

### MATLAB reports an invalid packet

The receiver expects exactly 52 bytes. Make sure the sender uses:

```python
struct.pack("<6dI", *marker_array, sequence_number)
```

Do not change the number, order, or types of fields without updating `GimbalUDPReceiver.m` as well.

## Recommended Startup Order

1. Connect the Raspberry Pi and MATLAB computer to the same network.
2. Start `pigpiod` on the Raspberry Pi.
3. Start MATLAB and open `anjaneya_V7.slx` or the UDP monitor model.
4. Start the Simulink model.
5. Start `python3 servo.py` on the Raspberry Pi.
6. Confirm the X/Y values and sequence number change in Simulink.
7. Only then attach or enable the full mechanical gimbal load.

## Important Limitation

The UDP receiver and sender are implemented and packet-compatible, but the binary `anjaneya_V7.slx` file cannot be safely rewired from this Python workspace. The receiver block must be added and connected inside Simulink, and those connections must match the model's expected signal units and coordinate conventions.
