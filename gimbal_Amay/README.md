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

`servo.py` is the normal runtime entry point. It first sweeps the gimbal until
marker ID `0` is centered, latches that position as a permanent origin, and
then performs detection, servo control, and UDP transmission in one process.

## Quick Start Checklist

Complete these steps in this order:

1. Set the physical target-marker edge and `MARKER_SIZE` in `gimbal_config.py`.
2. Run camera calibration and confirm that `camera_calibration.npz` is saved in this folder.
3. Find the MATLAB computer's IP address and set `SIMULINK_HOST` in `gimbal_config.py`.
4. Put `GimbalUDPReceiver.m` on the MATLAB path.
5. Test the UDP link with `build_udp_monitor_model.m`.
6. Add the receiver and Demux blocks to `anjaneya_V7.slx`.
7. Verify signal units and coordinate signs with Scopes.
8. Start the full guidance model only after the measurement signals look correct.

Do not start with the servos attached to the full mechanical assembly. First test camera detection and UDP reception, then test servo movement with safe mechanical limits.

## Files

- `servo.py`: Main runtime program. Reads camera frames, calls the ArUco detector, calculates servo angles, drives the servos, and sends UDP packets.
- `aruco_gimbal_picamera2.py`: Reusable camera and ArUco detection functions. The important public functions are `create_aruco_detector()` and `detect_marker_pose()`.
- `gimbal_config.py`: Shared camera calibration, marker size, optional distance diagnostics, servo settings, and Simulink network settings.
- `calibrate_camera_picamera2.py`: Captures ChArUco calibration images and creates `camera_calibration.npz`.
- `camera_calibration.npz`: Camera intrinsic calibration file used by the detector.
- `GimbalUDPReceiver.m`: MATLAB System object that receives the UDP pose packets.
- `build_udp_monitor_model.m`: Creates a small Simulink test model for checking the UDP connection.
- `README.md`: This guide.

## What Happens During One Frame

When `servo.py` is running, the following sequence repeats for every camera frame:

1. `Picamera2` captures a `640 x 480` image.
2. The image is converted to grayscale for ArUco detection.
3. `aruco_gimbal_picamera2.py` searches for markers from the `DICT_6X6_250` dictionary.
4. Before origin lock, only marker ID `0` is used. Its image center is aligned without `solvePnP()`, so its physical size is irrelevant.
5. Once ID `0` is centered, the current pan/tilt angles are latched permanently as the origin. ID `0` is excluded from subsequent target tracking.
6. Target markers are passed to OpenCV `solvePnP()` together with `MARKER_SIZE` and camera calibration.
7. `solvePnP()` returns the target translation `(x, y, z)` in meters. The target is converted to coordinates relative to the latched origin before UDP transmission.
6. The six values are stored in this order:

   ```text
   [x, y, z, pitch, yaw, distance]
   ```

7. The same pose is sent to MATLAB as a UDP datagram.
8. The `x` and `y` values are passed to `GimbalController`.
9. The controller calculates the desired pan and tilt angles, clamps them to the configured limits, converts them to pulse widths, and commands the servos through `pigpio`.
10. The camera frame is displayed. Pressing `q` exits the loop and releases the hardware and network resources.

If no marker is detected, no UDP pose packet is sent and the servo controller is not updated for that frame. The MATLAB receiver keeps its previous pose and reports `isNew = false`.

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

### Hardware checklist

- Power the servos from a suitable external supply. Do not rely on the Raspberry Pi 5 V rail for high-current servo loads.
- Connect the Raspberry Pi ground and the servo power-supply ground together.
- Confirm that the GPIO numbers in `gimbal_config.py` use BCM numbering.
- Mount the camera rigidly to the gimbal so that camera movement corresponds to servo movement.
- Place the marker far enough away that it remains inside the camera field of view during the motion.
- Keep an emergency way to disconnect servo power during initial testing.

### Software checklist

On the Raspberry Pi, verify:

```bash
python3 --version
python3 -c "import cv2, numpy, picamera2"
python3 -c "import cv2; print(hasattr(cv2, 'aruco'))"
systemctl status pigpiod
```

The last Python command must print `True`. If `picamera2` or `pigpio` cannot be imported, install the packages on the Raspberry Pi or select the correct Python interpreter.

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
MARKER_SIZE = 0.100
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
CONTROL_GAIN = 0.12
INTEGRAL_GAIN = 0.0
DERIVATIVE_GAIN = 0.0
DERIVATIVE_LIMIT_DEG_S = 45.0
INTEGRAL_LIMIT_DEG_S = 2.0
ERROR_DEADBAND_DEG = 1.0
MIN_COMMAND_DEG = 0.08
MAX_STEP_DEG = 0.5
CONTROL_UPDATE_PERIOD_S = 0.08
ANGLE_FILTER_ALPHA = 0.10
# No software angle limits; MAX_STEP_DEG still limits movement per frame.
YAW_OFFSET_DEG = 0.0
PITCH_OFFSET_DEG = 0.0
```

For detection speed, the shared configuration also provides:

```python
DETECTION_SCALE = 0.60
CORNER_REFINE_ITERATIONS = 10
CORNER_REFINE_EPSILON = 0.01
```

Lower `DETECTION_SCALE` can reduce response time, but a 100 mm marker may then
become too small to detect reliably. The runtime starts at `0.60`; test `0.50`
only if the marker remains clearly visible at its farthest operating distance.

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

## Z Distance

The servo controller uses the per-frame ArUco translation vector. It calculates
pan from `x/z` and tilt from `y/sqrt(x^2 + z^2)`, so no fixed screen-distance
calibration is required when the marker range changes.

`gimbal_config.py --calibrate` remains available as an optional diagnostic for
measuring a representative distance, but its cached value is not used by
`servo.py`.

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
x=+0.020m y=-0.010m | delta yaw=+2.86 deg, delta pitch=-1.43 deg | yaw=+2.86 deg, pitch=-1.43 deg | marker distance=0.400m
```

Press `q` in the camera window to stop. The program releases the servo outputs, closes the UDP socket, stops the camera, and closes the display window.

## Servo Angle Calculation

The detector returns marker coordinates in the camera coordinate system:

- `x`: horizontal offset in meters; positive is camera-right
- `y`: vertical offset in meters; positive is camera-down
- `z`: forward distance in meters

`servo.py` uses the full marker vector. Pan is calculated in the horizontal
plane, while tilt uses the horizontal range `sqrt(x^2 + z^2)`:

```python
yaw_error = -degrees(atan2(x, z)) + YAW_OFFSET_DEG
horizontal_range = sqrt(x**2 + z**2)
pitch_error = degrees(atan2(y, horizontal_range)) + PITCH_OFFSET_DEG
```

The controller filters these errors, applies a proportional correction, uses
a small deadband to prevent servo chatter, limits the maximum step per frame,
then clamps the result to the configured mechanical limits and converts the
angle to a PWM pulse width. This prevents noisy pose estimates from making the
servos continually overshoot the target.

Because the correction is recalculated from the current camera-frame vector on
every frame, it remains valid as the gimbal moves and the marker's `z` changes.
The feedback tuning values above use a filtered PID controller to reduce
vibration caused by pose noise:
`ANGLE_FILTER_ALPHA` smooths measurements, `CONTROL_GAIN` reduces the response,
`INTEGRAL_GAIN` corrects small persistent error, `DERIVATIVE_GAIN` damps rapid
changes, `INTEGRAL_LIMIT_DEG_S` prevents windup, `ERROR_DEADBAND_DEG` suppresses
tiny corrections, and `MAX_STEP_DEG` limits instantaneous movement.

### Meaning of the controller outputs

`GimbalController.update(x, y, z)` returns four values:

```text
delta_yaw, delta_pitch, current_yaw, current_pitch
```

- `delta_yaw`: change requested for the pan axis in degrees
- `delta_pitch`: change requested for the tilt axis in degrees
- `current_yaw`: current clamped pan angle estimate
- `current_pitch`: current clamped tilt angle estimate

The pulse-width conversion is:

```python
pan_pulse_us = PAN_CENTER_US + current_yaw * US_PER_DEGREE
tilt_pulse_us = TILT_CENTER_US + current_pitch * US_PER_DEGREE
```

If the gimbal moves in the wrong direction, correct the sign or the relevant `YAW_OFFSET_DEG`/`PITCH_OFFSET_DEG` after testing at low limits. Do not solve a direction problem by increasing the servo limits.

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

The resulting basic Simulink layout should look like this:

```text
[MATLAB System: GimbalUDPReceiver]
                     |
                     v
            [Demux, 6 outputs]
               |  |  |  |  |  |
               X  Y  Z  P  Y  D
```

For the first test, connect each Demux output to a Scope. This confirms the receiver and packet ordering before any signal is connected to the missile guidance loop.

#### MATLAB System block parameters

The block uses these values:

| Parameter | Value | Meaning |
|---|---:|---|
| System object | `GimbalUDPReceiver` | MATLAB class in this folder |
| Port | `25000` | UDP port shared with Python |
| LocalIP | `0.0.0.0` | Listen on all local network interfaces |

Only one process should bind to UDP port `25000` on the MATLAB computer. Close the monitor model before starting another model that uses the same receiver port.

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

### Simulink first-run procedure

Use this sequence for the first complete test:

1. Open MATLAB on the computer that will receive the packets.
2. Add the `gimbal_Amay` folder to the MATLAB path.
3. Run `which GimbalUDPReceiver` and confirm the correct file is found.
4. Open the monitor model or `anjaneya_V7.slx` with the receiver block configured.
5. Start the Simulink model with the servo/guidance actuator path disabled.
6. On the Raspberry Pi, start `python3 servo.py`.
7. Hold one marker in front of the camera.
8. Confirm that `X`, `Y`, `Z`, and `Distance` are nonzero and physically reasonable.
9. Move the marker horizontally and vertically and confirm that only the expected coordinates change significantly.
10. Cover the marker and confirm that `isNew` changes to false while the last pose is held.
11. Reconnect the pose values to the intended guidance inputs.
12. Enable the guidance and actuator path with conservative limits.

### Expected network behavior

The Python terminal prints a pose only when the marker is detected. MATLAB should show the sequence number increasing by one for every received pose packet. The sequence number may skip values if Simulink runs slower than the camera; this is expected because the receiver drains old datagrams and keeps the newest one.

The UDP sender does not wait for an acknowledgement. Therefore, a Python message such as `sendto()` succeeding only means that the operating system accepted the datagram locally. The monitor scopes are the proof that MATLAB received and decoded it.

### Connecting to existing model signals

Do not assume that `x`, `y`, `pitch`, and `yaw` can be connected directly to every input in `anjaneya_V7`. Check each destination input for:

- expected physical quantity
- units: meters, degrees, or radians
- coordinate frame: camera, world, missile, or screen frame
- sign convention: positive right/up or positive right/down
- required sample time and data type

Use Signal Conversion, Gain, Unit Delay, or Rate Transition blocks where the existing model requires them. Keep the raw receiver signals on Scopes so a conversion error can be identified quickly.

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
