# Raspberry Pi runtime

Runs on the Raspberry Pi. Detects ArUco markers with the Pi Camera, keeps the
target centred with the pan/tilt gimbal, and streams the target position to
Simulink over UDP.

```text
Picamera2 (640x480) → ArUco detection → corner refinement → solvePnP
   → pose validation → transform to origin frame ─┬→ gimbal control → servos
                                                   └→ UDP sender → Simulink
```

## Files

| File | Role |
|---|---|
| `servo.py` | **Entry point.** Origin search, tracking loop, gimbal controller, UDP sender, CSV log. |
| `aruco_gimbal_picamera2.py` | Detection library: `create_aruco_detector()`, `detect_marker_center()`, `detect_marker_pose()`. |
| `gimbal_config.py` | All settings: network, marker size, servo pins and gains, search behaviour. Loads the calibration file. |
| `calibrate_camera_picamera2.py` | ChArUco calibration. Writes `camera_calibration.npz`. |
| `camera_calibration.npz` | Camera matrix and distortion coefficients for the current camera. |
| `.env.example` | Template for your local `.env` (Simulink computer IP and port). |
| `requirements.txt` | Python packages. |

## Setup

On the Pi:

```bash
sudo apt update
sudo apt install python3-picamera2 python3-pigpio pigpio
sudo systemctl enable --now pigpiod
pip install -r requirements.txt

cp .env.example .env
nano .env          # set SIMULINK_HOST to the MATLAB computer's IP
```

Check everything imports and OpenCV has the aruco module:

```bash
python3 -c "import cv2, numpy, picamera2, pigpio; print(hasattr(cv2, 'aruco'))"
```

It must print `True`.

If there is no `.env`, `gimbal_config.py` falls back to a hard-coded IP from
the last test session, which is almost certainly wrong for you. UDP gives no
error when the address is wrong, so set `.env` first.

## Before the first run

1. **Marker size.** Measure the printed target marker's black edge and set
   `MARKER_SIZE` in `gimbal_config.py` in metres. It is currently 0.0738.
   A wrong value scales every distance by the same error.
2. **Calibration.** Re-run `python3 calibrate_camera_picamera2.py` if the
   camera, lens or resolution has changed. Capture at least 15 views of the
   ChArUco board at varied angles and distances. The board dimensions at the
   top of that script must match the printed board.
3. **Servo direction.** Test with the servos free to move. If an axis tracks
   the wrong way, fix the sign, not the limits.

## Running

Start the Simulink model first (see `../simulink/README.md`), then:

```bash
python3 servo.py
```

What happens:

1. **Origin search.** The gimbal does a slow raster scan (1.5° pan steps,
   2° tilt steps at each edge) looking for marker **ID 0**.
2. **Origin lock.** When ID 0 is within 18 px of the image centre, the
   current pan/tilt angles and ID 0's pose are latched as the origin. ID 0 is
   never tracked or sent after this.
3. **Tracking.** Any other visible marker is the target. Its camera-frame
   position is rotated back into the origin orientation and the origin pose
   is subtracted. The servos follow it at the same time.
4. Press `q` in the camera window to stop. The servos, socket and camera
   are released cleanly.

## What gets sent

One 52-byte UDP packet per send: six little-endian doubles and a uint32
sequence number (`struct.pack("<6dI", ...)`).

| Slot | Current `servo.py` sends | Receiver labels it |
|---|---|---|
| 1 | x relative to origin (m) | x |
| 2 | y relative to origin (m) | y |
| 3 | z relative to origin (m) | z |
| 4 | **distance** camera to target (m) | **bearing_pitch** |
| 5 | bearing yaw from origin (deg) | yaw |
| 6 | distance camera to target (m) | distance |

Slot 4 does not match its label. This is listed in `../known-issues.md`.
Camera axes: x positive right, y positive **down**, z forward.

### Timing

`FixedRateSender` sends at **50 Hz** (`period_s=0.02`). It does not send the
newest pose. It plays poses back **5 seconds behind real time**, linearly
interpolated between detections and smoothed with a Gaussian of
σ = 0.3 s. Nothing is sent for the first 5 seconds after the first
detection, and Simulink always sees the target as it was 5 s ago.

This gives a smooth, evenly spaced stream, but it is not real-time. The
earlier 20 Hz queue sender is still in `servo.py`, commented out.

## Gimbal control

Per frame, from the camera-frame target `(x, y, z)`:

```python
yaw_error   = -degrees(atan2(x, z)) + YAW_OFFSET_DEG
pitch_error =  degrees(atan2(y, hypot(x, z))) + PITCH_OFFSET_DEG
```

The error is filtered (`ANGLE_FILTER_ALPHA = 0.35`), multiplied by
`CONTROL_GAIN = 0.45` (I and D gains are 0), limited to `MAX_STEP_DEG = 4°`
per update, clamped to pan ±60° and tilt ±45°, and converted to a pulse:
`1500 µs + angle × 8 µs/deg`.

## Log file

`coordinates.csv` is rewritten at every start with one row per accepted pose:
raw and origin-relative position, relative yaw and pitch, and the origin
pose. It is ignored by git. Copy it somewhere else if you want to keep a run.

## Troubleshooting

| Symptom | Check |
|---|---|
| `Camera calibration file not found` | `camera_calibration.npz` must be in this folder. |
| No marker detected | Dictionary is `DICT_6X6_250`. Whole marker visible, even lighting. |
| Stays on "SEARCHING FOR ORIGIN ID 0" | Marker ID 0 must be inside the ±60° / ±45° search area. |
| Servos do not move | `systemctl status pigpiod`. Common ground. BCM pin numbers. |
| MATLAB receives nothing | `SIMULINK_HOST` is the MATLAB computer's IP. Same network. Firewall allows UDP 25000 in. Wait 5 s after the first detection. Test with `../tools/udp_test_listener.py`. |
| Distances look scaled | `MARKER_SIZE` does not match the printed marker. |
