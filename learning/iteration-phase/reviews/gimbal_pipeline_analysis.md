> **Note:** this is an AI-assisted review written in Sep 2026 against the
> V10-era code (`gimbal_Amay/` and `simulink/` on the backup branch). Folder
> names refer to that old layout. Findings were not re-checked against V12 or
> the Oct runtime; the ones confirmed so far are in `final-product/known-issues.md`.

# gimbal_Amay Pipeline Analysis

Deep-dive report on the `gimbal_Amay/` folder: the complete Raspberry Pi gimbal pipeline that detects ArUco markers with a Pi Camera, tracks them with a 2-servo pan/tilt gimbal, and streams line-of-sight (LOS) pose data to MATLAB/Simulink over UDP.

> Scope: `gimbal_Amay/` only. The `simulink/` folder is covered in a separate pass.

---

## 1. System Overview

This is the "Seeker-in-the-Loop" hardware front-end of the missile guidance simulator. A physical camera + gimbal simulates the missile's seeker: it keeps a target marker in view, measures the target's 3D offset relative to a latched origin, packages it as LOS information, and feeds it to the missile model in Simulink.

```
 Raspberry Pi (seeker)
 ┌────────────────────────────────────────┐
 │  Pi Camera (Picamera2 / libcamera)     │
 │      │                                 │
 │      ▼                                 │
 │  aruco_gimbal_picamera2.py             │
 │  (detection + solvePnP pose)           │
 │      │                 │               │
 │      ▼                 ▼               │
 │  servo.py           servo.py           │
 │  GimbalController   UDP pose queue     │
 │  (servos via pigpio)(20 Hz thread)     │
 │      │                 │               │
 │      ▼                 ▼               │
 │  pan/tilt servos   UDP socket          │
 └─────┬───────────────┬──────────────────┘
       │               │ port 25000 (SIMULINK_HOST)
       │               ▼
       │          MATLAB / Simulink
       │          GimbalUDPReceiver.m
       │          (MATLAB System block)
       │          → pose[6], seq, isNew
       │               │
       │               ▼
       │          anjaneya_V10.slx (missile model)
       │
       └── visual tracking feedback loop
```

### Hardware
- Raspberry Pi (Pi Cam v1.3 class sensor via Picamera2, 640x480 RGB888)
- Two hobby servos (pan GPIO 18, tilt GPIO 19, BCM) driven through `pigpio`
- External servo power supply; common ground

### Software (Python side)
- `picamera2`, `opencv-contrib` (ArUco), `numpy`, `pigpio`, `python-dotenv` (optional)

### MATLAB side
- `GimbalUDPReceiver.m` (a `matlab.System` object), consumed by `build_udp_monitor_model.m` or `anjaneya_V10.slx`
- Uses Java `DatagramSocket` directly → **no Instrument Control Toolbox required**

---

## 2. Files in `gimbal_Amay/` — Relevance Map

| File | Role | Category |
|---|---|---|
| `servo.py` | **Runtime entry point.** Main loop, origin-lock state machine, `GimbalController`, queue-based 20 Hz UDP sender, HUD, CSV logging | Core runtime |
| `aruco_gimbal_picamera2.py` | Detection library: `create_aruco_detector()`, `detect_marker_center()`, `detect_marker_pose()` | Core runtime (imported) |
| `gimbal_config.py` | All configuration + camera-calibration loader + optional `--calibrate` Z-distance diagnostic | Config / calibration |
| `camera_calibration.npz` | Camera intrinsics + distortion coefficients + RMS error | Calibration data |
| `GimbalUDPReceiver.m` | MATLAB System object that decodes the UDP pose packets | MATLAB receiver |
| `build_udp_monitor_model.m` | Auto-builds `gimbal_Amay_udp_monitor.slx` to verify the UDP link in isolation | MATLAB utility |
| `udp_test_listener.py` | Quick Python-side UDP listener for link tests | Utility (**has a bug — see §7**) |
| `calibrate_camera_picamera2.py` | ChArUco capture script that produces `camera_calibration.npz` | Offline calibration |
| `.env.example` | Template for `SIMULINK_HOST` / `SIMULINK_PORT` | Config template |
| `useful_readme.md` | Short excerpt of the main README (network + packet format) | Redundant doc |
| `README.md` | Full setup/run/troubleshooting guide | Doc |
| `aruco_marker_stage.html` | Standalone web page for positioning/arranging marker(s) on a printable stage (drag-and-drop placement). **Not used by the runtime.** | Utility |
| `coordinates.csv` | Runtime telemetry log written by `servo.py` (stale data committed to git) | Generated data |
| `z_distance.json` | Created only when `gimbal_config.py --calibrate` is run. Not present in repo; not used by `servo.py` | Generated (optional) |

### Effective input → output chain

```
[printed ArUco marker]  --camera-->  [photo]  --photo-->  [solvePnP pose]
      -> [4-value camera-frame pose]  -> [gimbal servos]  |  [6-value UDP packet]
                                                        -> [GimbalUDPReceiver.m] -> [Simulink]
```

---

## 3. Deep Dive — file by file

### 3.1 `gimbal_config.py` (config + calibration loader)

- Loads `.env` if `python-dotenv` present: `SIMULINK_HOST` (default `127.0.0.1`) and `SIMULINK_PORT` (default `25000`).
- Loads `camera_calibration.npz` (accepts `camera_matrix`/`mtx` and `dist_coeffs`/`dist` key pairs), validates shape/finiteness, and exposes `FX, FY, CX, CY` scalars.
- Defines `MARKER_SIZE = 0.0738` m — must match the printed marker edge, or distance/position estimates are wrong.
- Defines all servo / PID / search constants (`PAN_GPIO`, `TILT_GPIO`, centers in µs, gains, deadband, step limits, search raster parameters).
- Supplies `--calibrate` mode (`calibrate_z_distance()`) that averages `z` over ~30 frames and caches to `z_distance.json` — a **diagnostic-only** leftover; `servo.py` uses per-frame ArUco `z`, not this cache.
- Note: `MARKER_SIZE`, `CAMERA_MATRIX`, `DIST_COEFFS` are read at import time here; `aruco_gimbal_picamera2.py` imports them.

**Input:** env vars, `camera_calibration.npz`, optional `.env` / `z_distance.json`.
**Output:** module-level constants consumed everywhere else.

### 3.2 `aruco_gimbal_picamera2.py` (detection library)

Public API used by `servo.py`:

- `create_aruco_detector()` → returns `(detector, dictionary, parameters, legacy_api)`. Uses `DICT_6X6_250` and sets adaptive-threshold window parameters. Handles both new (`ArucoDetector`) and legacy (`detectMarkers`) OpenCV APIs. Forces `cv2.setNumThreads(4)`.
- `detect_marker_center(frame, ..., marker_id)` — full-resolution detection, `cornerSubPix` refinement (`CORNER_REFINE_ITERATIONS = 10`, `EPS = 0.01`), returns the pixel center of the first marker matching `marker_id`. Used only for the origin (marker ID 0) alignment; **no marker size / `solvePnP` involved** here.
- `detect_marker_pose(frame, ..., detection_scale=0.6, excluded_marker_id, required_marker_id, return_rotation)` —
  1. grayscale → resize to 60% (`INTER_AREA`) → detect markers;
  2. corners scaled back by `1/detection_scale` and refined with `cornerSubPix` at full resolution;
  3. `solvePnP(marker_object_points, corners, CAMERA_MATRIX, DIST_COEFFS, flags=SOLVEPNP_ITERATIVE)`;
  4. returns `[x, y, z, distance]` where `distance = sqrt(x² + y² + z²)`, plus optionally the **rotation matrix** of the marker (from `cv2.Rodrigues(rvec)`).
- `main()` — a standalone demo/legacy loop (not used by `servo.py`) that detects all markers per frame and prints coordinates. Contains its own placeholder intrinsics that get replaced by the calibration file if present. This is effectively debugging code.

`marker_object_points()` builds the 3D corner template (square, size `MARKER_SIZE`, centered at origin, z=0) used by `solvePnP`.

**Input:** BGR/RGB frame, detector config.
**Output:** pixel center, or `[x, y, z, distance]` in meters (camera frame: +x right, +y down, +z forward), optional rotation matrix.

### 3.3 `servo.py` (runtime — most important file)

**Startup:** opens `Picamera2`, creates detector, `GimbalController` (connects to `pigpiod`, immediately drives servos to center), `SimulinkPoseStreamer` + `FixedRateSender` thread (starts immediately), truncates and re-creates `coordinates.csv` with header.

**Main loop state machine (every frame):**

1. **Not origin-locked** → raster-scan for marker ID 0:
   - `detect_marker_center(..., ORIGIN_MARKER_ID=0)`.
   - Not found → `search_step()`: pan ±1.5°/step (0.10 s cadence), stepping tilt ±2° at the pan edges, clamped to ±60° pan / ±45° tilt.
   - Found but off-center (> `ORIGIN_CENTER_TOLERANCE_PX = 18` px) → image-space proportional steering via `update_direction()` (no `solvePnP`, so marker size doesn't matter yet).
   - Found and centered → `detect_marker_pose(required_marker_id=0)`; on success **latch the origin**: store `origin_pose` (camera-frame pose of marker 0) and `origin_angles` (current gimbal yaw/pitch), then `reset_control_state()`. ID 0 is excluded from all future tracking.
2. **Origin-locked** → track targets:
   - `detect_marker_pose(excluded_marker_id=0, return_rotation=True)` → any marker that is **not** ID 0.
   - No marker → clear `previous_valid_pose`, send nothing new (sender thread re-sends last pose), HUD shows `NO TARGET`.
   - Marker present but `pose_has_valid_step()` fails (3-D displacement > 0.15 m vs last accepted pose, or non-finite / z≤0) → rejected as outlier, not sent/used.
   - Accepted → update `previous_valid_pose`, log to CSV, compute `relative_pose = pose_relative_to_origin(...)` (see §4), push to sender queue, and steer via `process_marker()` → `gimbal.update(x, y, z)`.

**`GimbalController` (filtered P-controller with PID scaffolding):**

- Converts marker vector to angular errors every frame:
  - `yaw_error  = -degrees(atan2(x, z)) + YAW_OFFSET_DEG`
  - `pitch_error= +degrees(atan2(y, hypot(x, z))) + PITCH_OFFSET_DEG`
- Input error is low-pass filtered (`ANGLE_FILTER_ALPHA = 0.35`).
- Rate-limited `update_direction()`: only recomputes at `CONTROL_UPDATE_PERIOD_S = 0.02` s; `dt` clamped to [1/120, 0.2] s.
- `_pid_correction()`: deadband `ERROR_DEADBAND_DEG = 0.8°`, integral clamped to `INTEGRAL_LIMIT_DEG_S = 2.0`, derivative clamped to `DERIVATIVE_LIMIT_DEG_S = 45°/s`, final correction clamped to `MAX_STEP_DEG = 4.0°/frame`, sub-`MIN_COMMAND_DEG` corrections zeroed. (Gains: `CONTROL_GAIN=0.45`, `I=0`, `D=0` — so effectively proportional + filter + rate limiting.)
- New angles clamped to `PAN_LIMIT_DEG`/`TILT_LIMIT_DEG` (±60/±45), then `_drive()` converts to pulse width `center + angle * US_PER_DEGREE` (8.0 µs/deg) clamped to [1000, 2000] µs and commands `pigpio.set_servo_pulsewidth()`.
- `stop()` zeroes both signals and closes `pigpio`.

**Pose validation:**
- `pose_has_valid_depth()` (z finite, >0, step ≤ 0.15 m) — historical; only `pose_has_valid_step()` is actually used in the loop (its superset).

**HUD (`draw_target_hud`):** reticle, projected marker bracket, telemetry panel (status, response ms, FPS, range/pan/tilt), target-lock line. `q` exits.

**Outputs:** servo PWM (via pigpio), live HUD window, CSV rows, 52-byte UDP packets at 20 Hz.

### 3.4 `calibrate_camera_picamera2.py` (offline)

- Builds a ChArUco board (`SQUARES_X=7, SQUARES_Y=10`, square 0.0105 m, marker 0.007 m — **must match the printed board and the comment references a `generate_charuco.py` that is not in this repo**).
- Live-capture loop: press SPACE to grab a frame with ≥6 matched corners, `q` to finish; needs ≥4 captures (ideally 15–25) from varied poses.
- Uses `board.matchImagePoints()` + `cv2.calibrateCamera()` (version-proof path; `calibrateCameraCharuco` removed in OpenCV ≥ 4.7).
- Saves `camera_calibration.npz` with keys `camera_matrix`, `dist_coeffs`, `rms_error`. Prints RMS (aim < 1.0).

**Input:** live video, printed ChArUco board.
**Output:** `camera_calibration.npz`.

### 3.5 `GimbalUDPReceiver.m` (MATLAB receiver)

`matlab.System` class (`GimbalUDPReceiver`) with tunable `Port` (default 25000):

- `setupImpl`: `java.net.DatagramSocket(Port)`, `setSoTimeout(1)` ms (non-blocking poll).
- `stepImpl`: drains the socket in a `while` loop; for each packet, converts Java `byte[]` → MATLAB, and if exactly **52 bytes**, decodes `pose = <6 double LE>` and `seq = <uint32 LE>`. Holds last pose when no new packet (so `isNew=false`). Weirdly, `pose` default is `zeros(1,6)` even for 4-value history — handled by 6-value format only.
- Outputs:
  - `pose` (1×6 double): `[x y z bearing_pitch? bearing_yaw? distance?]` — see **§7.1** for the mismatch between the documented labeling and what is actually sent.
  - `seq` (1×1 uint32)
  - `isNew` (1×1 boolean)
- `releaseImpl` closes the socket; `resetImpl` clears state.

**Input:** UDP datagrams on port 25000.
**Output:** `pose[6]`, `seq`, `isNew` signal ports for Simulink.

### 3.6 `build_udp_monitor_model.m`

Creates `gimbal_Amay_udp_monitor.slx`: one MATLAB System block (`GimbalUDPReceiver`) → 6-way Demux → six Scopes named `X, Y, Z, Pitch, Yaw, Distance`. Run this to prove the network link before wiring into the missile model. Only one process may bind to port 25000 at a time (close the monitor before running the missile model).

### 3.7 `udp_test_listener.py`

Binds `0.0.0.0:25000` and prints decoded packets. **Broken — see §7.2.** Also, `.env.example` is malformed (uses `//` line-comment syntax in an ini-style file; see §7.6).

---

## 4. The Six-Value UDP Packet (as the code actually implements it)

Python side, `SimulinkPoseStreamer.send()`:
```python
struct.pack("<6dI", *marker_array, sequence)
```
- 6 little-endian doubles (48 B) + 1 uint32 sequence (4 B) = **52 bytes**.
- Sent at a fixed **20 Hz** by `FixedRateSender` (period 0.05 s) in a dedicated thread, independent of camera/detection timing. Bounded queue (len 200); if the queue is empty the **last pose is re-sent** so the receiver still receives packets; `seq` increments on **every transmitted packet**, including re-sends, and wraps at 0xFFFFFFFF.

The values pushed by `pose_relative_to_origin()` (`servo.py`) are:

```
index  value                              meaning
0      rotated_x  - origin_x              X offset in origin frame (m)
1      rotated_y  - origin_y              Y offset in origin frame (m)
2      rotated_z  - origin_z              Z offset in origin frame (m)
3      marker_array[3]                    straight-line distance to target (m)
4      degrees(atan2(-rotated_x, rotated_z))   bearing (yaw) angle, deg
5      marker_array[3]                    straight-line distance again (m)
```

So the **true payload is `[x, y, z, distance, bearing_yaw, distance]`.**

### 4.1 Transform details (`pose_relative_to_origin`)

`marker_array` is measured in the **current camera frame**. The code rotates it back into the **latched-origin camera frame** using only 2 DOF — the relative pan (`relative_yaw = current_yaw - origin_yaw`) and relative tilt between the latch moment and now:

```
cy=cos(rY); sy=sin(rY); cp=cos(rP); sp=sin(rP)
rotated_x = cy*tx - sy*tz
rotated_y = cp*ty - sp*(sy*tx + cy*tz)
rotated_z = sp*ty + cp*(sy*tx + cy*tz)
```

(no roll term — valid for a pan/tilt gimbal whose roll axis is fixed), then subtracts the latched `origin_pose` (`x,y,z`) so all outputs are relative to the ID-0 origin.

> ⚠️ The README and the monitor model name outputs `[x, y, z, bearing_pitch, bearing_yaw, distance]`. **`bearing_pitch` is never computed anywhere in the code.** The 4th slot is *distance*, and the distance slot is duplicated in position 6. See §7.1.

### 4.2 Coordinate conventions (important for the Simulink consumer)

| Quantity | Convention |
|---|---|
| Camera `x` | + right |
| Camera `y` | + down (image-y) |
| Camera `z` | + forward (out of lens) |
| Servo yaw (`offset_to_angles`) | `-atan2(x,z)` → positive yaw = target to the left |
| UDP bearing_yaw | `+atan2(-rotated_x, rotated_z)` → positive when target is to the left — **same sign convention** as servo yaw |
| Servo pitch | `+atan2(y, hypot(x,z))`, positive = target below (y-down) |

Both yaw expressions are mathematically equivalent (`-atan2(x,z) == atan2(-x,z)`), so the servo yaw and the transmitted bearing_yaw agree in sign. The pitch sent to Simulink, however, is **not present** in the packet at all (see §7.1) — if the missile model needs pitch, it must be derived from `(x, y, z)`.

---

## 5. CSV Log Format (`coordinates.csv`)

Rewritten at startup; one row per *accepted* camera pose:

```
timestamp, frame_gap_s, raw_x_m, raw_y_m, raw_z_m,
relative_x_m, relative_y_m, relative_z_m,
relative_yaw_deg, relative_pitch_deg,
origin_x_m, origin_y_m, origin_z_m, transform_source
```

- `frame_gap_s` measures **detection/logging** cadence, *not* the UDP interval (UDP is a separate 20 Hz thread).
- `relative_yaw_deg` / `relative_pitch_deg` are recomputed in the CSV writer from the relative pose and match the transmitted bearing_yaw/pitch conventions.
- `transform_source` is always `latched_origin`.
- Note: `raw_x/y/z` come straight from `solvePnP` (camera frame); `relative_*` are origin-frame values.

---

## 6. End-to-End Data Flow (one accepted tracking frame)

```
1. capture 640x480 RGB888 frame
2. convert to grayscale
3. detect markers at 0.6x scale, refine corners at full res
4. solvePnP -> [x,y,z] (m)   +  distance=||t||
5. pose_has_valid_step?  no -> drop (status "REJECTED POSE OUTLIER")
                   yes -> v
6. pose_relative_to_origin -> [x,y,z,distance,bearing_yaw,distance]
7. push 6-tuple to FixedRateSender queue   -> sent at 20 Hz as <6dI>
8. GimbalController.update(x,y,z)          -> new yaw/pitch
9. clamp -> 1000..2000 us PWM via pigpio   -> servos move
10. HUD draw + CSV row
```

---

## 7. Problems & Risks

### 7.1 (HIGH) Packet payload does not match the documented format

- **Code sends:** `[x, y, z, distance, bearing_yaw, distance]`
- **README + `build_udp_monitor_model.m` + `GimbalUDPReceiver.m` docs claim:** `[x, y, z, bearing_pitch, bearing_yaw, distance]`
- `bearing_pitch` is **never computed**. The 4th demux channel ("Pitch" scope) actually shows *distance*; the 6th ("Distance") is a duplicate of it.
- **Impact:** any Simulink wiring that maps channel 4 to pitch is fed range data. If the missile model uses `bearing_pitch` for guidance, the behavior is wrong while looking correct (a marker at ~0.4 m would read "0.4°"-ish constant on the pitch scope).
- **Fix options:** (a) compute `bearing_pitch = degrees(atan2(y, hypot(x,z)))` and send `[x,y,z, bearing_pitch, bearing_yaw, distance]` to match docs, or (b) update the README/`build_udp_monitor_model.m` labels and the Simulink mapping to the actual code format. Both sides must change together — sender, receiver naming, and monitor model.

### 7.2 (HIGH) `udp_test_listener.py` cannot run its own happy path

`struct.unpack` is placed **after** the `continue`, so it is unreachable when the packet length is wrong *and* `seq` is never defined when the packet length is correct → `NameError: name 'seq' is not defined`. The listener is only a dev tool, but it cannot work. Fix: move the `unpack` below the length check and delete the orphaned line.

### 7.3 (MEDIUM) `SIMULINK_HOST` defaults to `127.0.0.1`

`gimbal_config.py` defaults to localhost; the README shows `192.168.1.50`. If `.env` is not set on the Pi, packets go to the Pi itself and the MATLAB computer never receives anything (UDP gives no error). Risk of silent cross-machine failure; the README warns about this but the default is easy to miss.

### 7.4 (MEDIUM) Endianness + color-order assumptions

- `GimbalUDPReceiver.m`: `typecast(...,'double')`/`'uint32'` use **MATLAB native endianness**. This matches the Python little-endian `<6dI` on x86, but would silently corrupt data on a big-endian host.
- Nicknamed "RGB888" frames are converted with `cv2.COLOR_BGR2GRAY`. Picamera2's RGB888 buffer is actually BGR-ordered per the code comment, so BGR2GRAY is (probably) correct — but a silent red/blue swap would be invisible and yet change grayscale contrast slightly. Worth one explicit comment at the conversion site, and confirming with a color-check image.

### 7.5 (MEDIUM) No `bearing_pitch` anywhere → missile model may lack a needed signal

Confirmed in §7.1. If the guidance loop needs the elevation angle between origin and target, it must be derived in Simulink from `(x, y, z)` (or the sender must start sending it). Sign convention must be re-verified when added (camera `y` is +down, so pitch is `+atan2(y, hypot(x,z))`, i.e. positive when target is below the boresight).

### 7.6 (LOW) `.env.example` uses `//` comments in an ini-style file

`python-dotenv` accepts them (recent versions), but the file also has empty values (`SIMULINK_HOST = ""`, `SIMULINK_PORT = `) which, if copied as-is, would set an empty host and could break the socket bind. Inline comments/empty values are a footgun.

### 7.7 (LOW) Dead / unused code confirmed in the runtime

- `pose_relative_to_marker()` (`servo.py`) — **never called**; it also carries a `y = camera_y; z = -marker_z` sign shuffle that, if ever used, would need careful verification.
- `pose_has_valid_depth()` — superseded by `pose_has_valid_step()` (which is the one used).
- `detect_marker_pose(return_rotation=True)` — the rotation matrix is returned but **discarded** by `servo.py` (`marker_array, _, corners, ids = ...`). The transform `pose_relative_to_origin` reconstructs orientation from gimbal angles instead.
- The `legacy_api` / `ArucoDetector` dual paths everywhere — harmless but adds surface area.
- `aruco_gimbal_picamera2.main()` — standalone demo loop with hardcoded placeholder intrinsics; not part of runtime.

### 7.8 (LOW) Calibration board script referenced but absent

`calibrate_camera_picamera2.py` says the board must "MUST MATCH generate_charuco.py exactly" — that generator is **not in this repo**. If someone reprints/replaces the board, the matching dims are only documented in comments, not enforced.

### 7.9 (LOW) Mechanical/safety assumptions worth re-checking

- `US_PER_DEGREE = 8.0` and centers 1500 µs are bench-calibrated; if a servo is replaced the tuning changes.
- Servos are driven to center immediately on start; if wired backwards the rig moves unexpectedly (README warns to check direction at low limits).
- `PAN_LIMIT_DEG` (±60) equals `ORIGIN_SEARCH_LIMIT_DEG` (60) so the search envelope uses the full travel — no electrical/mechanical stall margin if hard limits are less than 120° of travel.

### 7.10 (INFO) Multi-target behavior

`detect_marker_pose` with `excluded_marker_id` uses only the **first** non-origin marker in the detected list each frame. With multiple visible targets, which one gets tracked depends on detector ordering per frame — not specified. If multiple targets matter, filtering by nearest z (or a fixed target ID) should be added.

---

## 8. Cleanup Recommendations

### Runtime (keep as-is)
`servo.py`, `aruco_gimbal_picamera2.py` (module functions), `gimbal_config.py`, `camera_calibration.npz`, `calibrate_camera_picamera2.py`, `GimbalUDPReceiver.m`, `build_udp_monitor_model.m`, `.env.example`, `README.md`.

### Fix, then keep
- `udp_test_listener.py` — fix the unpack/`continue` bug (§7.2).
- `GimbalUDPReceiver.m` / `servo.py` — resolve the payload-format discrepancy and update README + monitor-model labels (§7.1).

### Delete or archive
- `aruco_gimbal_picamera2.main()` demo loop (or move to a `demo.py`).
- `pose_relative_to_marker()`, `pose_has_valid_depth()` dead code.
- `useful_readme.md` — duplicate of README content.
- `aruco_marker_stage.html` — keep only if the printing stage is still used; otherwise archive to `tools/`.
- Committed `coordinates.csv` — runtime artifact; add `gimbal_Amay/coordinates.csv` and `gimbal_Amay/z_distance.json` to `.gitignore`.

### Legacy outside scope
The top-level `arUco/` folder is the **old 2-value** (`[x,y]`) precursor of this pipeline and should not be confused with `gimbal_Amay/`. It will be covered in the Simulink pass.