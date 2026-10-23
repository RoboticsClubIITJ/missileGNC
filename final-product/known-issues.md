# Known issues and risks

Open problems in the final product, for whoever works on this next. Each item
says what was checked and where. "Checked" means read from the code or from
the V12 `.slx` XML in Oct 2026, not tested on the rig.

## 1. UDP packet formats

Every version uses the same 52-byte layout (`<6dI`: six little-endian doubles
plus a uint32 sequence number), but three different meanings for the six
values have existed.

| Version | Where | Slots 1–6 |
|---|---|---|
| First link (Aug) | `learning/.../03-first-udp-link/` | x, y, z, **marker** pitch, **marker** yaw, distance |
| Board-plane variant (Sep) | `learning/.../04-gimbal-board-plane-variant/` | u, v, w (board plane, smoothed), marker pitch, bearing yaw, distance |
| **Final** (Oct) | `raspberry-pi/servo.py` | x, y, z (origin frame), **distance**, bearing yaw, distance |

- In the final version, slot 4 is labelled `bearing_pitch` in
  `GimbalUDPReceiver.m` and in `build_udp_monitor_model.m`, but `servo.py`
  sends the distance there. Slot 6 sends the distance again. Bearing pitch is
  never computed. (`pose_relative_to_origin()` in `servo.py`.)
- **Impact today: none on guidance.** In V12 only slots 1 and 2 feed the
  guidance subsystem. Slot 3 goes to a scope and slots 4–6 are unconnected.
  Anything that starts using slot 4 will be fed range, not an angle.
- **Fix:** compute `bearing_pitch = degrees(atan2(y, hypot(x, z)))` in
  `pose_relative_to_origin()`, or rename the label everywhere. The sender,
  receiver comment, monitor scope names and `tools/udp_test_listener.py`
  must change together.

## 2. The sender is 5 seconds behind real time

`FixedRateSender` in `servo.py` (`period_s=0.02, delay_s=5.0,
smooth_sigma_s=0.3`) plays poses back 5 s late, interpolated and smoothed.
The stream is smooth and evenly spaced, but Simulink guides against where
the target was 5 s ago. For a closed seeker loop this delay matters more
than the noise it removes. Decide whether the delay is intended and write
the reason here, or reduce it.

## 3. Multiple targets

`detect_marker_pose()` in `aruco_gimbal_picamera2.py` tracks the **first**
non-origin marker in OpenCV's detection list (`matching_indices[0]`). The
order of that list is not guaranteed, so with two targets in view the
tracked one can switch between frames. The first UDP link took the **last**
marker instead. Fix: track one fixed target ID, or the nearest marker.

## 4. Pose validation is effectively off

- `pose_has_valid_step()` is called with its default `max_step_m=1000000.0`,
  so no jump is ever rejected. The Sep variant used 0.15 m.
- `ERROR_DEADBAND_DEG` in `gimbal_config.py` is 0.00 (was 0.80).

If these were debugging values, restore them. If they are deliberate, say
why here.

## 5. Unused code

In `raspberry-pi/servo.py`:

- `pose_relative_to_marker()` is never called.
- `pose_has_valid_depth()` is never called (replaced by `pose_has_valid_step()`).
- The old 20 Hz `FixedRateSender` is still there, commented out.
- `detect_marker_pose(return_rotation=True)` returns a rotation matrix that
  the tracking loop discards.

In `raspberry-pi/aruco_gimbal_picamera2.py`, `main()` is a standalone demo
with its own placeholder settings and is not used by `servo.py`.

Remove these, or move them to the learning archive with a note.

## 6. Generated Simulink files

`slprj/`, `.slxc`, `.slx.autosave`, `.slx.original`, `.asv` and
`.fuse_hidden*` were committed before. They are deleted and now in
`.gitignore`. If one shows up in `git status`, do not add it.

## 7. Duplicate scripts

- Four plotting scripts overlap: `plotting/live_plotting.m`,
  `plotting/live_plot_2.m`, `plotting/target_live_plot.m`, and the older
  `learning/.../06-matlab-plotting/animation_wala_trajectory.m`.
  `live_plot_2.m` covers most of what the others do.
- Two different classes are both called `GimbalUDPReceiver`: the final one
  in `simulink/` and the first one in `learning/.../03-first-udp-link/`.
  Only one can be on the MATLAB path. Check with `which GimbalUDPReceiver`.

## 8. Coordinate frames

| Frame | Axes | Defined in |
|---|---|---|
| Camera | x right, **y down**, z forward, metres | OpenCV `solvePnP` |
| Origin | Camera frame rotated back to the pan/tilt angles at origin lock, with marker ID 0's position subtracted | `pose_relative_to_origin()` |
| Simulink engagement | 2-D plane, x and y of the origin frame × 100 | Gain blocks after the Demux in V12 |

Not yet checked: whether the model treats y as up or down. Camera y is
positive **down**. Put a marker above the boresight and confirm the missile
turns up. Also write down what the ×100 scale means physically (metres to
"simulation metres"?), since hit thresholds and thrust depend on it.

## 9. Target velocity estimate

In V12, target velocity is the output of two unfiltered **Derivative**
blocks on the ×100 positions. Differentiating noisy camera positions
amplifies the noise, and the ×100 gain amplifies it further. The 5 s
smoothing in the sender is currently what hides this. Validate the velocity
against a target with known motion (`tools/send_circle.py` gives one), and
consider a filtered derivative or a Kalman filter in the model.

## 10. Model scope

V12 is a **2-D point-mass** model with gravity, thrust vectoring, actuator
limits and proportional navigation (N = 4). It is not an aerodynamic or
6-DOF model. There is no drag, lift, attitude dynamics or mass change.
Results should be described as a guidance-loop demonstration, not as a
missile performance prediction.

## 11. Smaller items

- `gimbal_config.py` falls back to a hard-coded IP from a past session when
  there is no `.env`. A wrong IP fails silently with UDP.
- `calibrate_camera_picamera2.py` says the board must match
  `generate_charuco.py`, which is not in any branch. The board size is only
  recorded in that script's constants.
- `learning/.../06-matlab-plotting/compare_missile.m` still compares V5 and
  V7, not the current model.
- `GimbalUDPReceiver.m` uses `typecast`, which assumes the host is
  little-endian. True for every normal PC, but worth knowing.
- `isNew` from the receiver is not used by V12, so when the target is lost
  the missile keeps chasing its last position.
