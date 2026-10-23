# Gimbal runtime: board-plane variant (Sep 2026)

This is the version of the gimbal runtime that lived on the `backup_14_Aug_26`
branch (last commit 21 Sep 2026). It branched off the same code as the final
runtime in `final-product/raspberry-pi/`, then went in a different direction.
It is kept here because the ideas are worth knowing, not because it is current.

## What was tried

1. **Projecting the target onto the board plane.** `project_to_board_plane()`
   takes the origin-relative target vector and re-expresses it in the origin
   marker's own axes (`origin_rotation.T @ v`). The idea: if the target moves
   on the same board as marker ID 0, its in-plane coordinates `(u, v)` should
   trace a clean shape even if the board is tilted relative to the camera.
   The out-of-plane value `w` should stay near zero, so it works as a
   detection-quality check.
2. **Live smoothing with an EMA.** `ExponentialSmoother` (alpha = 0.3) is a
   causal low-pass filter on `u` and `v`, reset whenever tracking is lost.
   It was chosen over Savitzky-Golay because it only uses past samples, so
   it can run live.
3. **Sending marker orientation.** `detect_marker_pose()` returns a six-value
   pose `[x, y, z, pitch, yaw, distance]`, where pitch and yaw are the
   marker's own orientation from `cv2.RQDecomp3x3`.
4. **A tighter jump filter.** `pose_has_valid_step()` rejects frame-to-frame
   jumps above 0.15 m.

## What the packet contained

```text
[u_smooth, v_smooth, w, marker_pitch, bearing_yaw, distance] + uint32 seq
```

Same 52-byte `<6dI` layout as the final version, but different meanings in
slots 1 to 4. Simulink wiring built for one version is wrong for the other.

## Other settings that differ from the final version

| Setting | This variant | Final runtime |
|---|---|---|
| `MARKER_SIZE` | 0.099 m | 0.0738 m |
| `ERROR_DEADBAND_DEG` | 0.80 | 0.00 |
| Jump filter `max_step_m` | 0.15 m | 1,000,000 m (effectively off) |
| Sender | queue, 20 Hz, resend last pose | 5 s delayed, interpolated, Gaussian-smoothed, 50 Hz |

## What was retained and what was discarded

The final runtime kept the origin-lock search, `pose_relative_to_origin()`
and the four-value detector pose. It dropped the board-plane projection, the
EMA and the orientation angles in the packet.

**To do:** the reason for dropping the board-plane approach was not written
down. Whoever worked on it should add one or two lines here on what happened
when it was tested.

## Running it

It needs the same hardware and `camera_calibration.npz` as the final runtime.
Copy `final-product/raspberry-pi/camera_calibration.npz` into this folder and
run `python3 servo.py`. Do not run it at the same time as the final runtime,
since both bind the same GPIO pins and UDP port.
