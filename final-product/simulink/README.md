# Simulink side

Runs on the host computer. Receives the target pose from the Raspberry Pi
and flies a simulated interceptor against it.

```text
UDP target pose → target position and velocity → relative geometry
   → LOS rate → closing velocity → guidance → autopilot
   → actuator / hardware limits → force resolver (missile → inertial frame)
   → point-mass plant → feedback
```

## Files

| File | Role |
|---|---|
| `anjaneya_V12.slx` | Current guidance model (saved 10 Oct 2026, R2025a). |
| `GimbalUDPReceiver.m` | MATLAB System object that receives the pose packets. Must be on the MATLAB path. |
| `build_udp_monitor_model.m` | Builds `udp_monitor.slx`, a small model with one scope per received value, for testing the link. |
| `plotting/trajectory.m` | Static XY plot of missile and target after a run. |
| `plotting/live_plotting.m` | Minimal live plot while the model runs. |
| `plotting/live_plot_2.m` | Full engagement playback: trails, LOS line, intercept detection. |
| `plotting/target_live_plot.m` | Target-only playback with velocity arrow. |

The model logs `Xm Ym Xt Yt Vmx Vmy Vm Vtx Vty Vt gammaM gammaT Thrust
ThrustAngle` to the workspace through To Workspace blocks. The plotting
scripts read these from `out`.

## Requirements

- MATLAB and Simulink, R2025a (the model was saved in R2025a; older releases
  may not open it).
- No extra toolbox for the receiver. `GimbalUDPReceiver.m` uses Java's
  `DatagramSocket`, which ships with MATLAB.

## Setup

In MATLAB, from the repo root:

```matlab
addpath(fullfile(pwd, 'final-product', 'simulink'));
addpath(fullfile(pwd, 'final-product', 'simulink', 'plotting'));
which GimbalUDPReceiver   % must print the path in final-product/simulink
```

If `which` prints a path inside `learning/`, you have an old receiver on the
path. Remove it with `rmpath`.

Allow incoming UDP on port 25000 in this computer's firewall. On Windows the
first run usually shows a prompt for MATLAB; allow it on private networks.

## Test the link first

1. Run `build_udp_monitor_model`. It creates and opens `udp_monitor.slx` in
   the current folder (ignored by git).
2. Start the monitor model.
3. Send test data, either from the Pi (`python3 servo.py`) or from any
   computer with `python final-product/tools/send_circle.py <this-PC's-IP>`.
4. The X and Y scopes should move. With `send_circle.py` they trace a sine
   and cosine of amplitude 1.
5. Stop the monitor before starting V12. Only one model can hold port 25000.

## Run the guidance model

1. Open `anjaneya_V12.slx` and start it.
2. Start `servo.py` on the Pi. Data arrives about 5 s after the first
   detection, because of the playback delay in the sender.
3. After the run, call a plotting script, for example `live_plot_2`.

## Receiver block

| Item | Value |
|---|---|
| System object | `GimbalUDPReceiver` |
| Parameter `Port` | 25000 |
| Output 1 `pose` | 1×6 double. See the slot table in `../raspberry-pi/README.md` |
| Output 2 `seq` | uint32 packet counter |
| Output 3 `isNew` | true only on a step where a fresh packet arrived |

On each step the receiver reads every queued packet and keeps the newest, so
a slow model skips packets rather than lagging. If nothing arrives it holds
the last pose. Packets that are not exactly 52 bytes are ignored.

## Things to know before changing the model

- The engagement is **2-D**, in the camera's x–y plane. The camera's z
  (range) is not used for guidance.
- The measured target position is scaled ×100 inside the model before
  guidance. Keep that in mind when reading distances or hit thresholds.
- `isNew` is not used by the guidance logic yet, so a lost target is chased
  at its last known position.

These and other open points are in `../known-issues.md`.
