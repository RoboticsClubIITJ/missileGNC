# Final product

The working version of the seeker-in-the-loop simulator: a real camera on a
pan/tilt gimbal tracks a physical target, and a simulated interceptor in
Simulink is guided by what the camera sees.

## Architecture

```text
PHYSICAL SYSTEM (Raspberry Pi)                SIMULATION (host computer)

Target (ArUco marker)
   ↓
Camera (Picamera2)
   ↓
ArUco detection
   ↓
Pose estimation (solvePnP)
   ↓
Gimbal control → pan/tilt servos
   ↓
UDP transmission ──────── port 25000 ───────→ Simulink (GimbalUDPReceiver)
                                                 ↓
                                              Guidance (PN) ──→ MATLAB plotting
                                                 ↓
                                              Autopilot
                                                 ↓
                                              Plant (2-D point mass)
                                                 ↓
                                              Feedback to guidance
```

## Folders

| Folder | Contents | Guide |
|---|---|---|
| `raspberry-pi/` | Camera, detection, gimbal control and UDP sender | [`raspberry-pi/README.md`](raspberry-pi/README.md) |
| `simulink/` | Guidance model V12, UDP receiver, plotting scripts | [`simulink/README.md`](simulink/README.md) |
| `tools/` | Fake target sender, packet listener, marker stage page | [`tools/README.md`](tools/README.md) |
| `known-issues.md` | Open problems and risks | [`known-issues.md`](known-issues.md) |

## Startup order

1. Connect the Pi and the host computer to the same network.
2. On the host: add `final-product/simulink` to the MATLAB path and allow UDP
   port 25000 through the firewall.
3. Optional but recommended the first time: test the link with
   `build_udp_monitor_model` and `tools/send_circle.py`, no hardware needed.
4. On the Pi: set `SIMULINK_HOST` in `raspberry-pi/.env`, start `pigpiod`.
5. Start `anjaneya_V12.slx` in Simulink.
6. Run `python3 servo.py` on the Pi. Show it marker ID 0 to lock the origin,
   then the target marker.
7. Data reaches Simulink about 5 s after the first target detection (see
   known issue 2).
8. After the run, plot with `live_plot_2` or `trajectory`.

## Software versions

| Component | Version used |
|---|---|
| MATLAB / Simulink | R2025a |
| OpenCV | contrib build with the aruco module (4.7 or later API) |
| Python | 3, on Raspberry Pi OS |
| ArUco dictionary | `DICT_6X6_250` |
