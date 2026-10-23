# Software experiments

Numbered in the order the project developed. Each folder has its own README
where there is something to explain.

| Folder | Contents |
|---|---|
| `01-aruco-detection/` | Per-member ArUco detection and calibration |
| `02-plant-models/` | Per-member missile plant models in Simulink |
| `03-first-udp-link/` | First Raspberry Pi → Simulink pose stream, before the gimbal runtime |
| `04-gimbal-board-plane-variant/` | Alternative gimbal runtime from the Sep 2026 branch |
| `05-simulink-versions/` | `anjaneya_V2` to `V11`, earlier saves and a recording |
| `06-matlab-plotting/` | Older plotting and the ideal-vs-limited comparison |

## MATLAB path

`03-first-udp-link/GimbalUDPReceiver.m` has the same class name as the
final receiver in `final-product/simulink/`, but is an older version. Add a
learning folder to the MATLAB path only while you use it, and run
`which GimbalUDPReceiver` before running the final model.
