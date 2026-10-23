# Learning / iteration phase

How the project got to the final product: experiments, alternative
approaches, intermediate versions and things that were later replaced.
Nothing here is needed to run the final product. It is kept so the next
team can see what was tried and why.

## How the project evolved

Dates are the last-saved dates in the `.slx` metadata and the last commit
dates of the old branches. Where nothing recorded a date, none is given.

| Stage | When | Where |
|---|---|---|
| Individual ArUco detection and pose estimation | Start of project | `software/01-aruco-detection/` |
| Individual plant models | 23 Jul – 4 Aug 2026 | `software/02-plant-models/` |
| Shared guidance model, V2 to V7 | 5 – 24 Aug 2026 | `software/05-simulink-versions/` |
| Camera calibration (ChArUco) and pose estimation on the Pi | — | `software/01-aruco-detection/`, final calibration in `final-product/raspberry-pi/` |
| First Raspberry Pi → MATLAB UDP link | Before 1 Sep 2026 (the V9 early save already has its receiver) | `software/03-first-udp-link/` |
| Pan/tilt servos, gimbal + ArUco integration | — | `final-product/raspberry-pi/`, variant in `software/04-gimbal-board-plane-variant/` |
| UDP receiver inside the guidance model | V9, 1 Sep 2026 | `software/05-simulink-versions/early-saves/` |
| Live plotting and target streaming | Around V10, early Sep 2026 (old branch commits "live plotting in simulink" → "v10 of simulink") | `software/06-matlab-plotting/`, current scripts in `final-product/simulink/plotting/` |
| Board-plane gimbal variant | Last commit 21 Sep 2026 | `software/04-gimbal-board-plane-variant/` |
| V9 → V12 tuning against the live pipeline | 5 Sep – 10 Oct 2026 | `software/05-simulink-versions/`, V12 in `final-product/simulink/` |

## What each folder holds

| Folder | What was tried | Outcome |
|---|---|---|
| `software/01-aruco-detection/` | Each member's own ArUco detection and calibration, on a laptop or USB webcam | Detection with `solvePnP` and ChArUco calibration carried into the final runtime. The webcam was replaced by the Pi Camera with its own ChArUco calibration |
| `software/02-plant-models/` | Each member built a missile plant separately, from point-mass models to a rigid-body model with attitude dynamics (`Akshat_plant/`) | The shared `anjaneya` model uses a 2-D point-mass plant. The rigid-body work was not carried forward |
| `software/03-first-udp-link/` | Stream the marker pose from the Pi to Simulink, no servos | Worked, and set the 52-byte packet size every later version kept. Replaced by the gimbal runtime |
| `software/04-gimbal-board-plane-variant/` | Express the target in the origin board's plane, smooth it with an EMA, send marker orientation | Not kept. The reason is not recorded yet (see that folder's README) |
| `software/05-simulink-versions/` | Every version of the guidance model | V12 is the final one |
| `software/06-matlab-plotting/` | Earlier playback and the ideal-vs-limited comparison | `live_plot_2.m` replaced the playback. The comparison still works on V5 and V7 |
| `hardware/` | Components, wiring, power-up order | Reference |
| `testing/` | Subsystem test order and plant accuracy tests | Results still to be recorded |
| `reviews/` | AI-assisted code reviews of the Sep 2026 code | Confirmed findings moved to `final-product/known-issues.md` |

## Adding to this archive

Don't delete an approach that failed or was replaced. Move it here, in its
own folder, with a README that answers:

1. What was tried
2. Why it was tried
3. What happened
4. What was kept
5. What was dropped
6. What is still unresolved
7. What the next team should study

Keep generated files out (`slprj/`, `.slxc`, autosaves, run logs). The
`.gitignore` at the repo root already covers the usual ones.

## Open to-dos for this archive

- `04-gimbal-board-plane-variant/README.md`: why the board-plane approach was dropped.
- `05-simulink-versions/README.md`: one line per version on what changed.
- `testing/README.md`: the result of each test.
- `hardware/README.md`: photos, wiring diagram, exact servo and camera models.
- `software/01-aruco-detection/Palak_ArUco/`: this folder was called
  `Disha_ArUco` on the backup branches. Confirm whose code it is and rename
  if needed.
