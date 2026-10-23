# Simulink model versions

All versions of the main guidance model, `anjaneya_V*.slx`, before the one in
`final-product/simulink/`. Dates are the last-saved dates in each file's
`metadata/coreProperties.xml`. All were saved in MATLAB R2025a.

| Model | Last saved | Notes |
|---|---|---|
| `anjaneya_V2.slx` | 2026-08-05 | |
| `anjaneya_V3.slx` | 2026-08-07 | |
| `anjaneya_V4.slx` | 2026-08-09 | |
| `anjaneya_V6.slx` | 2026-08-12 | Saved before V5 despite the number |
| `anjaneya_V5.slx` | 2026-08-15 | "Ideal" model, no hardware limits. Used by `compare_missile.m` |
| `anjaneya_V7.slx` | 2026-08-24 | "Limited" model, thrust shown. Used by `compare_missile.m` |
| `anjaneya_V9_fixed.slx` | 2026-09-05 | Same block layout as V10 |
| `anjaneya_V9.slx` | 2026-09-05 | Same block layout as V10. V9 is the first version with the `GimbalUDPReceiver` block (the 1 Sep early save already has it) |
| `anjaneya_V10.slx` | 2026-09-05 | Live UDP loop tuned for the real pipeline; the model the Sep reviews describe |
| `anjaneya_V11.slx` | 2026-10-10 | Same 161 blocks as V10; changes to three subsystems and the root wiring |

There is no V1 or V8 in any branch.

V12 (saved 2026-10-10, two hours after V11) is in `final-product/simulink/`.
It has the same 161 blocks as V11. The differences are in one subsystem, the
root diagram and the solver configuration.

## `early-saves/`

Earlier saves of three models that were sitting in a duplicate `simulink/`
folder at the root of `main`. They differ from the versions above.

| File | Last saved |
|---|---|
| `anjaneya_V5_2026-08-11.slx` | 2026-08-11 |
| `anjaneya_V7_2026-08-13.slx` | 2026-08-13 |
| `anjaneya_V9_2026-09-01.slx` | 2026-09-01 |

## `media/`

`engagement-2026-08-25.gif` is a screen recording of an engagement run,
originally a WhatsApp video from 25 Aug 2026 converted to GIF. Going by the
date it shows the V7 model; confirm before quoting it.

## Opening these files

Each model expects its plotting script and, from V9 onwards,
`GimbalUDPReceiver.m` on the MATLAB path. Opening an old model creates an
`slprj/` cache folder next to it. That folder is ignored by git, so do not
force-add it.

**To do:** a one-line note per version on what changed (guidance law, limits,
plant) would make this table much more useful.
