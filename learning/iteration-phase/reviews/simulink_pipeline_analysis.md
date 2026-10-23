> **Note:** this is an AI-assisted review written in Sep 2026 against the
> V10-era code (`gimbal_Amay/` and `simulink/` on the backup branch). Folder
> names refer to that old layout. Findings were not re-checked against V12 or
> the Oct runtime; the ones confirmed so far are in `final-product/known-issues.md`.

# Simulink Pipeline Analysis — `anjaneya_V10.slx` + MATLAB scripts

> Deep-dive of the `simulink/` folder: the MATLAB analysis/visualization scripts and the
> current model `anjaneya_V10.slx` (the guidance & plant simulator that the gimbal/camera
> pipeline feeds into over UDP). Companion to `gimbal_pipeline_analysis.md`.

---

## 1. What this folder is (2-minute overview)

The `simulink/` folder holds the **flight model + guidance loop** of the TVC (thrust-vector
control) interceptor that runs **in Simulink** and, since V9/V10, closes the loop with the
real camera pipeline over UDP port 25000:

```
gimbal_Amay (python) ──UDP 25000──▶ anjaneya_V10.slx (Simulink)
   pose: [x, y, z, distance, ...]            │
                                            ▼
   measured marker offset  ──×100──▶ "target position"  Xt, Yt
   d/dt of that             ──▶ "target velocity"       Vtx, Vty
                                            ▼
   guidance (relative geometry) → lambda_dot
   Navigation (PN, N=4)          → a_cmd (±108)
   autopilot  (thrust vectoring) → delta
   hardware limitation (fin servo) → delta_limited
   force resolver                → T_x, T_y (body force in world frame)
   Plant2 (point-mass, g, m=2 kg) → xp, yp, vpx, vpy, gamma
                                            ▼
   recorded to base workspace + scopes, fed back into guidance (8 in),
   and a MATLAB-function "stop condition" halts the sim on hit or miss.
```

It is a **2-D planar engagement**: the plane of motion is the camera's X–Y (image-offset)
plane, i.e. the marker's lateral/elevation offset from the gimbal boresight, scaled ×100.
Down-range distance (camera `z`) is **not** used.

---

## 2. File relevance map

| File | Role | Current / legacy |
|---|---|---|
| `anjaneya_V10.slx` | **Current model** (latest save 2026‑09‑05, v1.161). Live UDP loop + PNG + TVC plant. | **CURRENT** |
| `anjaneya_V9_fixed.slx`, `anjaneya_V9.slx` | Immediate predecessors — **structurally identical** block layout to V10 (same block set, same SIDs); differ only in minor tuning. V10 is the newest. | legacy (superseded) |
| `anjaneya_V2.slx` … `anjaneya_V7.slx` | Earlier evolutions: V5 = "ideal" (no hardware limits), V7 = "limited" (thrust showing). Referenced by `compare_missile.m`. | legacy |
| `anjaneya_V10.slx.autosave` | Stale Simulink autosave of V10 (same timestamp). Safe to delete. | cleanup |
| `trajectory.m` | Static XY plot of `out.Xm/Ym` vs `out.Xt/Yt` after a run. | relevant |
| `live_plotting.m` | Minimal **live** animatedline: missile + target trails; reads `Xt/Yt` from base workspace while `sim()` runs. | relevant |
| `live_plot_2.m` | The **flagship live engagement animation**: playback of logged `out`, intercept detection (R<0.5 m), LOS line, divergence warning. | relevant |
| `target_live_plot.m` | Live target-only animation + velocity arrow, from `out.Xt/Yt/Vtx/Vty`. | relevant |
| `animation_wala_trajectory.m` | Older fancier playback with a rotating **rocket icon** + thrust exhaust arrow, closest-approach marker. | legacy variant of `live_plot_2.m` |
| `compare_missile.m` | Side-by-side IDEAL (V5) vs LIMITED (V7) comparison: animation, run-diff check, accuracy matrix. **Hard-coded to V5/V7, not V10.** | legacy / needs updating |
| `Akshat_plant/init_sim.mlx` | Live Script that sets alternative rigid-body params (m=15 kg, I_zz, L_cg = CG-to-nozzle, g, dt=0.01) for `Akshat_plant/model.slx`. | legacy (person's own plant) |
| `Akshat_plant/model.slx` | A teammate's older 6-DOF-ish plant model (attitude dynamics). Not used by V10. | legacy |
| `Aman_plant.slx`, `Disha_plant.slx`, `Palak_Plant_V1.slx`, `pranav_plant_iteration1.slx` | Individual plant-model experiments by different members. | legacy |
| `.fuse_hidden0000d6e70000001a` | Orphaned file (leftover of a moved/crashed transfer — a `.fuse_hidden` inode). | cleanup |
| `*.gif` , `slprj/` | Scratch/preview artifacts; `slprj/` is regenerable build cache. | cleanup |

---

## 3. MATLAB scripts — input → processing → output

### 3.1 `trajectory.m` (static plot)
- **Input:** workspace variable `out` (a `Simulink.SimulationOutput`) with `Xm, Ym, Xt, Yt` timeseries.
- **Processing:** extracts `.Time`/`.Data`, plots missile trail vs target trail on shared axes (auto limits), NaN/Inf checks, prints start/end positions.
- **Output:** static figure + console diagnostic prints.

### 3.2 `live_plotting.m` (minimal live plot)
- **Input:** model name; uses `set_param(mdl,'SimulationCommand','WriteDataLogs')` after `set_param(...,'SimulationCommand','start')`, then reads **base-workspace** `Xt`/`Yt` (from the model's To-Workspace blocks) and re-plots them via `animatedline` as the run proceeds.
- **Processing:** while loop over `get_param(mdl,'SimulationStatus')`; `evalin('base','Xt.Data')`. Adds points to the animatedline each poll.
- **Output:** live target/missile trail; figures. (Bare-bones compared to 3.3.)

### 3.3 `live_plot_2.m` (full live engagement playback)
- **Input:** model `anjaneya_V10` (or reuses existing `out` from base workspace); settings `PLAYBACK_SPEED=1`, `FRAME_DT=0.02 s`, **`R_HIT=0.5 m`** (matches the model's own stop condition), `AUTO_RESCALE` axis fitting.
- **Processing:**
  1. `local_getSimOutput()` — reuse base `out` if present & valid, else `sim(modelName)`.
  2. Resamples missile & target trajectories onto a uniform 0.02 s grid (`interp1`, extrap).
  3. Computes `range_t = hypot(Xt−Xm, Yt−Ym)`; finds first frame where `range ≤ R_HIT`.
  4. Animates trails, missile/target markers, dashed LOS line, on-screen telemetry; **pauses at intercept** if found.
- **Output:** animation + "INTERCEPT at t=… range=…" print, or a **warning with the known divergence-bug signature** when final range > 5× initial range (references "Autopilot/Force-Resolver sign chain and target velocity Derivative blocks").

### 3.4 `target_live_plot.m`
- **Input:** `out` (needs `Xt,Yt`; optional `Vtx,Vty`).
- **Processing:** uniform resampling; `quiver` velocity arrow at the target, speed readout; target trail.
- **Output:** target-only animation. (Good diagnostic for seeing how `Vtx/Vty` (derivatives of the ×100 pose) behave — noisy arrows are a red flag for the guidance.)

### 3.5 `animation_wala_trajectory.m` (older)
- **Input:** `out` (logs `Xm Ym Xt Yt`; derives `Fx,Fy,heading_angle,scaleFactor,maxPosRange` from logged thrust/angles in the first lines — the script computes thrust vector `theta_T = heading + delta_lim`).
- **Processing:** rocket drawn as a `hgtransform` polygon that translates+rotates with `atan2(vmy,vmx)`; thrust arrow = `T·[cos θ_T, sin θ_T]`; marks closest approach (`minIdx`).
- **Output:** animation + same sim-data/NaN/Inf console report. Superseded by 3.3.

### 3.6 `compare_missile.m` (IDEAL vs LIMITED benchmarking)
- **Input:** **hard-coded** `MODEL_IDEAL='anjaneya_V5.slx'`, `MODEL_LIMITED='anjaneya_V7.slx'`; `HIT_RADIUS=5.0 m`; expects logs `Xm Ym Xt Yt Vmx Vmy Vm Vtx Vty Vt gammaT gammaM Thrust ThrustAngle` (timeseries).
- **Processing:** runs both models; per-run uniform grids; common-grid resample; max-|diff| "run diff check"; resample/derive (`deriveRun` builds range, CPA, heading, thrust-inertial-frame); side-by-side tiled animation over the longer run; signal table; accuracy matrix (miss distance, time-to-CPA, total distance, peak deflection); auto deg/rad detection for `ThrustAngle`.
- **Output:** animation + console tables + intercept verdict (`miss < 5 m`). Prints an explicit warning if the two runs are numerically identical.
- **Caveats for reuse:** models are V5/V7 (no UDP receiver — pure simulations); `HIT_RADIUS` 5.0 m ≠ V10's 0.5 m stop; would need model names bumped to compare against the live-model evolution.

---

## 4. `anjaneya_V10.slx` — model deep-dive

### 4.1 Model metadata / solver
- Saved as MATLAB **R2025a**, GRT (`grt.tlc`), last saved 2026‑09‑05 by `kuppa`.
- Solver: `VariableStepAuto` (**ode45**), `RelTol 1e-6`, **`MaxStep 0.05 s`**, `StartTime 0.0`, **`StopTime 100000`**.
- Logging: `SaveOutput on`, `SaveFormat= StructureWithTime`, `SignalLogging on`. To-Workspace blocks write `timeseries` into `out.*` (name = variable name).
- 7 Stateflow charts (all trivial, single-`fcn` "MATLAB Function" style S-functions), everything inside them is pure combinatorial C/EML — i.e. no Stateflow *states*, just function wrappers.

### 4.2 Top-level block inventory (SIDs)
```
299→300   guidance1 inputs      (target geometry)  292
280       Xt  (pose.x ×100)         281  Yt  (pose.y ×100)
340 Vtx = d/dt(pose.x×100)          341 Vty = d/dt(pose.y×100)
278 Xm, 279 Ym, 349 Vmx, 350 Vmy    284 gammaM, 283 ThrustAngle(δ_lim)
282 Thrust (const 300) 285 gammaT   347 Vt(3) 348 Vm(3)
242 Accelaration2 (a_x,a_y) scope   287 Trajectory3 Record XY
368 Scope (raw x,y,z offsets)       369 Scope1 (UDP seq)
268 Scope3 (λ̇)  269 Scope4 (xp)   270 Scope5 (yp)  271 Scope6 (δ_cmd vs δ_lim)  289 Velocity2
246 MATLAB Function2 (stop)  [247 Mass1=2] [248 N1=4] [330 thrust1=300]
249 Navigation1  267 Saturation1(±108)  290 autopilot1  291 force resolver1  250 Plant2  331 hardware limitation2
365 Demux(6)  366 MATLAB System (GimbalUDPReceiver)  370/371 Derivative 372/373 Gain ×100  243 Clock1  277 Stop Simulation1
```

### 4.3 Signal flow (wiring, verified from `system_root.xml` Src/Dst)

1. **UDP → pose.** `MATLAB System` block (366) = `gimbal_Amay/GimbalUDPReceiver.m` (port 25000). `out:1` = 6‑element `pose` → `Demux` (365). `out:2` = `seq` → Scope1. `out:3` = `isNew` → **unused** (important, see §5).
2. **Image offsets.** Demux ch1 = pose[0]=x and ch2 = pose[1]=y go to Scope (368), and each into a **Gain ×100** (372/373). ch3 = pose[2]=z → Scope only (not used). Channels 4–6 unused.
   - `372#out (100·x)` → `Xt` To-Workspace (280), **guidance in:7** (`R_tx`), and `Derivative` (370) → `Vtx` (340) + **guidance in:5** (`v_tx`).
   - `373#out (100·y)` → `Yt` (281), **guidance in:8** (`R_ty`), `Derivative1` (371) → `Vty` (341) + **guidance in:6** (`v_ty`).
3. **Guidance `guidance1` (292)** — 8 in / 6 out:
   `in:1 x_p`(xp) `:2 y_p`(yp) `:3 v_px` `:4 v_py` `:5 v_tx` `:6 v_ty` `:7 R_tx`(Xt) `:8 R_ty`(Yt)
   ```
   R_x = Xt − xp     (Sub2)        R_y = Yt − yp    (Sub3)
   V_rx = Vtx − vpx  (Sub)        V_ry = Vty − vpy  (Sub1)
   |r|  = sqrt(R_x² + R_y²)                       → out4 (R)
   λ̇    = (R_x·V_ry − R_y·V_rx) / (R_x²+R_y²)     → out1 (λ̇)   [MATLAB Fcn]
   v_c  = −(V_rx·R_x + V_ry·R_y) / max(|r|, 1e-3) → out2 (v_c) [Range Floor guard]
   v_m  = sqrt(vpx² + vpy²)                       → out3       [VM=missile speed]
   γ_T  = atan2(Vty, Vtx)                         → out5       [target heading, LOGGED ONLY]
   V_t  = sqrt(Vtx² + Vty²)                       → out6       [target speed, LOGGED ONLY]
   ```
   (Dead branch: `sqrt(V_rx²+V_ry²)` computed at 312 but its output is **unconnected**.)
4. **Navigation `Navigation1` (249)** — pure proportional navigation + gravity comp (chart_21):
   ```
   a_guide = (v_c > 0) ? N·v_m·λ̇ : 0     # N = 4  (N1)
   a_cmd   = a_guide + g·cos(γ)          # g = 9.81, γ = missile flight-path angle
   ```
5. `a_cmd` → **Saturation1 ±108 (m/s²)** → `autopilot1` in:2.
6. **Autopilot `autopilot1` (290)** (chart_50) — solves thrust angle for the demanded lateral accel:
   ```
   s   = m·a_cmd / T            (m = Mass1 = 2 kg, T = thrust1 = 300 N)
   s   = clamp(s, ±0.99)
   δ   = asin(s)                → commanded thrust deflection
   ```
7. `δ` → **hardware limitation2 (331)** (fin-servo limit chain, blocks 332→339):
   ```
   δ_lim = RateLimit( Sat( δ + 8.33 · L{ dδ/dt } , ±0.72 rad ), ±0.523 rad/s )
           L = DiscreteFilter 1/(1+0.005 s)
   ```
   I.e. commanded deflection plus a rate-proportional lead term, then magnitude-limited to
   **±0.72 rad (≈41°)** and slew-limited to **±0.523 rad/s (30°/s)**. Output logged as `ThrustAngle` and used by the force resolver. `Scope6` plots δ_cmd vs δ_lim side by side.
8. **Force resolver `force resolver1` (291)** (chart_59):
   ```
   θ_T = δ_lim + γ            # thrust direction in world frame = body deflection + heading
   T_x = T·cos θ_T
   T_y = T·sin θ_T
   ```
9. **Plant `Plant2` (250)** (subsystem, mass point) (charts 32 & 42):
   ```
   a_x = T_x / m
   a_y = T_y / m − g                       # gravity acts −y
   vpx = ∫a_x (IC 0.5)   vpy = ∫a_y (IC 0.5)
   xp  = ∫vpx (IC 0)     yp  = ∫vpy (IC 0)
   γ   = atan2(vpy, vpx)                   # flight-path angle (from velocity)
   ```
7 outputs: `a_x, a_y, xp, yp, vpx, vpy, γ`. `xp/yp` → `Xm/Ym` + Trajectory3 + guidance.
`vpx/vpy` → `Vmx/Vmy` + Velocity2 + guidance. `γ` → `gammaM` + Navigation1 (gravity comp) + force resolver.
10. **Stop condition `MATLAB Function2` (246)** (chart_12) — fed `R` (from guidance out4), `v_c` (out2), `t` (Clock1):
    ```
    if t > 0.05 && (R < 0.5 || v_c <= 0) && t < 100000   → stop simulation
    ```
    `R < 0.5 m` = **intercept**; `v_c <= 0` = **closing speed non-positive**, i.e. target pulling
    away / miss / divergence → simulation halts.
11. **Logging to `out`** (To-Workspace, uniform `timeseries`): `Xm,Ym,Xt,Yt, Vmx,Vmy,Vm, Vtx,Vty,Vt, gammaM, gammaT, Thrust, ThrustAngle` plus Record `Trajectory3`.

---

## 5. Problems, risks & suspicious design choices

1. **Target velocity = derivative of a noisy, ×100-scaled pose (biggest issue).**
   `Vtx = d/dt(100·x)`, `Vty = d/dt(100·y)`. The Derivative block also amplifies the Poisson/pose
   noise and any lost-frame gaps by 100×. `Vtx/Vty` feed `λ̇`, `v_c`, `v_m`? no — `v_m` is missile;
   but they feed `λ̇`, `v_c`, plus logged `V_t`. The **noise on λ̇** plus the **gain `N·v_m`** is the
   most plausible cause of the *"known missile-divergence bug"* the scripts warn about. No smoothing
   (e.g. Kalman/EMA) on this path whatsoever. The only filter in the model is on the *fin* path.

2. **The ×100 scale factor is undocumented and dimen-suspicious.**
   Camera `x,y` are **already metres** (pose from solvePnP). Multiplying by 100 turns a ±0.3 m
   offset into a ±30 m "target position". Either it is an intentional hack to make the sim work at
   its current units, or wrongly interpreted as "cm → m". Either way it must be reviewed — AND it
   scales `Vtx/Vty` by the same 100 (see #1). There is no unit annotation anywhere in the model.

3. **The `z` (down-range) channel is dropped.** The model works in the plane of the camera
   offsets only. There is **no real range feedback** — the missile is guided purely by the image
   offsets `(x,y)`, and intercept is declared when `Xt−xp`+`Yt−yp` distance (in units of 100×offset)
   < 0.5 m. The tower/wall distances in `distance` channel (pose[3]) are never used.

4. **`isNew` (UDP fresh-flag) is unconnected.** The `MATLAB System` receiver's third output
   (`isNew`) is dropped; the guidance is driven by the *last received* pose. If the marker is
   occluded, the receiver holds the stale pose → the "target" freezes and the missile chases a
   ghost point. No time-out / validity logic.

5. **Sim-time vs wall-time mismatch.** Variable-step ode45 with `MaxStep 0.05`; the UDP data is
   only sampled on integration steps. If the model runs faster than 20 Hz in wall-clock time the
   sensor appears to "stall" between packets; if much slower, guidance sees blocky pose changes.
   Derivative blocks make this discontinuity a jitter spike. There is no real-time pacing/tasking.

6. **Coordinate/sign convention risk (image plane vs world).** Camera `y` is **+down**; the plant
   integrates with gravity **−y** (`a_y = T_y/m − g`). `Yt = +100·pose.y` therefore maps "marker
   below boresight" to a *positive* Y in a world where up is positive — the vertical guidance acts
   the **wrong way** unless the sign is deliberately compensated. Same question for `x` (gimbal yaw
   mirroring). This is worth validating empirically before trusting vertical convergence.

7. **Aggressive dynamics vs measured sizes.** Missile 2 kg with 300 N thrust → ~150 m/s² axial
   accel, and `a_cmd` up to ±108 m/s², against target positions of only a few metres (100 × cm).
   Intercept runs complete in fractions of a sim-second; meaningful visual/experimental comparison
   with the real tower is therefore ill-conditioned. Also **no drag, no aero, no mass depletion**,
   and `γ` comes from velocity direction (fin body attitude is absent).

8. **Intercept/miss verdict depends on noisy `v_c`.** Because stop-condition `v_c <= 0` uses the
   differentiated (noisy) target velocity, a spurious negative can abort the run as a "miss".
   Prefer a smoothed `v_c` or use `R` trend.

9. **Autopilot δ vs the ±0.72 rad servo limit interplay.** `δ = asin(clamp(m·a_cmd/T))` can ask for
   up to ±1.43 rad (asin 0.99), but the servo saturates at ±0.72 rad — i.e. the autopilot commonly
   begs for more deflection than the actuator can give, and the force resolver then uses δ_lim.
   Acceptable (that's the point of the block) but means `a_cmd` is frequently not achieved → log
   saturation time to quantify.

10. **Stale comparison tools.** `compare_missile.m` targets V5/V7 (no UDP), HIT_RADIUS 5 m vs
    V10's 0.5 m; `animation_wala_trajectory.m` duplicates `live_plot_2.m`. All live scripts depend
    on clips/axes conventions that drift with model renames.

---

## 6. Version history (from model metadata)

| Model | Last saved | Note |
|---|---|---|
| pranav_plant_iteration1 | 2026‑07‑23 | earliest plant attempt |
| anjaneya_V2 | 2026‑08‑05 | |
| anjaneya_V3 | 2026‑08‑07 | |
| anjaneya_V4 | 2026‑08‑09 | |
| anjaneya_V6 | 2026‑08‑12 | (saved before V5 despite numbering) |
| anjaneya_V5 | 2026‑08‑15 | IDEAL (no hardware limits) — used by compare_missile |
| anjaneya_V7 | 2026‑08‑24 | LIMITED — used by compare_missile |
| anjaneya_V9_fixed | 2026‑09‑05 18:09 | identical block layout to V10 |
| anjaneya_V9 | 2026‑09‑05 18:15 | identical block layout to V10 |
| **anjaneya_V10** | **2026‑09‑05 18:20** | **current; adds live UDP loop tuned for the real pipeline** |

Git history on `backup_14_Aug_26`: `… "testing again" → "added queue" → "changes in aruco marker gui" → "live plotting in simulink" → "v10 of simulink" → "live plots"`.

---

## 7. Recommendations / cleanup

1. **Filter the target pose before the derivative** (real Kalman or at least a 2nd-order low-pass +
   smooth differentiation) — this is the single highest-value fix against divergence.
2. **Document/lock the ×100 unit decision**; better, pass real metres and rescale thrust/mass or
   divide range threshold accordingly.
3. **Wire `isNew`/`seq` into guidance** (hold-last / lost-target logic) instead of ghost-chasing; log
   a `valid` flag into `out`.
4. **Recheck the `y` sign** convention (camera-down vs world-up) with a marker placed physically
   above/below boresight and confirm the missile flies the right way.
5. Update `compare_missile.m` to V9_fixed vs V10 (or V10 vs V7) and harmonise `HIT_RADIUS`.
6. Delete stale `.xslx.autosave`, `.fuse_hidden0000d6e70000001a`, `slprj/`, and archive the
   per-person plant models (`Aman/Disha/Palak/pranav/Akshat`) out of the "current" folder.
7. Remove the dead `sqrt(Vrx²+Vry²)` branch (312) in guidance1 for clarity.

---

*Generated by repo deep-dive (`.slx` parsed via its SLX/XML). All wiring & code listings verified against `anjaneya_V10.slx`.*