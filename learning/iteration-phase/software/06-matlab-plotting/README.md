# Older MATLAB plotting scripts

The current plotting scripts are in `final-product/simulink/plotting/`.

- `animation_wala_trajectory.m`: an earlier playback of a finished run, with a
  rotating rocket icon, a thrust exhaust arrow and a closest-approach marker.
  `live_plot_2.m` in the final product replaced it.
- `compare_missile.m`: runs the ideal model (V5) and the limited model (V7)
  side by side, animates both, and prints a comparison table and an accuracy
  matrix (miss distance, time to closest approach, peak deflection). It is
  hard-coded to V5 and V7, both in `../05-simulink-versions/`. Updating it to
  compare V11 and V12 would be a useful next step.
