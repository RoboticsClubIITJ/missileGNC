# Testing

Testing was done one subsystem at a time, in this order:

1. Camera-only ArUco detection
2. Camera calibration
3. Marker pose estimation
4. Servo movement
5. Gimbal tracking
6. UDP transmission
7. Simulink integration
8. End-to-end testing

## Files

- `plant-accuracy-tests.md`: seven test cases for checking the plant and
  autopilot in Simulink (free fall, projectile, hover, vertical thrust, fixed
  deflection, sign symmetry, energy balance), each with the expected result
  and the usual cause when it fails. Converted from `Checking_accuracy.docx`,
  which is kept as the original.

## Tools for testing without the full rig

These are in `final-product/tools/`:

- `send_circle.py` sends a fake target moving in a 1 m circle, so the
  Simulink side can be tested without the camera or gimbal.
- `udp_test_listener.py` prints decoded packets on any computer, so the Pi
  side can be tested without MATLAB.
- `final-product/simulink/build_udp_monitor_model.m` builds a small model
  with scopes on every received signal.

**To do:** record the results of each test above (date, setup, what was seen)
so the next team knows which checks passed.
