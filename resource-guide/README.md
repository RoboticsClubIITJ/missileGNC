# Resource guide

Reading for anyone joining the project, in roughly the order the topics
come up. Add to it whenever something helps you.

## Vision: ArUco markers and pose

- OpenCV, *Detection of ArUco Markers*: dictionaries, detector parameters,
  drawing results.
  https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html
- OpenCV, *Camera calibration with ArUco and ChArUco boards*: the method used by
  `calibrate_camera_picamera2.py`.
  https://docs.opencv.org/4.x/da/d13/tutorial_aruco_calibration.html
- OpenCV, *Perspective-n-Point (PnP) pose computation*: what `solvePnP`
  solves and the solver flags.
  https://docs.opencv.org/4.x/d5/d1f/calib3d_solvePnP.html

## Raspberry Pi hardware

- *The Picamera2 Library* manual (Raspberry Pi Ltd).
  https://datasheets.raspberrypi.com/camera/picamera2-manual.pdf
- pigpio library, used for servo PWM through the `pigpiod` daemon.
  https://abyz.me.uk/rpi/pigpio/

## MATLAB and Simulink

- MATLAB System block: how a `matlab.System` class such as
  `GimbalUDPReceiver` runs inside a model.
  https://www.mathworks.com/help/simulink/slref/matlabsystem.html
- *Proportional Navigation*: the guidance law in V12
  (`a_cmd = N · Vm · λ̇` plus a gravity term, N = 4).
  Paul Zarchan, *Tactical and Strategic Missile Guidance*, AIAA (the standard textbook on PN).

## Inside this repo

- `final-product/known-issues.md`: what is known to be wrong or unverified.
- `learning/iteration-phase/reviews/`: two long code reviews of the Sep 2026
  code. Useful for understanding how the pipeline fits together, but check
  any finding against the current code before acting on it.
- `learning/iteration-phase/testing/plant-accuracy-tests.md`: physics checks
  for the plant.
