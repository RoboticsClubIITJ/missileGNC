# Hardware

The physical half of the seeker-in-the-loop setup: a camera on a pan/tilt
gimbal, driven by a Raspberry Pi, that keeps an ArUco target in view and
streams its position to a computer running MATLAB/Simulink.

## Components

| Part | Used for | Notes |
|---|---|---|
| Raspberry Pi | Camera capture, ArUco detection, servo control, UDP sender | Runs `final-product/raspberry-pi/servo.py` |
| Pi Camera (Picamera2 / libcamera) | Seeker image | Captured at 640 × 480, RGB888 |
| Pan servo | Yaw axis of the gimbal | BCM GPIO 18 |
| Tilt servo | Pitch axis of the gimbal | BCM GPIO 19 |
| Gimbal mechanism | Holds the camera rigidly on the two servo axes | Camera movement must follow servo movement exactly |
| External servo supply | Powers the servos | Ground shared with the Pi |
| Host computer | MATLAB, Simulink, Instrument Control Toolbox | Same network as the Pi |
| Printed ArUco markers | ID 0 = origin, any other ID = target | Dictionary `DICT_6X6_250`, target edge 73.8 mm in the final config |
| Printed ChArUco board | Camera calibration | Must match the board defined in `calibrate_camera_picamera2.py` |

The GPIO pins, pulse widths and limits are set in
`final-product/raspberry-pi/gimbal_config.py`. The current values are a
1500 µs centre pulse, 8 µs per degree, pan limits ±60° and tilt limits ±45°.

## Wiring and power

- Power the servos from their own supply. The Pi's 5 V rail cannot supply
  servo stall current, and a brown-out resets the Pi mid-run.
- Connect the servo supply ground to a Pi ground pin. Without a common ground
  the PWM signal has no reference and the servos jitter or do not move.
- The GPIO numbers in the config are BCM numbers, not physical pin numbers.
- PWM comes from the `pigpio` daemon, so `pigpiod` must be running.

## First power-up

1. Test the camera and ArUco detection with the servos unplugged.
2. Test the UDP link to MATLAB, still with the servos unplugged.
3. Plug in the servos with the gimbal off its mount, or with the arms free to
   move, and check each axis turns the right way.
4. Only then mount the full mechanism.

Keep a way to cut servo power quickly during the first tests. A wrong GPIO
number or a sign error can drive a servo hard into its end stop.

## What changed during the project

The early per-member ArUco tests in `software/01-aruco-detection/` used a
laptop or USB webcam (`cv2.VideoCapture`). The project moved to the Pi Camera
with Picamera2 and a ChArUco calibration once the camera had to ride on the
gimbal.

**To do:** add photos of the gimbal, a wiring diagram and the exact servo
and camera models used.
