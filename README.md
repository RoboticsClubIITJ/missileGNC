# Not Rocket Science: Physical Visual Servoing based Missile Guidance Simulator

A Robotics Club IITJ learning project. The goal is a low-cost
seeker-in-the-loop simulation platform that connects a real vision system to
a simulated guidance environment.

A Raspberry Pi camera on a pan/tilt gimbal detects an ArUco target and keeps
it near the centre of the frame (visual servoing). The measured target
position is sent over UDP to Simulink. There, a virtual interceptor is
guided towards it with classical guidance laws. The current model uses
proportional navigation.

```text
Target → Camera → ArUco detection → Pose estimation → Gimbal control
                                                          ↓
                 MATLAB plotting ← Guidance ← Simulink ← UDP
                                         ↓
                                    Autopilot → Plant → Feedback
```

## Repository layout

```text
missileGNC/
├── final-product/          the working system
│   ├── raspberry-pi/       camera, ArUco detection, gimbal control, UDP sender
│   ├── simulink/           guidance model (V12), UDP receiver, plotting
│   ├── tools/              test without hardware: fake target, packet listener
│   └── known-issues.md     open problems and risks
├── learning/
│   └── iteration-phase/    how we got here: experiments, old versions, tests
│       ├── software/       numbered 01–06 in the order the project developed
│       ├── hardware/       components, wiring, power-up order
│       ├── testing/        test order and plant accuracy tests
│       └── reviews/        code reviews of the Sep 2026 code
└── resource-guide/         reading list for new members
```

## Where to start

- **To run the system:** [`final-product/README.md`](final-product/README.md)
  gives the startup order, then the Raspberry Pi and Simulink guides.
- **To test without hardware:** [`final-product/tools/README.md`](final-product/tools/README.md).
- **To understand the project's history:** [`learning/iteration-phase/README.md`](learning/iteration-phase/README.md).
- **Before changing anything:** read [`final-product/known-issues.md`](final-product/known-issues.md).

## Requirements

- Raspberry Pi with a Pi Camera, two servos and an external servo supply
- Python 3 with OpenCV (contrib), NumPy, Picamera2 and pigpio on the Pi
- MATLAB and Simulink R2025a on the host computer

## Contributing

Work on a branch and open a pull request into `main`. When an approach is
replaced, move the old version into `learning/iteration-phase/` with a short
README on what was tried and what happened, rather than deleting it.
Generated MATLAB files (`slprj/`, `.slxc`, autosaves) are ignored by git; do
not force-add them.
