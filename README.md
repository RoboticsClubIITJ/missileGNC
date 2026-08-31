# Not Rocket Science: Physical Visual Servoing based Missile Guidance Simulator
The primary goal of the project is to build a low-cost Seeker-in-the-Loop simulation
platform that integrates a real vision system with a simulated guidance environment.
The project will begin with camera interfacing and target detection using ArUco
markers. Then implement visual servoing algorithms to maintain the target near the
center of the camera frame using a pan-tilt gimbal.
The resulting image measurements
will be converted into LOS information and transmitted to Simulink through UDP com-
munication.
Within Simulink, a virtual interceptor model will be developed along with classical
guidance algorithms such as Pure Pursuit, LOS Guidance, and Proportional Navigation.
