# Tools

Helpers for testing one half of the system without the other.

| Tool | Runs on | What it does |
|---|---|---|
| `send_circle.py` | Any computer | Sends a fake target moving in a 1 m circle at 20 packets/s, in the same 52-byte format as `servo.py`. Use it to test Simulink without the camera or gimbal. |
| `udp_test_listener.py` | Any computer | Listens on UDP port 25000 and prints every decoded packet. Use it to test the Pi side without MATLAB. |
| `aruco_marker_stage.html` | Browser | Page for placing ArUco markers on a printable stage. Not used by the runtime. |

Quick test of the whole link with no hardware, on one computer:

```bash
python udp_test_listener.py          # terminal 1
python send_circle.py 127.0.0.1      # terminal 2
```

Only one program can listen on port 25000 at a time, so close the listener
before starting the Simulink model, and the other way round.
