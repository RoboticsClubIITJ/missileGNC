"""Print pose packets sent by servo.py, without MATLAB.

Run this on the computer that SIMULINK_HOST points to (or on the Pi itself
with SIMULINK_HOST=127.0.0.1) to check that packets arrive and decode.

Packet: six little-endian doubles + one uint32 sequence number (52 bytes).
Slot 4 is labelled bearing_pitch in GimbalUDPReceiver.m, but the current
servo.py puts the distance there. See final-product/known-issues.md.
"""
import socket
import struct

PORT = 25000  # must match SIMULINK_PORT
PACKET_SIZE = struct.calcsize("<6dI")  # 52

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", PORT))

print(f"Listening on UDP port {PORT}... (Ctrl+C to stop)")
try:
    while True:
        data, addr = sock.recvfrom(1024)
        if len(data) != PACKET_SIZE:
            print(f"Unexpected size {len(data)} bytes from {addr}")
            continue
        x, y, z, slot4, bearing_yaw, distance, seq = struct.unpack("<6dI", data)
        print(f"seq={seq:>10} x={x:+.3f} y={y:+.3f} z={z:+.3f} "
              f"slot4={slot4:+.3f} bearing_yaw={bearing_yaw:+.2f} dist={distance:.3f}")
except KeyboardInterrupt:
    print("\nStopped.")
finally:
    sock.close()
