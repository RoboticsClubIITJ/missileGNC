"""Send a fake target moving in a circle to Simulink, without the camera.

Usage:
    python send_circle.py                 # uses SIMULINK_HOST / SIMULINK_PORT env vars
    python send_circle.py 192.168.1.50    # or give the MATLAB computer's IP

Falls back to 127.0.0.1 (same computer as MATLAB) if neither is given.
"""
import os
import socket
import struct
import sys
import time
import math

# IP of the computer running Simulink, and the receiver port
TARGET_IP = sys.argv[1] if len(sys.argv) > 1 else os.getenv("SIMULINK_HOST", "127.0.0.1")
TARGET_PORT = int(os.getenv("SIMULINK_PORT", "25000"))

# UDP setup
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
address = (TARGET_IP, TARGET_PORT)

# Circle parameters
radius = 1.0  # 1 meter radius
frequency = 0.5  # 0.5 Hz (1 circle every 2 seconds)
send_rate_hz = 20.0  # Send 20 packets per second
period_s = 1.0 / send_rate_hz

sequence = 0
start_time = time.time()

print(f"Sending circle coordinates to {TARGET_IP}:{TARGET_PORT} at {send_rate_hz}Hz...")
print("Press Ctrl+C to stop.")

try:
    while True:
        current_time = time.time()
        elapsed = current_time - start_time
        
        # Calculate x and y for a circle
        # angle = 2 * pi * f * t
        angle = 2 * math.pi * frequency * elapsed
        x = radius * math.cos(angle)
        y = radius * math.sin(angle)
        
        # Fill the remaining fields with dummy data to match Simulink expectation:
        # Expected format from servo.py: [x, y, z, pitch, yaw, distance] + sequence
        z = 0.0
        pitch = 0.0
        yaw = 0.0
        distance = radius
        
        # Pack data using little-endian, 6 doubles (d) and 1 unsigned int (I)
        # Matches `struct.pack("<6dI", ...)` in servo.py
        packet = struct.pack("<6dI", x, y, z, pitch, yaw, distance, sequence)
        
        # Send via UDP
        sock.sendto(packet, address)
        
        if sequence % 20 == 0:
            print(f"Seq: {sequence:4d} | Sent: x={x:+.3f}, y={y:+.3f}")
            
        sequence += 1
        
        # Maintain fixed send rate
        time.sleep(period_s)
        
except KeyboardInterrupt:
    print("\nStopped.")
finally:
    sock.close()
