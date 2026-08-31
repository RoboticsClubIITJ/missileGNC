# ai kam use kiya kar akshat 

# simulink and hardware integration
### Simulink network settings

```python
SIMULINK_HOST = "192.168.1.50"
SIMULINK_PORT = 25000
```

Set `SIMULINK_HOST` to the IP address of the computer running MATLAB. The MATLAB receiver listens on port `25000` by default. Both computers must be on the same network, and the MATLAB computer firewall must allow incoming UDP traffic on that port.

## UDP Packet Format

For every valid marker detection, `servo.py` sends one UDP datagram to `SIMULINK_HOST:SIMULINK_PORT`.

The payload is:

```text
[x, y, z, pitch, yaw, distance, sequence_number]
```

Binary format:

```python
struct.pack("<6dI", *marker_array, sequence_number)
```

This is:

- six little-endian 64-bit floating-point values: 48 bytes
- one little-endian 32-bit unsigned sequence number: 4 bytes
- total packet size: 52 bytes

The sequence number increments for each packet and wraps after `0xFFFFFFFF`.

