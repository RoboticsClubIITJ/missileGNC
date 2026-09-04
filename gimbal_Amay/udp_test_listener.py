import socket, struct

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", 25000))  # match SIMULINK_PORT

print("Listening...")
while True:
    data, addr = sock.recvfrom(1024)
    if len(data) != 52:
        print(f"Unexpected size {len(data)} bytes from {addr}")
        continue
    x, y, z, pitch, yaw, distance, seq = struct.unpack("<6dI", data)
    print(f"seq={seq:>10} x={x:+.3f} y={y:+.3f} z={z:+.3f} "
          f"pitch={pitch:+.2f} yaw={yaw:+.2f} dist={distance:.3f}")
