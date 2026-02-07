import socket
import struct

# Configuration
UDP_IP = "0.0.0.0" # Listen on all interfaces
UDP_PORT = 7002

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

print(f"Listening for UDP broadcast on {UDP_IP}:{UDP_PORT}...")

while True:
    data, addr = sock.recvfrom(1024) # buffer size is 1024 bytes
    
    if len(data) == 8:
        # Unpack double (float64)
        heading_rad = struct.unpack('d', data)[0]
        heading_deg = heading_rad * (180.0 / 3.141592653589793)
        print(f"[{addr[0]}] Heading: {heading_rad:.4f} rad ({heading_deg:.2f} deg)")
    else:
        print(f"Received packet of unexpected size: {len(data)} bytes from {addr}")
