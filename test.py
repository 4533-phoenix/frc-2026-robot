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
        # 'd' is for double float. 
        # Using native byte order defaults. If you see weird numbers, try '<d' (little-endian) or '>d' (big-endian)
        heading = struct.unpack('d', data)[0]
        print(f"Received heading from {addr}: {heading}")
    else:
        print(f"Received packet of unexpected size: {len(data)} bytes from {addr}")
