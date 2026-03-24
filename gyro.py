import socket
import struct
from dataclasses import dataclass
from typing import Generator, Optional

@dataclass(frozen=True)
class GyroPacket:
    """
    Decoded representation of the C GyroPacket struct.
    Size: 56 bytes (8-byte uint64 + 6 * 8-byte doubles)
    """
    fpga_timestamp: int
    roll: float
    pitch: float
    yaw: float
    roll_velocity: float
    pitch_velocity: float
    yaw_velocity: float

class WhacknetReceiver:
    # < = Little Endian, Q = uint64, d = double
    _STRUCT_FORMAT = "<Qdddddd"
    _EXPECTED_SIZE = struct.calcsize(_STRUCT_FORMAT)

    def __init__(self, port: int = 7002):
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        # SO_REUSEADDR allows restarting the script without "Address already in use" errors
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        
        try:
            self.sock.bind(('', self.port))
            print(f"[Whacknet-Py] Listening on port {self.port}")
        except PermissionError:
            print(f"[Whacknet-Py] Error: Permission denied binding to {self.port}. Try sudo or a different port.")
        except Exception as e:
            print(f"[Whacknet-Py] Bind error: {e}")

    def stream(self, timeout: float = 0.1) -> Generator[GyroPacket, None, None]:
        """
        A generator that yields packets as they arrive.
        """
        self.sock.settimeout(timeout)
        while True:
            try:
                data, _ = self.sock.recvfrom(1024)
                if len(data) == self._EXPECTED_SIZE:
                    yield GyroPacket(*struct.unpack(self._STRUCT_FORMAT, data))
            except socket.timeout:
                continue
            except KeyboardInterrupt:
                break

    def close(self):
        self.sock.close()

# --- Example Usage ---
if __name__ == "__main__":
    receiver = WhacknetReceiver()
    
    print("Starting Telemetry Stream (Ctrl+C to stop)...")
    try:
        # The generator approach makes the main loop very clean
        for packet in receiver.stream():
          #  print all values
          print(f"[{packet.fpga_timestamp}], Roll: {packet.roll:.2f}, Pitch: {packet.pitch:.2f}, Yaw: {packet.yaw:.2f}, "
                f"Roll Vel: {packet.roll_velocity:.2f}, Pitch Vel: {packet.pitch_velocity:.2f}, Yaw Vel: {packet.yaw_velocity:.2f}", end="\r")
    except KeyboardInterrupt:
        print("\nShutting down.")
    finally:
        receiver.close()