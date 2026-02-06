package frc.robot.util;

import edu.wpi.first.wpilibj.RobotBase;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;

public class VisionNative {
  private static final int MAX_QUEUE_SIZE = 32;
  public static final int STRUCT_SIZE = 64;

  private static final ByteBuffer buffer;
  private static boolean loaded = false;

  static {
    // Allocate direct memory (Zero Copy)
    buffer = ByteBuffer.allocateDirect(MAX_QUEUE_SIZE * STRUCT_SIZE);
    buffer.order(ByteOrder.nativeOrder());

    // 1. Only load the library if the bot is not being simulated
    if (RobotBase.isReal()) {
      try {
        System.loadLibrary("vision_server");
        loaded = true;
      } catch (UnsatisfiedLinkError e) {
        System.err.println("Failed to load vision_server library: " + e.getMessage());
        loaded = false;
      }
    } else {
      System.out.println("Simulation detected: Skipping native vision library load.");
    }
  }

  /** Starts the UDP listener thread in C. */
  private static native void startServer(int port);

  /**
   * Drains the C queue into the shared ByteBuffer.
   *
   * @param buf The DirectByteBuffer to write to.
   * @return The number of packets written.
   */
  private static native int drainPackets(ByteBuffer buf);

  /** Safe wrapper for starting the server */
  public static void start(int port) {
    if (loaded) {
      startServer(port);
    }
  }

  /** Call this in your robot periodic. Parses the raw bytes into a usable Java format. */
  public static void readPackets() {
    if (!loaded) return;

    int count = drainPackets(buffer);

    for (int i = 0; i < count; i++) {
      int offset = i * STRUCT_SIZE;

      // Read doubles directly from raw memory
      double x = buffer.getDouble(offset);
      double y = buffer.getDouble(offset + 8);
      double rot = buffer.getDouble(offset + 16);

      // Skip stds (3 doubles = 24 bytes) for now

      // Read timestamp and ID
      long ts = buffer.getLong(offset + 48);
      long camId = buffer.getLong(offset + 56);

      // TODO: Process your data here
      // System.out.printf("Cam: %d | X: %.2f Y: %.2f%n", camId, x, y);
    }
  }
}
