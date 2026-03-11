// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.util;

import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.function.ObjIntConsumer;

/**
 * JNI wrapper for the custom 'whacknet' native library.
 *
 * <p>Handles communication with a high-performance C++ vision pipeline. Uses a direct {@link
 * ByteBuffer} to share memory efficiently between Java and native code, avoiding unnecessary data
 * copying.
 */
public class Whacknet {
  private static Whacknet instance;

  /** Maximum number of vision packets queued in native memory. */
  private static final int MAX_QUEUE_SIZE = 64;

  /** The size in bytes of the C struct representing a vision observation. */
  public static final int STRUCT_SIZE = 64;

  /** Shared memory buffer between Java and Native. */
  private final ByteBuffer buffer;

  private static boolean loaded = false;

  // Cache for current packet count to avoid recalculating offsets
  private int currentPacketCount = 0;

  // Single pre-allocated instance to avoid Garbage Collection
  private final PacketView packetView = new PacketView();

  /**
   * Returns the singleton instance of Whacknet.
   *
   * @return The Whacknet instance.
   */
  public static synchronized Whacknet getInstance() {
    if (instance == null) {
      instance = new Whacknet();
    }
    return instance;
  }

  static {
    // Only attempt to load the native library on a real robot
    if (RobotBase.isReal()) {
      try {
        System.loadLibrary("whacknet");
        loaded = true;
      } catch (UnsatisfiedLinkError e) {
        System.err.println("[Whacknet-java] Failed to load whacknet library: " + e.getMessage());
        loaded = false;
      }
    }
  }

  /** Initializes the direct byte buffer for shared memory. */
  private Whacknet() {
    // Allocate direct buffer to be accessible by JNI
    buffer = ByteBuffer.allocateDirect(MAX_QUEUE_SIZE * STRUCT_SIZE);
    buffer.order(ByteOrder.nativeOrder());
  }

  // Native Methods

  /**
   * Starts the native vision server thread.
   *
   * @param port The port to bind the server to.
   */
  private static native void startServer(int port);

  /**
   * Broadcasts the current robot heading to the vision pipeline for pose estimation.
   *
   * @param angle Robot angle in radians.
   */
  private static native void broadcastRobotHeading(double angle);

  /**
   * Transfers packets from the native queue into the shared ByteBuffer.
   *
   * @param buf The direct ByteBuffer to write data into.
   * @param currentHalTime The current FPGA time for timestamping.
   * @return The number of packets copied into the buffer.
   */
  private static native int drainPackets(ByteBuffer buf, long currentHalTime);

  // Java Methods

  /**
   * Starts the native vision server.
   *
   * @param port The port to start the server on.
   */
  public void start(int port) {
    if (!loaded) {
      System.err.println("[Whacknet-java] Cannot start server: native library not loaded.");
      return;
    }
    startServer(port);
    System.out.println("[Whacknet-java] Vision server started on port " + port);
  }

  /**
   * Broadcasts the robot heading to the native library.
   *
   * @param angle Robot angle in radians.
   */
  public void broadcast(double angle) {
    if (!loaded) return;
    broadcastRobotHeading(angle);
  }

  /**
   * Checks if the native library is loaded.
   *
   * @return True if library is loaded, false otherwise.
   */
  public boolean isLoaded() {
    return loaded;
  }

  /**
   * Drains the C queue and returns the count of observations. Must be called periodically to get
   * the latest data.
   *
   * @return The number of packets available in the buffer.
   */
  public int readPackets() {
    if (!loaded) return 0;
    currentPacketCount = drainPackets(buffer, HALUtil.getFPGATime());
    return currentPacketCount;
  }

  /**
   * Extremely lightweight way to iterate over packets. Uses the Flyweight pattern with a single
   * reused object to prevent memory allocation (Zero GC overhead).
   *
   * @param consumer A lambda to process each vision observation.
   */
  public void forEachPacket(ObjIntConsumer<PacketView> consumer) {
    for (int i = 0; i < currentPacketCount; i++) {
      packetView.setIndex(i);
      consumer.accept(packetView, i);
    }
  }

  // Struct Field Offset Mapping
  // These offsets correspond directly to the layout of the C struct.

  private static final int OFFSET_X = 0;
  private static final int OFFSET_Y = 8;
  private static final int OFFSET_ROT = 16;
  private static final int OFFSET_STD_X = 24;
  private static final int OFFSET_STD_Y = 32;
  private static final int OFFSET_STD_ROT = 40;
  private static final int OFFSET_TIMESTAMP = 48;
  private static final int OFFSET_CAMERA_ID = 56;
  private static final int OFFSET_NUM_TAGS = 57;

  /**
   * A zero-allocation view into the ByteBuffer representing a single Vision Observation.
   * Pre-calculates array offsets to save CPU cycles.
   */
  public class PacketView {
    private int baseOffset = 0;

    // Package-private so only Whacknet can move the cursor
    void setIndex(int index) {
      this.baseOffset = index * STRUCT_SIZE;
    }

    /**
     * @return X position in meters.
     */
    public double getX() {
      return buffer.getDouble(baseOffset + OFFSET_X);
    }

    /**
     * @return Y position in meters.
     */
    public double getY() {
      return buffer.getDouble(baseOffset + OFFSET_Y);
    }

    /**
     * @return Rotation in radians.
     */
    public double getRot() {
      return buffer.getDouble(baseOffset + OFFSET_ROT);
    }

    /**
     * @return Standard deviation of X in meters.
     */
    public double getStdX() {
      return buffer.getDouble(baseOffset + OFFSET_STD_X);
    }

    /**
     * @return Standard deviation of Y in meters.
     */
    public double getStdY() {
      return buffer.getDouble(baseOffset + OFFSET_STD_Y);
    }

    /**
     * @return Standard deviation of rotation in radians.
     */
    public double getStdRot() {
      return buffer.getDouble(baseOffset + OFFSET_STD_ROT);
    }

    /**
     * @return FPGA Timestamp in microseconds.
     */
    public long getTimestamp() {
      return buffer.getLong(baseOffset + OFFSET_TIMESTAMP);
    }

    /**
     * @return Camera ID that made the observation.
     */
    public int getCameraId() {
      return Byte.toUnsignedInt(buffer.get(baseOffset + OFFSET_CAMERA_ID));
    }

    /**
     * @return Number of AprilTags detected in this observation.
     */
    public int getNumTags() {
      return Byte.toUnsignedInt(buffer.get(baseOffset + OFFSET_NUM_TAGS));
    }

    // WPILib Methods

    /**
     * Easily converts the raw data into a WPILib Pose2d.
     *
     * @return The Pose2d object representing the robot's estimated field position.
     */
    public Pose2d getPose2d() {
      return new Pose2d(getX(), getY(), new Rotation2d(getRot()));
    }
  }
}
