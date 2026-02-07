// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.util;

import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.wpilibj.RobotBase;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;

public class VisionNative {
  private static VisionNative instance;
  private static final int MAX_QUEUE_SIZE = 32;
  public static final int STRUCT_SIZE = 64;

  private final ByteBuffer buffer;
  private static boolean loaded = false;

  // Cache for current packet count to avoid recalculating offsets
  private int currentPacketCount = 0;

  public static synchronized VisionNative getInstance() {
    if (instance == null) {
      instance = new VisionNative();
    }
    return instance;
  }

  static {
    if (RobotBase.isReal()) {
      try {
        System.loadLibrary("vision_server");
        loaded = true;
      } catch (UnsatisfiedLinkError e) {
        System.err.println(
            "[VisionNative-java] Failed to load vision_server library: " + e.getMessage());
        loaded = false;
      }
    }
  }

  private VisionNative() {
    buffer = ByteBuffer.allocateDirect(MAX_QUEUE_SIZE * STRUCT_SIZE);
    buffer.order(ByteOrder.nativeOrder());
  }

  private static native void startServer(int port);

  private static native void broadcastRobotHeading(double angle);

  private static native int drainPackets(ByteBuffer buf, long currentHalTime);

  public void start(int port) {
    if (!loaded) {
      System.err.println("[VisionNative-java] Cannot start server: native library not loaded.");
      return;
    }
    startServer(port);
    System.out.println("[VisionNative-java] Vision server started on port " + port);
  }

  public void broadcast(double angle) {
    if (!loaded) return;
    broadcastRobotHeading(angle);
  }

  public boolean isLoaded() {
    return loaded;
  }

  /** Drains the C queue and returns the count of observations. */
  public int readPackets() {
    if (!loaded) return 0;
    currentPacketCount = drainPackets(buffer, HALUtil.getFPGATime());
    return currentPacketCount;
  }

  // Pre-calculated byte offsets for struct fields
  private static final int OFFSET_X = 0;
  private static final int OFFSET_Y = 8;
  private static final int OFFSET_ROT = 16;
  private static final int OFFSET_STD_X = 24;
  private static final int OFFSET_STD_Y = 32;
  private static final int OFFSET_STD_ROT = 40;
  private static final int OFFSET_TIMESTAMP = 48;
  private static final int OFFSET_CAMERA_ID = 56;
  private static final int OFFSET_NUM_TAGS = 57;

  public double getX(int i) {
    return buffer.getDouble(i * STRUCT_SIZE + OFFSET_X);
  }

  public double getY(int i) {
    return buffer.getDouble(i * STRUCT_SIZE + OFFSET_Y);
  }

  public double getRot(int i) {
    return buffer.getDouble(i * STRUCT_SIZE + OFFSET_ROT);
  }

  public double getStdX(int i) {
    return buffer.getDouble(i * STRUCT_SIZE + OFFSET_STD_X);
  }

  public double getStdY(int i) {
    return buffer.getDouble(i * STRUCT_SIZE + OFFSET_STD_Y);
  }

  public double getStdRot(int i) {
    return buffer.getDouble(i * STRUCT_SIZE + OFFSET_STD_ROT);
  }

  public long getTimestamp(int i) {
    return buffer.getLong(i * STRUCT_SIZE + OFFSET_TIMESTAMP);
  }

  public int getCameraId(int i) {
    return Byte.toUnsignedInt(buffer.get(i * STRUCT_SIZE + OFFSET_CAMERA_ID));
  }

  public int getNumTags(int i) {
    return Byte.toUnsignedInt(buffer.get(i * STRUCT_SIZE + OFFSET_NUM_TAGS));
  }
}
