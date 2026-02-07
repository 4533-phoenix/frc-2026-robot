// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.RobotBase;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.ArrayList;
import java.util.List;

public class VisionNative {
  private static final int MAX_QUEUE_SIZE = 32;
  public static final int STRUCT_SIZE = 64;

  private static final ByteBuffer buffer;
  private static boolean loaded = false;

  // Reusable list to minimize GC pressure
  private static final List<VisionObservation> observations = new ArrayList<>(MAX_QUEUE_SIZE);

  /** Represents a single vision measurement with WPILib helper methods. */
  public record VisionObservation(
      double x,
      double y,
      double rotRadians,
      double stdX,
      double stdY,
      double stdRot,
      long timestampMicros,
      long cameraId) {

    /** Converts raw coordinates to a WPILib Pose2d. */
    public Pose2d getPose() {
      return new Pose2d(x, y, new Rotation2d(rotRadians));
    }

    /** Converts raw standard deviations to a WPILib Matrix. */
    public Matrix<N3, N1> getStdDevs() {
      return VecBuilder.fill(stdX, stdY, stdRot);
    }

    /** Converts microsecond timestamp to seconds (FPGA time). */
    public double getTimestampSeconds() {
      return timestampMicros / 1.0e6;
    }
  }

  static {
    buffer = ByteBuffer.allocateDirect(MAX_QUEUE_SIZE * STRUCT_SIZE);
    buffer.order(ByteOrder.nativeOrder());

    if (RobotBase.isReal()) {
      try {
        System.loadLibrary("vision_server");
        loaded = true;
      } catch (UnsatisfiedLinkError e) {
        System.err.println(
            "[VisionNative] Failed to load vision_server library: " + e.getMessage());
        loaded = false;
      }
    }
  }

  private static native void startServer(int port);

  private static native int drainPackets(ByteBuffer buf);

  public static void start(int port) {
    if (loaded) {
      startServer(port);
      System.out.println("[VisionNative] Vision server started on port " + port);
    } else {
      System.err.println("[VisionNative] Cannot start server: native library not loaded.");
    }
  }

  /** Drains the C queue and returns the latest observations. */
  public static List<VisionObservation> readPackets() {
    observations.clear();
    if (!loaded) return observations;

    int count = drainPackets(buffer);

    for (int i = 0; i < count; i++) {
      int offset = i * STRUCT_SIZE;

      observations.add(
          new VisionObservation(
              buffer.getDouble(offset), // x
              buffer.getDouble(offset + 8), // y
              buffer.getDouble(offset + 16), // rot (radians)
              buffer.getDouble(offset + 24), // stdX
              buffer.getDouble(offset + 32), // stdY
              buffer.getDouble(offset + 40), // stdRot
              buffer.getLong(offset + 48), // ts
              buffer.getLong(offset + 56) // camId
              ));
    }
    return observations;
  }
}
