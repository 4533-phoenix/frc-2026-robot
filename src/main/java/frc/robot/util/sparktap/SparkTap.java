// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.util.sparktap;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

/**
 * High-performance Spark Max CAN Interceptor. Provides sub-millisecond access to motor telemetry
 * via shared native memory.
 */
public class SparkTap {
  private static SparkTap instance;

  private static final int MAX_MOTORS = 64;
  private static final int STATUS_FRAMES = 7;
  private static final int SLOT_SIZE = 16; // 8 bytes data + 8 bytes timestamp
  private static final int MOTOR_BLOCK_SIZE = SLOT_SIZE * STATUS_FRAMES;

  private final ByteBuffer buffer;
  private final MotorView[] motors = new MotorView[MAX_MOTORS];

  public enum Frame {
    S0(0), S1(1), S2(2), S3(3), S4(4), S5(5), S6(6);
    public final int idx;
    Frame(int i) { this.idx = i; }
  }

  public static synchronized SparkTap getInstance() {
    if (instance == null) instance = new SparkTap();
    return instance;
  }

  private SparkTap() {
    ByteBuffer raw = SparkTapJNI.init();
    if (raw != null) {
      buffer = raw.order(ByteOrder.LITTLE_ENDIAN);
    } else {
      buffer = ByteBuffer.allocate(MAX_MOTORS * MOTOR_BLOCK_SIZE).order(ByteOrder.LITTLE_ENDIAN); // Simulation fallback
    }

    // Pre-allocate thread-safe views for all possible CAN IDs
    for (int i = 0; i < MAX_MOTORS; i++) {
      motors[i] = new MotorView(i);
    }
  }

  /**
   * Blocks the calling thread until the motor sends the specified status frame.
   */
  public void sync(int deviceId, Frame frame) {
    SparkTapJNI.waitForFrame(deviceId, frame.idx);
  }

  /**
   * Returns a dedicated, thread-safe view into a specific motor's status data.
   */
  public MotorView getMotor(int deviceId) {
    return motors[deviceId];
  }

  /** Dedicated view into the shared CAN telemetry table for a specific motor. */
  public class MotorView {
    private final int motorOffset;

    private MotorView(int deviceId) {
      this.motorOffset = deviceId * MOTOR_BLOCK_SIZE;
    }

    public long getTimestamp(Frame frame) {
      return buffer.getLong(motorOffset + (frame.idx * SLOT_SIZE) + 8);
    }

    /** Decodes Position (float32) from Status 2. */
    public double getPosition() {
      return buffer.getFloat(motorOffset + (Frame.S2.idx * SLOT_SIZE));
    }

    /** Decodes Velocity (float32) from Status 1. */
    public double getVelocity() {
      return buffer.getFloat(motorOffset + (Frame.S1.idx * SLOT_SIZE));
    }

    /** Decodes Motor Current from Status 1 (Bytes 4-5, 12-bit). */
    public double getCurrent() {
      int offset = motorOffset + (Frame.S1.idx * SLOT_SIZE);
      int raw = (Byte.toUnsignedInt(buffer.get(offset + 4)))
              | ((Byte.toUnsignedInt(buffer.get(offset + 5)) & 0xF) << 8);
      return raw * 0.125;
    }

    /** Decodes Bus Voltage from Status 1 (Bytes 5-7, 12-bit). */
    public double getBusVoltage() {
      int offset = motorOffset + (Frame.S1.idx * SLOT_SIZE);
      int raw = ((Byte.toUnsignedInt(buffer.get(offset + 5)) & 0xF0) >> 4)
              | (Byte.toUnsignedInt(buffer.get(offset + 6)) << 4);
      return raw * 0.125;
    }
  }
}
