// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Modified by FRC Team 4533 (Phoenix) 2026
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
  private final StatusView view = new StatusView();

  public enum Frame {
    S0(0),
    S1(1),
    S2(2),
    S3(3),
    S4(4),
    S5(5),
    S6(6);
    public final int idx;

    Frame(int i) {
      this.idx = i;
    }
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
      buffer = ByteBuffer.allocate(0); // Fallback for simulation
    }
  }

  /**
   * Blocks the calling thread until the motor sends the specified status frame.
   *
   * @param deviceId CAN ID of the motor.
   * @param frame The frame type to wait for.
   */
  public void sync(int deviceId, Frame frame) {
    SparkTapJNI.waitForFrame(deviceId, frame.idx);
  }

  /**
   * Returns a view into a specific motor's status data. This does NOT allocate memory; it reuses a
   * flyweight object.
   */
  public StatusView lookup(int deviceId) {
    view.setMotor(deviceId);
    return view;
  }

  /** Flyweight view into the shared CAN telemetry table. */
  public class StatusView {
    private int motorOffset = 0;

    void setMotor(int deviceId) {
      this.motorOffset = deviceId * MOTOR_BLOCK_SIZE;
    }

    /** Returns the absolute FPGA timestamp of when a specific frame was received. */
    public long getTimestamp(Frame frame) {
      return buffer.getLong(motorOffset + (frame.idx * SLOT_SIZE) + 8);
    }

    // --- DECODERS FOR STATUS 2 (Position/Velocity) ---

    /** Decodes Position (float32) from Status 2. */
    public double getPosition() {
      int offset = motorOffset + (Frame.S2.idx * SLOT_SIZE);
      return buffer.getFloat(offset);
    }

    // --- DECODERS FOR STATUS 1 (Current/Voltage/Velocity) ---

    /** Decodes Velocity (float32) from Status 1. */
    public double getVelocity() {
      int offset = motorOffset + (Frame.S1.idx * SLOT_SIZE);
      return buffer.getFloat(offset);
    }

    /** Decodes Motor Current from Status 1 (Bytes 4-5, 12-bit). */
    public double getCurrent() {
      int offset = motorOffset + (Frame.S1.idx * SLOT_SIZE);
      // Byte 4: bits 0-7, Byte 5: bits 0-3
      int raw =
          (Byte.toUnsignedInt(buffer.get(offset + 4)))
              | ((Byte.toUnsignedInt(buffer.get(offset + 5)) & 0xF) << 8);
      return raw * 0.125; // Standard REV scaling
    }

    /** Decodes Bus Voltage from Status 1 (Bytes 5-7, 12-bit). */
    public double getBusVoltage() {
      int offset = motorOffset + (Frame.S1.idx * SLOT_SIZE);
      // Byte 5: bits 4-7, Byte 6: bits 0-7
      int raw =
          ((Byte.toUnsignedInt(buffer.get(offset + 5)) & 0xF0) >> 4)
              | (Byte.toUnsignedInt(buffer.get(offset + 6)) << 4);
      return raw * 0.125; // Standard REV scaling
    }

    /** Returns raw 8-byte data for custom decoding. */
    public long getRawData(Frame frame) {
      return buffer.getLong(motorOffset + (frame.idx * SLOT_SIZE));
    }
  }
}
