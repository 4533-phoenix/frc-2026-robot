// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.util;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;

/**
 * High-performance Spark Max CAN Interceptor. Provides zero-JNI, sub-millisecond access to motor
 * telemetry via shared native memory.
 */
public class SparkTap {
  private static SparkTap instance;

  private static final int MAX_MOTORS = 64;
  private static final int STATUS_FRAMES = 7;
  private static final int SLOT_SIZE = 24; // 4 byte SeqLock + 4 byte pad + 8 data + 8 timestamp
  private static final int MOTOR_BLOCK_SIZE = SLOT_SIZE * STATUS_FRAMES;

  private final ByteBuffer buffer;
  private final MotorView[] motors = new MotorView[MAX_MOTORS];
  private static boolean loaded = false;

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

  static {
    if (RobotBase.isReal()) {
      try {
        System.loadLibrary("sparktap");
        HAL.report(tResourceType.kResourceType_Language, tInstances.kLanguage_CPlusPlus);
        loaded = true;
      } catch (UnsatisfiedLinkError e) {
        System.err.println("[SparkTap-java] Native library failed to load!");
        loaded = false;
      }
    }
  }

  /**
   * Returns the singleton instance of SparkTap.
   *
   * @return The SparkTap instance.
   */
  public static synchronized SparkTap getInstance() {
    if (instance == null) {
      instance = new SparkTap();
    }
    return instance;
  }

  private SparkTap() {
    if (loaded) {
      buffer = initNative().order(ByteOrder.LITTLE_ENDIAN);
    } else {
      buffer =
          ByteBuffer.allocateDirect(MAX_MOTORS * MOTOR_BLOCK_SIZE).order(ByteOrder.LITTLE_ENDIAN);
    }

    // Pre-allocate views for all possible CAN IDs to ensure Zero GC during operation
    for (int i = 0; i < MAX_MOTORS; i++) {
      motors[i] = new MotorView(i);
    }
  }

  /** Starts the native worker thread and returns the memory map. */
  private static native ByteBuffer initNative();

  /** Returns a dedicated, zero-allocation view into a specific motor's status data. */
  public MotorView getMotor(int deviceId) {
    return motors[deviceId];
  }

  /** Dedicated view into the shared CAN telemetry table for a specific motor. */
  public class MotorView {
    private final int motorOffset;

    private MotorView(int deviceId) {
      this.motorOffset = deviceId * MOTOR_BLOCK_SIZE;
    }

    /** Gets the raw position in Rotations from the most recent CAN frame. */
    public double getPosition() {
      int offset = motorOffset + (Frame.S2.idx * SLOT_SIZE);
      int seq1;
      int seq2 = 0;
      float pos = 0.0f;
      do {
        seq1 = buffer.getInt(offset);
        if ((seq1 & 1) != 0) continue;
        pos = buffer.getFloat(offset + 12);
        seq2 = buffer.getInt(offset);
      } while (seq1 != seq2);
      return pos;
    }

    /** Gets the raw velocity in RPM from the most recent CAN frame. */
    public double getVelocity() {
      int offset = motorOffset + (Frame.S2.idx * SLOT_SIZE);
      int seq1;
      int seq2 = 0;
      float vel = 0.0f;
      do {
        seq1 = buffer.getInt(offset);
        if ((seq1 & 1) != 0) continue;
        vel = buffer.getFloat(offset + 8);
        seq2 = buffer.getInt(offset);
      } while (seq1 != seq2);
      return vel;
    }

    /** Gets the exact FPGA microsecond timestamp of the most recent Status 2 frame. */
    public long getTimestampUs() {
      int offset = motorOffset + (Frame.S2.idx * SLOT_SIZE);
      int seq1;
      int seq2 = 0;
      long ts = 0L;
      do {
        seq1 = buffer.getInt(offset);
        if ((seq1 & 1) != 0) continue;
        ts = buffer.getLong(offset + 16);
        seq2 = buffer.getInt(offset);
      } while (seq1 != seq2);
      return ts;
    }

    /**
     * Latency Compensation. Atomically reads position, velocity, and timestamp from the same CAN
     * frame and extrapolates the position to the current FPGA microsecond.
     *
     * @return The mathematically time-aligned position in Rotations.
     */
    public double getLatencyCompensatedPosition() {
      int offset = motorOffset + (Frame.S2.idx * SLOT_SIZE);
      int seq1;
      int seq2 = 0;
      float rawPos = 0.0f, rawVelRpm = 0.0f;
      long rawTsUs = 0L;

      do {
        seq1 = buffer.getInt(offset);
        if ((seq1 & 1) != 0) continue;
        rawVelRpm = buffer.getFloat(offset + 8);
        rawPos = buffer.getFloat(offset + 12);
        rawTsUs = buffer.getLong(offset + 16);
        seq2 = buffer.getInt(offset);
      } while (seq1 != seq2);

      long currentTsUs = RobotController.getFPGATime();
      double latencySec = (currentTsUs - rawTsUs) / 1.0e6;

      // Cap extrapolation at 100ms to prevent runaway values if CAN bus drops
      if (latencySec < 0.0 || latencySec > 0.1) {
        latencySec = 0.0;
      }

      return rawPos + ((rawVelRpm / 60.0) * latencySec);
    }

    /** Gets the current sequence number for a given frame. */
    public int getSequenceNumber(Frame frame) {
      return buffer.getInt(motorOffset + (frame.idx * SLOT_SIZE));
    }
  }
}
