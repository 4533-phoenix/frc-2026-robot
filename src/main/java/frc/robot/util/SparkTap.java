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
import java.lang.invoke.MethodHandles;
import java.lang.invoke.VarHandle;
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
  private static final int SLOT_SIZE = 24;
  private static final int METADATA_SIZE = 24;
  private static final int MOTOR_BLOCK_SIZE = (SLOT_SIZE * STATUS_FRAMES) + METADATA_SIZE;
  private static final int METADATA_OFFSET = STATUS_FRAMES * SLOT_SIZE;

  private static final VarHandle INT_VH =
      MethodHandles.byteBufferViewVarHandle(int[].class, ByteOrder.nativeOrder());
  private static final VarHandle LONG_VH =
      MethodHandles.byteBufferViewVarHandle(long[].class, ByteOrder.nativeOrder());

  private final ByteBuffer buffer;
  private final MotorView[] motors = new MotorView[MAX_MOTORS];
  private static boolean loaded = false;

  /** Represents the different CAN status frames sent by the Spark Max. */
  public enum Frame {
    /** Status 0 frame (Bus, Power, and Limits). */
    S0(0),
    /** Status 1 frame (Diagnostics and Status). */
    S1(1),
    /** Status 2 frame (Primary Encoder). */
    S2(2),
    /** Status 3 frame (Analog Sensor). */
    S3(3),
    /** Status 4 frame (Alternate Encoder). */
    S4(4),
    /** Status 5 frame (Duty Cycle Absolute Encoder). */
    S5(5),
    /** Status 6 frame (Duty Cycle Absolute Encoder Velocity). */
    S6(6);

    /** The internal index of the frame in the memory map. */
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

    for (int i = 0; i < MAX_MOTORS; i++) {
      motors[i] = new MotorView(i);
    }
  }

  /**
   * Starts the native worker thread and returns the memory map.
   *
   * @return A ByteBuffer mapping the shared native memory.
   */
  private static native ByteBuffer initNative();

  /**
   * Returns a dedicated, zero-allocation view into a specific motor's status data.
   *
   * @param deviceId The CAN ID of the motor to access (0-63).
   * @return A MotorView providing access to the specified motor's telemetry.
   * @throws IllegalArgumentException if the deviceId is out of range.
   */
  public MotorView getMotor(int deviceId) {
    if (deviceId < 0 || deviceId >= MAX_MOTORS) {
      throw new IllegalArgumentException("Device ID out of range");
    }
    return motors[deviceId];
  }

  /** Dedicated view into the shared CAN telemetry table for a specific motor. */
  public class MotorView {
    private final int motorOffset;
    private final long[] cacheData = new long[8];
    private final long[] cacheTs = new long[8];
    private double lastLatCompPos = 0.0;

    private MotorView(int deviceId) {
      this.motorOffset = deviceId * MOTOR_BLOCK_SIZE;
    }

    private int getIntAcquire(int offset) {
      return (int) INT_VH.getAcquire(buffer, offset);
    }

    private long getLongAcquire(int offset) {
      return (long) LONG_VH.getAcquire(buffer, offset);
    }

    private long readDataAtomic(Frame frame) {
      int offset = motorOffset + (frame.idx * SLOT_SIZE);
      int seq1 = getIntAcquire(offset);
      if ((seq1 & 1) == 0) {
        long data = getLongAcquire(offset + 8);
        if (seq1 == getIntAcquire(offset)) {
          cacheData[frame.idx] = data;
        }
      }
      return cacheData[frame.idx];
    }

    /**
     * 1 Gets the exact FPGA microsecond timestamp of the most recent CAN frame.
     *
     * @param frame The status frame to get the timestamp for.
     * @return The FPGA timestamp in microseconds.
     */
    public long getTimestampUs(Frame frame) {
      int offset = motorOffset + (frame.idx * SLOT_SIZE);
      int seq1 = getIntAcquire(offset);
      if ((seq1 & 1) == 0) {
        long ts = getLongAcquire(offset + 16);
        if (seq1 == getIntAcquire(offset)) {
          cacheTs[frame.idx] = ts;
        }
      }
      return cacheTs[frame.idx];
    }

    /**
     * Gets the timestamp of the most recent CAN frame 0 in microseconds, or 0 if no frame has been
     * received.
     *
     * @return The FPGA timestamp of the most recently seen frame in microseconds.
     */
    public long getLastSeenTimestampUs() {
      int offset = motorOffset + METADATA_OFFSET;
      int seq1 = getIntAcquire(offset);
      if ((seq1 & 1) == 0) {
        long ts = getLongAcquire(offset + 16);
        if (seq1 == getIntAcquire(offset)) {
          cacheTs[7] = ts;
        }
      }
      return cacheTs[7];
    }

    /**
     * Gets the current sequence number for a given frame.
     *
     * @param frame The status frame to check.
     * @return The sequence number.
     */
    public int getSequenceNumber(Frame frame) {
      return getIntAcquire(motorOffset + (frame.idx * SLOT_SIZE));
    }

    /**
     * Checks if the motor has sent any CAN frames within the specified timeout.
     *
     * @param timeoutSeconds Time to wait before considering the device disconnected.
     * @return True if a frame has been received within the timeout, false otherwise.
     */
    public boolean isConnected(double timeoutSeconds) {
      long lastSeen = getLastSeenTimestampUs();
      if (lastSeen == 0) return false;

      long delta = RobotController.getFPGATime() - lastSeen;
      return delta >= 0 && delta < (timeoutSeconds * 1.0e6);
    }

    /**
     * Checks if the motor is connected (defaults to 100ms timeout).
     *
     * @return True if connected, false otherwise.
     */
    public boolean isConnected() {
      return isConnected(0.1);
    }

    // Status 0 (Bus, Power, and Limits)

    /**
     * Checks if the forward hard limit switch is reached. Found in Status 0.
     *
     * @return True if the limit switch is reached, false otherwise.
     */
    public boolean getForwardLimit() {
      return (readDataAtomic(Frame.S0) & (1L << 48)) != 0;
    }

    /**
     * Checks if the reverse hard limit switch is reached. Found in Status 0.
     *
     * @return True if the limit switch is reached, false otherwise.
     */
    public boolean getReverseLimit() {
      return (readDataAtomic(Frame.S0) & (1L << 49)) != 0;
    }

    /**
     * Checks if the forward soft limit is reached. Found in Status 0.
     *
     * @return True if the soft limit is reached, false otherwise.
     */
    public boolean getForwardSoftLimit() {
      return (readDataAtomic(Frame.S0) & (1L << 50)) != 0;
    }

    /**
     * Checks if the reverse soft limit is reached. Found in Status 0.
     *
     * @return True if the soft limit is reached, false otherwise.
     */
    public boolean getReverseSoftLimit() {
      return (readDataAtomic(Frame.S0) & (1L << 51)) != 0;
    }

    /**
     * Checks if the motor is currently configured as inverted. Found in Status 0.
     *
     * @return True if the motor is inverted, false otherwise.
     */
    public boolean getInverted() {
      return (readDataAtomic(Frame.S0) & (1L << 52)) != 0;
    }

    /**
     * Gets the Applied Output (Duty Cycle) from -1.0 to 1.0. Found in Status 0.
     *
     * @return The applied output from -1.0 to 1.0.
     */
    public double getAppliedOutput() {
      return ((short) readDataAtomic(Frame.S0)) * 0.00003082369457075716;
    }

    /**
     * Gets the Bus Voltage (Input Voltage) in Volts. Found in Status 0.
     *
     * @return The bus voltage in Volts.
     */
    public double getBusVoltage() {
      return ((readDataAtomic(Frame.S0) >> 16) & 0xFFF) * 0.0073260073260073;
    }

    /**
     * Gets the Output Current (Stator Current) in Amps. Found in Status 0.
     *
     * @return The output current in Amps.
     */
    public double getOutputCurrent() {
      return ((readDataAtomic(Frame.S0) >> 28) & 0xFFF) * 0.0366300366300366;
    }

    /**
     * Gets the Motor Temperature in Celsius from Status 0.
     *
     * @return The motor temperature in degrees Celsius.
     */
    public int getMotorTemperature() {
      return (int) ((readDataAtomic(Frame.S0) >> 40) & 0xFF);
    }

    /**
     * Calculates the estimated Input (Supply) Current in Amps. This is derived from Applied Output
     * * Output Current.
     *
     * @return The calculated input current in Amps.
     */
    public double getInputCurrent() {
      long data = readDataAtomic(Frame.S0);
      double applied = ((short) data) * 0.00003082369457075716;
      double outputCurrent = ((data >> 28) & 0xFFF) * 0.0366300366300366;
      return Math.abs(outputCurrent * applied);
    }

    // Status 1 (Diagnostics and Status)

    /**
     * Gets the full 64-bit fault and warning bitfield. Found in Status 1.
     *
     * @return The fault and warning bitfield.
     */
    public long getFaults() {
      return readDataAtomic(Frame.S1);
    }

    /**
     * Gets the 8-bit bitfield of currently active faults. Found in Status 1.
     *
     * @return The active faults bitfield.
     */
    public int getActiveFaults() {
      return (int) (readDataAtomic(Frame.S1) & 0xFF);
    }

    /**
     * Gets the 8-bit bitfield of sticky faults (faults that occurred since boot). Found in Status
     * 1.
     *
     * @return The sticky faults bitfield.
     */
    public int getStickyFaults() {
      return (int) ((readDataAtomic(Frame.S1) >> 24) & 0xFF);
    }

    /**
     * Gets the 8-bit bitfield of currently active warnings. Found in Status 1.
     *
     * @return The active warnings bitfield.
     */
    public int getActiveWarnings() {
      return (int) ((readDataAtomic(Frame.S1) >> 16) & 0xFF);
    }

    /**
     * Gets the 8-bit bitfield of sticky warnings. Found in Status 1.
     *
     * @return The sticky warnings bitfield.
     */
    public int getStickyWarnings() {
      return (int) ((readDataAtomic(Frame.S1) >> 40) & 0xFF);
    }

    /**
     * Checks if the motor is currently in Follower Mode. Found in Status 1.
     *
     * @return True if the motor is in follower mode, false otherwise.
     */
    public boolean isFollower() {
      return (readDataAtomic(Frame.S1) & (1L << 48)) != 0;
    }

    /**
     * Checks if the device has reset since the last time faults were cleared. Found in Status 1.
     *
     * @return True if a reset has occurred, false otherwise.
     */
    public boolean hasResetOccurred() {
      return (readDataAtomic(Frame.S1) & (1L << 22)) != 0;
    }

    // Status 2 (Primary Encoder)

    /**
     * Gets the raw position in Rotations from the most recent CAN frame.
     *
     * @return The position in Rotations.
     */
    public double getPosition() {
      long data = readDataAtomic(Frame.S2);
      return Float.intBitsToFloat((int) (data >> 32));
    }

    /**
     * Gets the raw velocity in RPM from the most recent CAN frame.
     *
     * @return The velocity in RPM.
     */
    public double getVelocity() {
      long data = readDataAtomic(Frame.S2);
      return Float.intBitsToFloat((int) data);
    }

    /**
     * Latency Compensation. Atomically reads position, velocity, and timestamp from the same CAN
     * frame and extrapolates the position to the current FPGA microsecond.
     *
     * @return The mathematically time-aligned position in Rotations.
     */
    public double getLatencyCompensatedPosition() {
      int offset = motorOffset + (Frame.S2.idx * SLOT_SIZE);
      int seq1 = getIntAcquire(offset);
      if ((seq1 & 1) == 0) {
        long data = getLongAcquire(offset + 8);
        long ts = getLongAcquire(offset + 16);
        if (seq1 == getIntAcquire(offset)) {
          double pos = Float.intBitsToFloat((int) (data >> 32));
          double vel = Float.intBitsToFloat((int) data);
          double dt = Math.min(Math.max(0, (RobotController.getFPGATime() - ts) / 1.0e6), 0.1);
          lastLatCompPos = pos + ((vel / 60.0) * dt);
        }
      }
      return lastLatCompPos;
    }

    // Status 5 (Duty Cycle Absolute Encoder)

    /** Gets the absolute encoder velocity. Found in Status 5. */
    public double getAbsoluteEncoderVelocity() {
      long data = readDataAtomic(Frame.S5);
      return Float.intBitsToFloat((int) data);
    }

    /** Gets the absolute encoder position. Found in Status 5. */
    public double getAbsoluteEncoderPosition() {
      long data = readDataAtomic(Frame.S5);
      return Float.intBitsToFloat((int) (data >> 32));
    }
  }
}
