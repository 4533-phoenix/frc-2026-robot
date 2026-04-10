// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Modified by FRC Team 4533 (Phoenix) 2026
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive.gyro;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.measure.*;
import frc.lib.hardware.GyroType;
import frc.robot.subsystems.drive.Drive.IMUDataConsumer;
import org.littletonrobotics.junction.AutoLog;

/** Interface for the gyroscope input/output abstraction. */
public interface GyroIO {
  /** Contains all of the inputs received from the gyro hardware. */
  @AutoLog
  public static class GyroIOInputs {
    /** Whether the gyro is currently connected and communicating. */
    public boolean connected = false;

    /** Whether the gyro is ready for use. */
    public boolean locked = false;

    /** The current yaw position from the gyro. */
    public Angle yawPosition = Radians.zero();

    /** The current angular velocity around the yaw axis. */
    public AngularVelocity yawVelocity = RadiansPerSecond.zero();

    /** The current roll position from the gyro. */
    public Angle rollPosition = Radians.zero();

    /** The current pitch position from the gyro. */
    public Angle pitchPosition = Radians.zero();

    /** The current angular velocity around the roll axis. */
    public AngularVelocity rollVelocity = RadiansPerSecond.zero();

    /** The current angular velocity around the pitch axis. */
    public AngularVelocity pitchVelocity = RadiansPerSecond.zero();

    /** Whether the gyro is healthy. */
    public boolean healthy = false;

    /** The types of gyros in use. */
    public GyroType[] types = new GyroType[] {};

    /** The active faults associated with the gyro. */
    public int[] activeFaults = new int[] {};

    /** The sticky faults associated with the gyro. */
    public int[] stickyFaults = new int[] {};

    /**
     * Timestamps for high-frequency yaw measurements used for odometry.
     *
     * <p>Measured in seconds via {@link edu.wpi.first.wpilibj.RobotController#getFPGATime()}.
     */
    public double[] odometryYawTimestamps = new double[] {};

    /**
     * Yaw positions corresponding to the timestamps in {@link #odometryYawTimestamps}.
     *
     * <p>Measured in radians.
     */
    public double[] odometryYawPositions = new double[] {};

    /**
     * Pitch positions corresponding to the timestamps in {@link #odometryYawTimestamps}.
     *
     * <p>Measured in radians.
     */
    public double[] odometryPitchPositions = new double[] {};

    /**
     * Roll positions corresponding to the timestamps in {@link #odometryYawTimestamps}.
     *
     * <p>Measured in radians.
     */
    public double[] odometryRollPositions = new double[] {};
  }

  /**
   * Updates the set of loggable inputs.
   *
   * @param inputs The inputs object to update.
   */
  public default void updateInputs(GyroIOInputs inputs) {}

  /**
   * Polls high-frequency data for the 200Hz odometry loop.
   *
   * @param timestampSec The exact timestamp of the current 200Hz tick.
   * @param callback A consumer to receive the raw IMU data if valid.
   */
  public default void updateHighFreq(double timestampSec, IMUDataConsumer callback) {}

  /** Clears all faults and warnings. */
  public default void clearFaults() {}

  /**
   * Sets the gyro's rotation to the specified angle.
   *
   * @param rotation The angle to set the gyro's rotation to.
   */
  public default void setRotation(Rotation3d rotation) {}
}
