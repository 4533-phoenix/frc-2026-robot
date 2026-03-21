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

import edu.wpi.first.units.measure.*;
import frc.lib.hardware.GyroType;
import org.littletonrobotics.junction.AutoLog;

/** Interface for the gyroscope input/output abstraction. */
public interface GyroIO {
  /** Contains all of the inputs received from the gyro hardware. */
  @AutoLog
  public static class GyroIOInputs {
    /** Whether the gyro is currently connected and communicating. */
    public boolean connected = false;

    /** The current yaw position from the gyro. */
    public Angle yawPosition = Radians.zero();

    /** The current angular velocity around the yaw axis. */
    public AngularVelocity yawVelocity = RadiansPerSecond.zero();

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
  }

  /**
   * Updates the set of loggable inputs.
   *
   * @param inputs The inputs object to update.
   */
  public default void updateInputs(GyroIOInputs inputs) {}

  /** Clears all faults and warnings. */
  public default void clearFaults() {}

  /**
   * Sets whether this gyro should broadcast telemetry to Whacknet. Useful for preventing duplicate
   * broadcasts in Dual Gyro setups.
   */
  public default void setWhacknetEnabled(boolean enabled) {}

  /**
   * Sets the gyro's yaw position to the specified angle.
   *
   * @param yaw The angle to set the gyro's yaw position to.
   */
  public default void setYaw(Angle yaw) {}
}
