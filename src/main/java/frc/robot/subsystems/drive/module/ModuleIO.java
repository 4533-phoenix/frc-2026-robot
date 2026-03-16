// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Modified by FRC Team 4533 (Phoenix) 2026
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive.module;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.AutoLog;

/** Interface for the swerve module input/output abstraction. */
public interface ModuleIO {
  /** Contains all of the inputs received from the module hardware. */
  @AutoLog
  public static class ModuleIOInputs {
    // Drive Motor Inputs
    /** Whether the drive motor controller is currently connected. */
    public boolean driveConnected = false;

    /** The current position of the drive motor in radians. */
    public Angle drivePosition = Radians.zero();

    /** The current velocity of the drive motor in radians per second. */
    public AngularVelocity driveVelocity = RadiansPerSecond.zero();

    /** The voltage currently being applied to the drive motor. */
    public Voltage driveAppliedVoltage = Volts.zero();

    /** The current being drawn by the drive motor. */
    public Current driveCurrent = Amps.zero();

    /** Whether the drive motor is functioning correctly. */
    public boolean driveHealthy = true;

    /** The full status of the drive motor controller. */
    public int[] driveStatus = new int[] {0, 0, 0, 0};

    // Turn Motor Inputs
    /** Whether the turn motor controller is currently connected. */
    public boolean turnConnected = false;

    /** The current position of the turn motor in radians. */
    public Angle turnPosition = Radians.zero();

    /** The current velocity of the turn motor in radians per second. */
    public AngularVelocity turnVelocity = RadiansPerSecond.zero();

    /** The voltage currently being applied to the turn motor. */
    public Voltage turnAppliedVoltage = Volts.zero();

    /** The current being drawn by the turn motor. */
    public Current turnCurrent = Amps.zero();

    /** Whether the turn motor is functioning correctly. */
    public boolean turnHealthy = true;

    /** The full status of the turn motor controller. */
    public int[] turnStatus = new int[] {0, 0, 0, 0};

    // Turn Encoder Inputs
    /** Whether the turn encoder is currently connected. */
    public boolean turnEncoderConnected = false;

    // Odometry Inputs
    /** Timestamps for odometry measurements. */
    public double[] odometryTimestamps = new double[] {};

    /** Drive positions for odometry measurements. */
    public double[] odometryDrivePositionsRad = new double[] {};

    /** Turn positions for odometry measurements. */
    public double[] odometryTurnPositionsRad = new double[] {};
  }

  /**
   * Updates the set of loggable inputs.
   *
   * @param inputs The inputs object to update.
   */
  public default void updateInputs(ModuleIOInputs inputs) {}

  /**
   * Run the drive motor at the specified open loop voltage.
   *
   * @param output The voltage to apply.
   */
  public default void setDriveOpenLoop(Voltage output) {}

  /**
   * Run the turn motor at the specified open loop voltage.
   *
   * @param output The voltage to apply.
   */
  public default void setTurnOpenLoop(Voltage output) {}

  /**
   * Run the drive motor at the specified angular velocity.
   *
   * @param velocity The target velocity in radians per second.
   */
  public default void setDriveVelocity(AngularVelocity velocity) {}

  /**
   * Run the turn motor to the specified rotation.
   *
   * @param rotation The target angle as an Angle measure.
   */
  public default void setTurnPosition(Angle rotation) {}

  /** Clears all faults and warnings. */
  public default void clearFaults() {}
}
