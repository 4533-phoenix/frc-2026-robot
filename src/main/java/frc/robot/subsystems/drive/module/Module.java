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
import static frc.robot.subsystems.drive.DriveConstants.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.lib.FaultUtil;
import org.littletonrobotics.junction.Logger;

/**
 * Subsystem for a single swerve drive module.
 *
 * <p>Handles the control of drive and turn motors for a single wheel, including kinematics
 * conversions and hardware fault monitoring.
 */
public class Module {
  private final ModuleIO io;
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
  private final int index;
  private final String name;

  private final Alert driveDisconnectedAlert;
  private final Alert turnDisconnectedAlert;
  private final Alert driveFaultAlert;
  private final Alert turnFaultAlert;
  private final Alert turnEncoderDisconnectedAlert;
  private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};

  /**
   * Creates a new Module subsystem.
   *
   * @param io The abstraction layer for the module hardware.
   * @param index The index of the module (0-3).
   */
  public Module(ModuleIO io, int index) {
    this.io = io;
    this.index = index;
    this.name = MODULE_CONFIGS[index].name();

    driveDisconnectedAlert = new Alert(name + " drive motor disconnected", AlertType.kError);
    turnDisconnectedAlert = new Alert(name + " turn motor disconnected", AlertType.kError);
    driveFaultAlert = new Alert(name + " drive motor fault detected", AlertType.kError);
    turnFaultAlert = new Alert(name + " turn motor fault detected", AlertType.kError);
    turnEncoderDisconnectedAlert = new Alert(name + " turn encoder disconnected", AlertType.kError);
  }

  /** Updates hardware inputs, calculates odometry data, and updates status alerts. */
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Drive/Module" + Integer.toString(index), inputs);

    // Calculate positions for odometry
    int sampleCount = inputs.odometryTimestamps.length;
    odometryPositions = new SwerveModulePosition[sampleCount];
    for (int i = 0; i < sampleCount; i++) {
      Distance positionMeters = WHEEL_RADIUS.times(inputs.odometryDrivePositionsRad[i]);
      Rotation2d angle = new Rotation2d(inputs.odometryTurnPositionsRad[i]);
      odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
    }

    // Update alerts
    driveDisconnectedAlert.set(!inputs.driveConnected);
    turnDisconnectedAlert.set(!inputs.turnConnected);
    turnEncoderDisconnectedAlert.set(!inputs.turnEncoderConnected);

    // Check for drive faults/warnings
    if (inputs.driveConnected) {
      driveFaultAlert.set(!inputs.driveHealthy);
      if (!inputs.driveHealthy) {
        driveFaultAlert.setText(
            FaultUtil.getArrayString(
                name + " Module Drive Faults: ", FaultUtil.getSparkFaults(inputs.driveStatus[0])));
      }
    } else {
      driveFaultAlert.set(false);
    }

    if (inputs.turnConnected) {
      turnFaultAlert.set(!inputs.turnHealthy);
      if (!inputs.turnHealthy) {
        turnFaultAlert.setText(
            FaultUtil.getArrayString(
                name + " Module Turn Faults: ", FaultUtil.getSparkFaults(inputs.turnStatus[0])));
      }
    } else {
      turnFaultAlert.set(false);
    }
  }

  /**
   * Runs the module with the specified setpoint state.
   *
   * <p>Optimizes the state to minimize turn angle (turning &lt; 90 degrees instead of &gt; 90) and
   * scales the drive velocity based on the cosine of the error between the setpoint angle and
   * current angle.
   *
   * @param state The desired state (angle and velocity) for the module.
   */
  public void runSetpoint(SwerveModuleState state) {
    // Optimize velocity setpoint to minimize turning
    state.optimize(new Rotation2d(getCurrentAngle()));

    // Scale speed based on angle error to prevent driving while turning
    state.cosineScale(new Rotation2d(inputs.turnPosition));

    // Apply setpoints to hardware
    io.setDriveVelocity(RadiansPerSecond.of(state.speedMetersPerSecond / WHEEL_RADIUS.in(Meters)));
    io.setTurnPosition(Radians.of(state.angle.getRadians()));
  }

  /**
   * Runs the module with the specified output while holding the turn module to zero degrees.
   *
   * @param output The voltage to apply to the drive motor.
   */
  public void runCharacterization(Voltage output) {
    io.setDriveOpenLoop(output);
    io.setTurnPosition(Radians.zero());
  }

  /** Disables all outputs to motors. */
  public void stop() {
    io.setDriveOpenLoop(Volts.zero());
    io.setTurnOpenLoop(Volts.zero());
  }

  /**
   * Returns the current turn angle of the module.
   *
   * @return The current angle as an Angle measure.
   */
  public Angle getCurrentAngle() {
    return inputs.turnPosition;
  }

  /**
   * Returns the current drive position of the module in meters.
   *
   * @return The current distance as a Distance measure.
   */
  public Distance getCurrentPosition() {
    return WHEEL_RADIUS.times(inputs.drivePosition.in(Radians));
  }

  /**
   * Returns the current drive velocity of the module in meters per second.
   *
   * @return The current velocity as a LinearVelocity measure.
   */
  public LinearVelocity getCurrentVelocity() {
    return MetersPerSecond.of(inputs.driveVelocity.in(RadiansPerSecond) * WHEEL_RADIUS.in(Meters));
  }

  /**
   * Returns the module position (turn angle and drive position).
   *
   * @return The current SwerveModulePosition.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getCurrentPosition(), new Rotation2d(getCurrentAngle()));
  }

  /**
   * Returns the module state (turn angle and drive velocity).
   *
   * @return The current SwerveModuleState.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getCurrentVelocity(), new Rotation2d(getCurrentAngle()));
  }

  /**
   * Returns the module positions received this cycle from the asynchronous thread.
   *
   * @return An array of SwerveModulePositions.
   */
  public SwerveModulePosition[] getOdometryPositions() {
    return odometryPositions;
  }

  /**
   * Returns the timestamps of the samples received this cycle from the asynchronous thread.
   *
   * @return An array of timestamps in seconds.
   */
  public double[] getOdometryTimestamps() {
    return inputs.odometryTimestamps;
  }

  /**
   * Returns the module position in radians for wheel radius characterization.
   *
   * @return The drive position in radians.
   */
  public Angle getWheelRadiusCharacterizationPosition() {
    return inputs.drivePosition;
  }

  /**
   * Returns the module velocity in rad/sec for feedforward characterization.
   *
   * @return The drive velocity in radians per second.
   */
  public AngularVelocity getFFCharacterizationVelocity() {
    return inputs.driveVelocity;
  }

  /**
   * Returns whether or not the subsystem is healthy
   *
   * @return True if the subsystem is healthy, false otherwise.
   */
  public boolean isHealthy() {
    return inputs.driveHealthy
        && inputs.turnHealthy
        && inputs.driveConnected
        && inputs.turnConnected
        && inputs.turnEncoderConnected;
  }

  /** Clears all faults and warnings. */
  public void clearFaults() {
    io.clearFaults();
  }
}
