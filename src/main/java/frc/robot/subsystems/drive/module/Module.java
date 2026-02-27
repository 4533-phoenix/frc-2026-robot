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
import org.littletonrobotics.junction.Logger;

public class Module {
  private final ModuleIO io;
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
  private final int index;

  private final Alert driveDisconnectedAlert;
  private final Alert turnDisconnectedAlert;
  private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};

  public Module(ModuleIO io, int index) {
    this.io = io;
    this.index = index;
    driveDisconnectedAlert =
        new Alert(
            "Disconnected drive motor on module " + Integer.toString(index) + ".",
            AlertType.kError);
    turnDisconnectedAlert =
        new Alert(
            "Disconnected turn motor on module " + Integer.toString(index) + ".", AlertType.kError);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Drive/Module" + Integer.toString(index), inputs);

    // Calculate positions for odometry
    int sampleCount = inputs.odometryTimestamps.length; // All signals are sampled together
    odometryPositions = new SwerveModulePosition[sampleCount];
    for (int i = 0; i < sampleCount; i++) {
      double positionMeters = inputs.odometryDrivePositionsRad[i] * wheelRadius.in(Meters);
      Rotation2d angle = inputs.odometryTurnPositions[i];
      odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
    }

    // Update alerts
    driveDisconnectedAlert.set(!inputs.driveConnected);
    turnDisconnectedAlert.set(!inputs.turnConnected);
  }

  /** Runs the module with the specified setpoint state. Mutates the state to optimize it. */
  public void runSetpoint(SwerveModuleState state) {
    // Optimize velocity setpoint
    state.optimize(Rotation2d.fromRadians(getCurrentAngle().in(Radians)));
    state.cosineScale(Rotation2d.fromRadians(inputs.turnPosition.in(Radians)));

    // Apply setpoints
    io.setDriveVelocity(RadiansPerSecond.of(state.speedMetersPerSecond / wheelRadius.in(Meters)));
    io.setTurnPosition(Radians.of(state.angle.getRadians()));
  }

  /** Runs the module with the specified output while controlling to zero degrees. */
  public void runCharacterization(Voltage output) {
    io.setDriveOpenLoop(output);
    io.setTurnPosition(Radians.of(0.0));
  }

  /** Disables all outputs to motors. */
  public void stop() {
    io.setDriveOpenLoop(Volts.of(0.0));
    io.setTurnOpenLoop(Volts.of(0.0));
  }

  /** Returns the current turn angle of the module. */
  public Angle getCurrentAngle() {
    return inputs.turnPosition;
  }

  /** Returns the current drive position of the module in meters. */
  public Distance getCurrentPosition() {
    return Meters.of(inputs.drivePosition.in(Radians) * wheelRadius.in(Meters));
  }

  /** Returns the current drive velocity of the module in meters per second. */
  public LinearVelocity getCurrentVelocityMetersPerSec() {
    return MetersPerSecond.of(inputs.driveVelocity.in(RadiansPerSecond) * wheelRadius.in(Meters));
  }

  /** Returns the module position (turn angle and drive position). */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        getCurrentPosition().in(Meters), Rotation2d.fromRadians(getCurrentAngle().in(Radians)));
  }

  /** Returns the module state (turn angle and drive velocity). */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        getCurrentVelocityMetersPerSec().in(MetersPerSecond),
        Rotation2d.fromRadians(getCurrentAngle().in(Radians)));
  }

  /** Returns the module positions received this cycle. */
  public SwerveModulePosition[] getOdometryPositions() {
    return odometryPositions;
  }

  /** Returns the timestamps of the samples received this cycle. */
  public double[] getOdometryTimestamps() {
    return inputs.odometryTimestamps;
  }

  /** Returns the module position in radians. */
  public Angle getWheelRadiusCharacterizationPosition() {
    return inputs.drivePosition;
  }

  /** Returns the module velocity in rad/sec. */
  public AngularVelocity getFFCharacterizationVelocity() {
    return inputs.driveVelocity;
  }
}
