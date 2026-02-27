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

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {
  @AutoLog
  public static class ModuleIOInputs {
    public boolean driveConnected = false;
    public Angle drivePosition = Radians.of(0.0);
    public AngularVelocity driveVelocity = RadiansPerSecond.of(0.0);
    public Voltage driveAppliedVoltage = Volts.of(0.0);
    public Current driveCurrent = Amps.of(0.0);

    public boolean turnConnected = false;
    public Angle turnPosition = Radians.of(0.0);
    public AngularVelocity turnVelocity = RadiansPerSecond.of(0.0);
    public Voltage turnAppliedVoltage = Volts.of(0.0);
    public Current turnCurrent = Amps.of(0.0);

    public double[] odometryTimestamps = new double[] {};
    public double[] odometryDrivePositionsRad = new double[] {};
    public Rotation2d[] odometryTurnPositions = new Rotation2d[] {};
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ModuleIOInputs inputs) {}

  /** Run the drive motor at the specified open loop value. */
  public default void setDriveOpenLoop(Voltage output) {}

  /** Run the turn motor at the specified open loop value. */
  public default void setTurnOpenLoop(Voltage output) {}

  /** Run the drive motor at the specified velocity. */
  public default void setDriveVelocity(AngularVelocity velocity) {}

  /** Run the turn motor to the specified rotation. */
  public default void setTurnPosition(Angle rotation) {}
}
