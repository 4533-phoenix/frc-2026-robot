// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

/** Hardware and tuning constants for the intake subsystem. */
public final class IntakeConstants {
  // CAN IDs
  public static final int armMotorCanId = 15;
  public static final int spinnerMotorCanId = 16;

  // Gear ratio: 112 NEO rotations = 1 output shaft rotation
  public static final double armMotorReduction = 112.0;

  // Encoder conversion factors
  public static final double armInternalEncoderPositionFactor = (2.0 * Math.PI) / armMotorReduction;
  public static final double armInternalEncoderVelocityFactor =
      ((2.0 * Math.PI) / 60.0) / armMotorReduction;

  // Global encoder offset
  public static final Angle globalEncoderOffset = Degrees.of(83.45);

  // Motor current limits
  public static final Current armMotorCurrentLimit = Amps.of(30);
  public static final Current spinnerMotorCurrentLimit = Amps.of(30);

  // Arm PID configuration
  public static final double armKp = 0.05;
  public static final double armKd = 0.1;

  // Arm Feedforward configuration
  public static final double armKs = 0.1;
  public static final double armKg = 0.1;
  public static final double armKv = 1.9;
  public static final double armKa = 0.0;

  // Arm Motion Profile Constraints
  public static final AngularVelocity armCruiseVelocity = RadiansPerSecond.of(3.5);
  public static final AngularAcceleration armMaxAcceleration = RadiansPerSecondPerSecond.of(6.0);

  // Arm positions
  public static final Angle armDeployedPosition = Degrees.of(37.0);
  public static final Angle armRetractedPosition = Degrees.of(131.0);
  public static final Angle armPositionPIDTolerance = Degrees.of(1.0);
  public static final Angle armPositionIntakeTolerance = Degrees.of(5.0);
  public static final Angle armPositionSoftLimitTolerance = Degrees.of(5.0);

  // Spinner voltage
  public static final Voltage spinnerIntakeVoltage = Volts.of(6.0);
}
