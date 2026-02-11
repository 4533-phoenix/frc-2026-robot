// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation2d;

/** Hardware and tuning constants for the intake subsystem. */
public final class IntakeConstants {
  // CAN IDs
  public static final int armMotorCanId = 15;
  public static final int spinnerMotorCanId = 16;

  // DIO pins
  public static final int dutyCycleEncoderPin = 8;

  // Gear ratio: 112 NEO rotations = 1 output shaft rotation
  public static final double armMotorReduction = 112.0;

  // Encoder conversion factors (motor rotations -> output radians)
  public static final double armEncoderPositionFactor = (2.0 * Math.PI) / armMotorReduction;
  public static final double armEncoderVelocityFactor =
      ((2.0 * Math.PI) / 60.0) / armMotorReduction;

  // Global encoder offset (radians)
  public static final double globalEncoderOffsetRad = Math.toRadians(304.8);

  // Motor current limits
  public static final int armMotorCurrentLimit = 40;
  public static final int spinnerMotorCurrentLimit = 30;

  // Arm PID configuration (on the SparkMax, in output-shaft radians)
  public static final double armKp = 12.0; // TODO: Tune
  public static final double armKi = 0.0;
  public static final double armKd = 0.0; // TODO: Tune

  // Arm Feedforward configuration
  public static final double armKs = 0.0; // TODO: Tune
  public static final double armKg = 0.5; // TODO: Tune
  public static final double armKv = 4.65; // TODO: Tune
  public static final double armKa = 0.04; // TODO: Tune

  // Arm Motion Profile Constraints
  public static final double armMaxVelocityRadPerSec = Math.PI * 2; // TODO: Tune
  public static final double armMaxAccelerationRadPerSecSquared = Math.PI * 4; // TODO: Tune

  // Arm positions (output shaft, relative to stow = 0)
  public static final Rotation2d armDeployedPosition = Rotation2d.fromDegrees(101.9);
  public static final Rotation2d armRetractedPosition = Rotation2d.kZero;

  // Arm soft limits (radians, relative to stow = 0)
  public static final double armForwardSoftLimitRad = Math.toRadians(108.0);
  public static final double armReverseSoftLimitRad = Math.toRadians(-5.0);

  // Spinner voltage
  public static final double spinnerIntakeVoltage = 6.0;
}
