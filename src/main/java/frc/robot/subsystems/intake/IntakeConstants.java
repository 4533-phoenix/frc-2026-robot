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

  // Encoder conversion factors (motor rotations -> output radians)
  public static final double armEncoderPositionFactor = (2.0 * Math.PI) / armMotorReduction;
  public static final double armEncoderVelocityFactor =
      ((2.0 * Math.PI) / 60.0) / armMotorReduction;

  // Global encoder offset (radians)
  public static final Angle globalEncoderOffset = Degrees.of(304.8);

  // Motor current limits
  public static final Current armMotorCurrentLimit = Amps.of(40);
  public static final Current spinnerMotorCurrentLimit = Amps.of(40);

  // Arm PID configuration
  public static final double armKp = 1.0;
  public static final double armKi = 0.0;
  public static final double armKd = 0.0;

  // Arm Feedforward configuration
  public static final double armKs = 0.0;
  public static final double armKg = 0.0;
  public static final double armKv = 0.0;
  public static final double armKa = 0.0;

  // Arm Motion Profile Constraints
  public static final AngularVelocity armMaxVelocity = RadiansPerSecond.of(Math.PI * 2);
  public static final AngularAcceleration armMaxAcceleration =
      RadiansPerSecondPerSecond.of(Math.PI * 4);

  // Arm positions (output shaft, relative to stow = 0)
  public static final Angle armDeployedPosition = Degrees.of(101.9);
  public static final Angle armRetractedPosition = Degrees.of(0.0);

  // MaxMotion BS
  public static final Angle armPositionTolerance = Degrees.of(2.0);

  // Spinner voltage
  public static final Voltage spinnerIntakeVoltage = Volts.of(6.0);
}
