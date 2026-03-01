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

/** Hardware and tuning constants for the intake subsystem. */
public final class IntakeConstants {
  // ---------- CAN IDs ----------
  /** CAN ID for the arm lifting motor. */
  public static final int armMotorCanId = 15;
  /** CAN ID for the roller spinner motor. */
  public static final int spinnerMotorCanId = 16;

  // ---------- Gear Ratios ----------
  /** Gear ratio: 112 motor rotations = 1 output shaft rotation for the arm. */
  public static final double armMotorReduction = 112.0;
  /** Gear ratio: 2.13 motor rotations = 1 output shaft rotation for the spinner. */
  public static final double spinnerMotorReduction = 2.13;

  // ---------- Encoder Conversion Factors ----------
  /** Converts internal motor rotations to radians for the arm. */
  public static final double armInternalEncoderPositionFactor = (2.0 * Math.PI) / armMotorReduction;
  /** Converts internal motor velocity (RPM) to radians per second for the arm. */
  public static final double armInternalEncoderVelocityFactor =
      ((2.0 * Math.PI) / 60.0) / armMotorReduction;
  /** Converts internal motor rotations to radians for the spinner. */
  public static final double spinnerInternalEncoderPositionFactor =
      (2.0 * Math.PI) / spinnerMotorReduction;
  /** Converts internal motor velocity (RPM) to radians per second for the spinner. */
  public static final double spinnerInternalEncoderVelocityFactor =
      ((2.0 * Math.PI) / 60.0) / spinnerMotorReduction;

  // ---------- Encoder Offsets ----------
  /** Absolute encoder offset to align 0 degrees with the fully retracted position. */
  public static final Angle globalEncoderOffset = Degrees.of(83.45);

  // ---------- Motor Current Limits ----------
  /** Max current draw for the arm motor. */
  public static final Current armMotorCurrentLimit = Amps.of(30);
  /** Max current draw for the spinner motor. */
  public static final Current spinnerMotorCurrentLimit = Amps.of(30);

  // ---------- Arm PID Configuration ----------
  /** Proportional gain for arm position control. */
  public static final double armKp = 0.0;
  /** Derivative gain for arm position control. */
  public static final double armKd = 0.0;

  // ---------- Arm Feedforward Configuration ----------
  /** Static friction feedforward gain for the arm. */
  public static final double armKs = 0.0;
  /** Gravity feedforward gain for the arm. */
  public static final double armKg = 0.0;
  /** Velocity feedforward gain for the arm. */
  public static final double armKv = 0.0;
  /** Acceleration feedforward gain for the arm. */
  public static final double armKa = 0.0;

  // ---------- Spinner PID Configuration ----------
  /** Proportional gain for spinner velocity control. */
  public static final double spinnerKp = 0.0;
  /** Derivative gain for spinner velocity control. */
  public static final double spinnerKd = 0.0;

  // ---------- Spinner Feedforward Configuration ----------
  /** Static friction feedforward gain for the spinner. */
  public static final double spinnerKs = 0.0;
  /** Velocity feedforward gain for the spinner. */
  public static final double spinnerKv = 0.0375;
  /** Acceleration feedforward gain for the spinner. */
  public static final double spinnerKa = 0.0;

  // ---------- Arm Motion Profile Constraints ----------
  /** Maximum angular velocity for the arm during movement. */
  public static final AngularVelocity armCruiseVelocity = RadiansPerSecond.of(3.5);
  /** Maximum angular acceleration for the arm during movement. */
  public static final AngularAcceleration armMaxAcceleration = RadiansPerSecondPerSecond.of(6.0);

  // ---------- Arm Positions ----------
  /** Angle of the arm when fully deployed for intaking. */
  public static final Angle armDeployedPosition = Degrees.of(37.0);
  /** Angle of the arm when fully retracted inside the robot perimeter. */
  public static final Angle armRetractedPosition = Degrees.of(131.0);
  /** Tolerance for considering the arm at a specific PID setpoint. */
  public static final Angle armPositionPIDTolerance = Degrees.of(1.0);
  /** Tolerance for considering the arm deployed enough to start intaking. */
  public static final Angle armPositionIntakeTolerance = Degrees.of(5.0);
  /** Safety buffer for soft limits. */
  public static final Angle armPositionSoftLimitTolerance = Degrees.of(5.0);

  // ---------- Spinner Velocities ----------
  /** Velocity for pulling game pieces in. */
  public static final AngularVelocity spinnerIntakeVelocity = RadiansPerSecond.of(180.0);
  /** Velocity for pushing game pieces out. */
  public static final AngularVelocity spinnerExtakeVelocity = RadiansPerSecond.of(-180.0);
}
