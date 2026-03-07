// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.intake.arm;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.*;

/** Hardware and tuning constants for the intake subsystem. */
public final class ArmConstants {
  // ---------- CAN IDs ----------
  /** CAN ID for the arm lifting motor. */
  public static final int canId = 15;

  // ---------- Gear Ratios ----------
  /** Gear ratio: 112 motor rotations = 1 output shaft rotation for the arm. */
  public static final double motorReduction = 112.0;

  // ---------- Encoder Conversion Factors ----------
  /** Converts internal motor rotations to radians for the arm. */
  public static final double internalEncoderPositionFactor = (2.0 * Math.PI) / motorReduction;
  /** Converts internal motor velocity (RPM) to radians per second for the arm. */
  public static final double internalEncoderVelocityFactor =
      ((2.0 * Math.PI) / 60.0) / motorReduction;

  // ---------- Encoder Offsets ----------
  /** Absolute encoder offset to align 0 degrees with the fully retracted position. */
  public static final Angle globalEncoderOffset = Degrees.of(83.45);

  // ---------- Motor Current Limits ----------
  /** Max current draw for the arm motor. */
  public static final Current motorCurrentLimit = Amps.of(30);

  // ---------- Arm PID Configuration ----------
  /** Proportional gain for arm position control. */
  public static final double armKp = 0.1;
  /** Derivative gain for arm position control. */
  public static final double armKd = 0.0;

  // ---------- Arm Feedforward Configuration ----------
  /** Static friction feedforward gain for the arm. */
  public static final double armKs = 0.1;
  /** Gravity feedforward gain for the arm. */
  public static final double armKg = 0.0;
  /** Velocity feedforward gain for the arm. */
  public static final double armKv = 1.9; // 1.9
  /** Acceleration feedforward gain for the arm. */
  public static final double armKa = 0.0;

  // ---------- Arm Motion Profile Constraints ----------
  /** Maximum angular velocity for the arm during movement. */
  public static final AngularVelocity cruiseVelocity = RadiansPerSecond.of(7.0);
  /** Maximum angular acceleration for the arm during movement. */
  public static final AngularAcceleration maxAcceleration = RadiansPerSecondPerSecond.of(6.0);

  // ---------- Arm Positions ----------
  /** Angle of the arm when fully deployed for intaking. */
  public static final Angle deployedPosition = Degrees.of(37.0);
  /** Angle of the arm when fully retracted inside the robot perimeter. */
  public static final Angle retractedPosition = Degrees.of(131.0);
  /** Tolerance for considering the arm at a specific PID setpoint. */
  public static final Angle positionPIDTolerance = Degrees.of(1.0);
  /** Tolerance for considering the arm deployed enough to start intaking. */
  public static final Angle intakeTolerance = Degrees.of(5.0);
  /** Safety buffer for soft limits. */
  public static final Angle softLimitTolerance = Degrees.of(5.0);

  /** The motor model and quantity used for the arm gearbox. */
  public static final DCMotor gearbox = DCMotor.getNEO(1);
}
