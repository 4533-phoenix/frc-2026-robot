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
  public static final int CAN_ID = 15;

  // ---------- Gear Ratios ----------
  /** Gear ratio: 112 motor rotations = 1 output shaft rotation for the arm. */
  public static final double REDUCTION = 112.0;

  // ---------- Encoder Conversion Factors ----------
  /** Converts internal motor rotations to radians for the arm. */
  public static final double INTERNAL_ENCODER_POSITION_FACTOR = (2.0 * Math.PI) / REDUCTION;
  /** Converts internal motor velocity (RPM) to radians per second for the arm. */
  public static final double INTERNAL_ENCODER_VELOCITY_FACTOR =
      ((2.0 * Math.PI) / 60.0) / REDUCTION;

  // ---------- Encoder Offsets ----------
  /** Absolute encoder offset to align 0 degrees with the fully retracted position. */
  public static final Angle GLOBAL_ENCODER_OFFSET = Degrees.of(83.45);

  // ---------- Motor Current Limits ----------
  /** Max current draw for the arm motor. */
  public static final Current MOTOR_CURRENT_LIMIT = Amps.of(30);

  // ---------- Arm PID Configuration ----------
  /** Proportional gain for arm position control. */
  public static final double KP = 0.1;
  /** Derivative gain for arm position control. */
  public static final double KD = 0.0;

  // ---------- Arm Feedforward Configuration ----------
  /** Static friction feedforward gain for the arm. */
  public static final double KS = 0.1;
  /** Gravity feedforward gain for the arm. */
  public static final double KG = 0.0;
  /** Velocity feedforward gain for the arm. */
  public static final double KV = 1.9; // 1.9
  /** Acceleration feedforward gain for the arm. */
  public static final double KA = 0.0;

  // ---------- Arm Motion Profile Constraints ----------
  /** Maximum angular velocity for the arm during movement. */
  public static final AngularVelocity CRUISE_VELOCITY = RadiansPerSecond.of(7.0);
  /** Maximum angular acceleration for the arm during movement. */
  public static final AngularAcceleration MAX_ACCELERATION = RadiansPerSecondPerSecond.of(6.0);

  // ---------- Arm Positions ----------
  /** Angle of the arm when fully deployed for intaking. */
  public static final Angle DEPLOYED_POSITION = Degrees.of(37.0);
  /** Angle of the arm when fully retracted inside the robot perimeter. */
  public static final Angle RETRACTED_POSITION = Degrees.of(131.0);
  /** Tolerance for considering the arm at a specific PID setpoint. */
  public static final Angle PID_TOLERANCE = Degrees.of(1.0);
  /** Tolerance for considering the arm to be physically in the desired state. */
  public static final Angle STATE_TOLERANCE = Degrees.of(5.0);
  /** Safety buffer for soft limits. */
  public static final Angle SOFT_LIMIT_TOLERANCE = Degrees.of(5.0);

  /** The motor model and quantity used for the arm gearbox. */
  public static final DCMotor GEARBOX = DCMotor.getNEO(1);

  /** Maximum angular velocity for the arm to be considered "still". */
  public static final AngularVelocity VELOCITY_GATE = RadiansPerSecond.of(0.5);
  /** Maximum allowable error between internal and absolute encoder positions. */
  public static final Angle ERROR_THRESHOLD = Radians.of(0.05);
}
