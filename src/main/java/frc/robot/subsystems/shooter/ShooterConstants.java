// Copyright (c) 2026 FRC Team 4533 (Phoenix)
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.*;
import frc.robot.subsystems.shooter.Shooter.ShooterState;

/**
 * Physical constants and hardware configurations for the shooter subsystem.
 *
 * <p>Includes data on the game piece, actuator geometry, gear ratios, and motor controller
 * settings.
 */
public final class ShooterConstants {
  //  Hardware IDs
  /** CAN ID for the flywheel motor controller. */
  public static final int CAN_ID = 18;

  /** PWM channel for the hood servo motor. */
  public static final int SERVO_CHANNEL = 0;

  /** Whether the flywheel motor is inverted. */
  public static final boolean FLYWHEEL_INVERTED = false;

  //  Hood geometry & motion
  /** Length of the cranking arm in the hood mechanism. */
  public static final Distance CRANK_ARM_LENGTH = Inches.of(6.403);

  /** Distance between the pivot points of the hood linkage. */
  public static final Distance GROUND_LINK_DISTANCE = Inches.of(7.521);

  /** Offset angle between the crank tangent and the actual launch angle. */
  public static final Angle CRANK_TANGENT_TO_LAUNCH_ANGLE = Degrees.of(12.875);

  /** Total angular range of the hood mechanism. */
  public static final Angle HOOD_TOTAL_ANGLE = Degrees.of(149.007);

  /** Minimum physical length of the hood servo actuator. */
  public static final Distance SERVO_MIN_LENGTH = Inches.of(6.925);

  /** Maximum physical length of the hood servo actuator. */
  public static final Distance SERVO_MAX_LENGTH = Inches.of(10.5);

  /** Maximum linear speed of the hood servo actuator. */
  public static final LinearVelocity MAX_SERVO_VELOCITY = MetersPerSecond.of(0.02);

  /** Tolerance for the hood servo actuator length. */
  public static final Distance HOOD_LENGTH_TOLERANCE = Meters.of(0.005);

  //  Flywheel constants
  /** Motor model for the flywheel (Neo Vortex driven by SparkFlex). */
  public static final DCMotor GEARBOX = DCMotor.getNeoVortex(1);

  /** Gear reduction ratio for the flywheel (motor to wheel). */
  public static final double REDUCTION = 1.0;

  /** Moment of inertia of the flywheel assembly. */
  public static final MomentOfInertia MOI = KilogramSquareMeters.of(0.0021);

  /** Radius of the flywheel driving wheel. */
  public static final Distance WHEEL_RADIUS = Inches.of(2.0);

  /** Allowed velocity error before the shooter is considered "ready". */
  public static final AngularVelocity ANGULAR_TOLERANCE = RadiansPerSecond.of(15.0);

  /** Maximum current limit for the flywheel motor. */
  public static final Current MOTOR_CURRENT_LIMIT = Amps.of(60.0);

  //  Encoder conversion factors
  /** Converts motor rotations to mechanism radians. */
  public static final double FLYWHEEL_ENCODER_POSITION_FACTOR = (2.0 * Math.PI) / REDUCTION;

  /** Converts motor RPM to mechanism rad/s. */
  public static final double FLYWHEEL_ENCODER_VELOCITY_FACTOR =
      ((2.0 * Math.PI) / 60.0) / REDUCTION;

  // PID constants for flywheel velocity control
  /** Proportional gain for flywheel velocity control. */
  public static final double FLYWHEEL_KP = 0.0085;

  /** Integral gain for flywheel velocity control. */
  public static final double FLYWHEEL_KI = 0.0;

  /** Derivative gain for flywheel velocity control. */
  public static final double FLYWHEEL_KD = 0.0001;

  /** Static friction feedforward gain for the flywheel. */
  public static final double FLYWHEEL_KS = 0.0;

  /** Velocity feedforward gain for the flywheel. */
  public static final double FLYWHEEL_KV = 0.0175;

  /** Acceleration feedforward gain for the flywheel. */
  public static final double FLYWHEEL_KA = 0.0;

  //  Shooter state constants
  /** Preset shooter state for lobbing game pieces into the coral station. */
  public static final ShooterState LOB_STATE =
      new ShooterState(RotationsPerSecond.of(60.0), Degrees.of(70.0));

  /** Default shooter state with zero velocity and angle. */
  public static final ShooterState DEFAULT_STATE =
      new ShooterState(RadiansPerSecond.of(0), Degrees.of(85.0));

  //  Aiming / lead constants
  /** Estimated time of flight for the game piece from shooter to target. */
  public static final Time ESTIMATED_TOF = Seconds.of(0.55);

  /**
   * Position of the shooter on the robot relative to the robot center (robot-frame). X is forward,
   * Y is left.
   */
  public static final Translation2d SHOOTER_ROBOT_OFFSET =
      new Translation2d(Inches.of(-10.25), Inches.of(8.5));
}
