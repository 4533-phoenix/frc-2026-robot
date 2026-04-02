// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Modified by FRC Team 4533 (Phoenix) 2026
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.*;

/**
 * Hardware and tuning constants for the drive subsystem.
 *
 * <p>Contains physical dimensions, CAN IDs, gear ratios, motor current limits, and PID tuning
 * values for both real and simulated hardware.
 */
public class DriveConstants {
  public static final LinearVelocity STATIONARY_VELOCITY_THRESHOLD = MetersPerSecond.of(0.01);
  public static final AngularVelocity STATIONARY_ROTATION_THRESHOLD = RadiansPerSecond.of(0.01);

  // Physical constants
  /** The maximum achievable linear velocity of the robot in meters per second. */
  public static final LinearVelocity MAX_LINEAR_VELOCITY = MetersPerSecond.of(4.0);

  /** The maximum achievable linear acceleration of the robot in meters per second squared. */
  public static final LinearAcceleration MAX_LINEAR_ACCELERATION =
      MetersPerSecondPerSecond.of(15.0);

  /** The frequency at which odometry calculations are updated. */
  public static final Frequency ODOMETRY_FREQUENCY = Hertz.of(200);

  /** The distance between the left and right wheels. */
  public static final Distance TRACK_WIDTH = Inches.of(20.5);

  /** The distance between the front and back wheels. */
  public static final Distance WHEEL_BASE = Inches.of(20.5);

  /** The radius of the circle defined by the module locations. */
  public static final Distance DRIVE_BASE_RADIUS =
      Meters.of(Math.hypot(TRACK_WIDTH.in(Meters) / 2.0, WHEEL_BASE.in(Meters) / 2.0));

  /** Maximum heading error allowed when determining if the robot is aligned to shoot. */
  public static final Angle HEADING_ALIGNMENT_TOLERANCE = Degrees.of(3.0);

  /**
   * Represents the hardware configuration for a single swerve module.
   *
   * @param name A descriptive name for the module.
   * @param driveCanId The CAN ID of the drive motor controller.
   * @param turnCanId The CAN ID of the turn motor controller.
   * @param encoderCanId The CAN ID of the absolute CANcoder.
   * @param zeroOffset The absolute encoder offset to read zero when the wheel is facing forward.
   * @param translation The physical location of the module relative to the center of the robot.
   */
  public record SwerveModuleConfig(
      String name,
      int driveCanId,
      int turnCanId,
      int encoderCanId,
      Angle zeroOffset,
      Translation2d translation) {}

  /**
   * Hardware configurations for all four swerve modules.
   *
   * <p>Index Order: Front Left (0), Front Right (1), Back Left (2), Back Right (3)
   */
  public static final SwerveModuleConfig[] MODULE_CONFIGS = {
    // Front Left (Module 0)
    new SwerveModuleConfig(
        "Front Left",
        2,
        3,
        4,
        Degrees.of(35.51),
        new Translation2d(TRACK_WIDTH.in(Meters) / 2.0, WHEEL_BASE.in(Meters) / 2.0)),
    // Front Right (Module 1)
    new SwerveModuleConfig(
        "Front Right",
        5,
        6,
        7,
        Degrees.of(293.03),
        new Translation2d(TRACK_WIDTH.in(Meters) / 2.0, -WHEEL_BASE.in(Meters) / 2.0)),
    // Back Left (Module 2)
    new SwerveModuleConfig(
        "Back Left",
        8,
        9,
        10,
        Degrees.of(123.49),
        new Translation2d(-TRACK_WIDTH.in(Meters) / 2.0, WHEEL_BASE.in(Meters) / 2.0)),
    // Back Right (Module 3)
    new SwerveModuleConfig(
        "Back Right",
        11,
        12,
        13,
        Degrees.of(7.73),
        new Translation2d(-TRACK_WIDTH.in(Meters) / 2.0, -WHEEL_BASE.in(Meters) / 2.0))
  };

  /**
   * The locations of the modules relative to the center of the robot.
   *
   * <p>Order: Front Left, Front Right, Back Left, Back Right
   */
  public static final Translation2d[] MODULE_TRANSLATIONS =
      new Translation2d[] {
        MODULE_CONFIGS[0].translation(),
        MODULE_CONFIGS[1].translation(),
        MODULE_CONFIGS[2].translation(),
        MODULE_CONFIGS[3].translation()
      };

  // Device CAN IDs
  /** CAN ID for the IMU (gyroscope). */
  public static final int IMU_CAN_ID = 14;

  // Dual gyro parameters for drift compensation
  /** The gain for correcting drift based on the secondary gyro. */
  public static final double DRIFT_GAIN = 0.01;

  /** The threshold for angular error before correction is applied. */
  public static final Angle ERROR_THRESHOLD = Degrees.of(0.5);

  /** The maximum angular correction allowed per control frame. */
  public static final Angle MAX_CORRECTION_PER_FRAME = Degrees.of(0.1);

  /** The velocity threshold below which the robot is considered stationary. */
  public static final AngularVelocity VELOCITY_GATE = DegreesPerSecond.of(1.0);

  /** Latency compensation for NavX over USB (seconds). */
  public static final Time NAVX_LATENCY_SEC = Seconds.of(0.010);

  /** Latency compensation for Canandgyro over CAN (seconds). */
  public static final Time CANANDGYRO_LATENCY_SEC = Seconds.of(0.0025);

  // Drive motor configuration
  /** Whether the drive motor is inverted. */
  public static final boolean DRIVE_INVERTED = false;

  /** Maximum current limit for the drive motors. */
  public static final Current DRIVE_MOTOR_CURRENT_LIMIT = Amps.of(40);

  /** Secondary current limit for the drive motors. */
  public static final Current DRIVE_MOTOR_SECONDARY_CURRENT_LIMIT = Amps.of(80);

  /** The radius of the drive wheels. */
  public static final Distance WHEEL_RADIUS = Inches.of(1.907);

  /** The gear reduction between the drive motor and the wheel. */
  public static final double DRIVE_MOTOR_REDUCTION = 6.75;

  /** The gearbox model for the drive motor. */
  public static final DCMotor DRIVE_GEARBOX = DCMotor.getNEO(1);

  // Drive encoder configuration
  /** Conversion factor for drive position from motor rotations to meters. */
  public static final double DRIVE_ENCODER_POSITION_FACTOR = 2 * Math.PI / DRIVE_MOTOR_REDUCTION;

  /** Conversion factor for drive velocity from motor RPM to meters per second. */
  public static final double DRIVE_ENCODER_VELOCITY_FACTOR =
      (2 * Math.PI) / 60.0 / DRIVE_MOTOR_REDUCTION;

  // Drive PID configuration
  /** Proportional gain for the drive motor PID controller. */
  public static final double DRIVE_KP = 0.015;

  /** Derivative gain for the drive motor PID controller. */
  public static final double DRIVE_KD = 0.0;

  /** Static friction feedforward gain for the drive motor. */
  public static final double DRIVE_KS = 0.0;

  /** Velocity feedforward gain for the drive motor. */
  public static final double DRIVE_KV = 0.1;

  /** Acceleration feedforward gain for the drive motor. */
  public static final double DRIVE_KA = 0.0;

  // Turn motor configuration
  /** Whether the turn motor is inverted. */
  public static final boolean TURN_INVERTED = false;

  /** Maximum current limit for the turn motors. */
  public static final Current TURN_MOTOR_CURRENT_LIMIT = Amps.of(40);

  /** Secondary current limit for the turn motors. */
  public static final Current TURN_MOTOR_SECONDARY_CURRENT_LIMIT = Amps.of(80);

  /** The gear reduction between the turn motor and the module. */
  public static final double TURN_MOTOR_REDUCTION = 12.8;

  /** The gearbox model for the turn motor. */
  public static final DCMotor TURN_GEARBOX = DCMotor.getNEO(1);

  // Turn encoder configuration
  /** Whether the turn encoder is inverted. */
  public static final boolean TURN_ENCODER_INVERTED = false;

  /** Conversion factor for turn position from encoder rotations to radians. */
  public static final double TURN_ENCODER_POSITION_FACTOR = 2 * Math.PI;

  /** Conversion factor for turn velocity from encoder RPM to radians per second. */
  public static final double TURN_ENCODER_VELOCITY_FACTOR = 2 * Math.PI;

  // Turn PID configuration
  /** Proportional gain for the turn motor PID controller. */
  public static final double TURN_KP = 0.4;

  /** Derivative gain for the turn motor PID controller. */
  public static final double TURN_KD = 0.01;

  /** Minimum input range for turn PID (in radians). */
  public static final double TURN_PID_MIN_INPUT = -Math.PI;

  /** Maximum input range for turn PID (in radians). */
  public static final double TURN_PID_MAX_INPUT = Math.PI;

  // PathPlanner configuration
  /** Total mass of the robot. */
  public static final Mass ROBOT_MASS = Pounds.of(117);

  // ---------- Drive Command Configuration ----------
  /** Joystick deadband for linear and rotational inputs. */
  public static final double JOYSTICK_DEADBAND = 0.1;

  // Angle PID configuration (for rotation-locked drive commands)
  /** Proportional gain for rotational PID control. */
  public static final double ANGLE_KP = 10.0;

  /** Derivative gain for rotational PID control. */
  public static final double ANGLE_KD = 0.5;

  /** Maximum angular velocity for the rotation motion profile in radians per second. */
  public static final AngularVelocity MAX_ANGULAR_VELOCITY = RadiansPerSecond.of(16.0);

  /** Maximum angular acceleration for the rotation motion profile in radians per second squared. */
  public static final AngularAcceleration MAX_ANGULAR_ACCELERATION =
      RadiansPerSecondPerSecond.of(40.0);

  // Feedforward characterization configuration
  /** Delay in seconds before feedforward characterization starts ramping voltage. */
  public static final Time FF_START_DELAY = Seconds.of(2.0);

  /** Voltage ramp rate in volts per second for feedforward characterization. */
  public static final double FF_RAMP_RATE = 0.1;

  // Wheel radius characterization configuration
  /** Maximum angular velocity in radians per second for wheel radius characterization. */
  public static final AngularVelocity WHEEL_RADIUS_MAX_VELOCITY = RadiansPerSecond.of(0.25);

  /** Angular velocity ramp rate in radians per second squared for wheel radius characterization. */
  public static final AngularAcceleration WHEEL_RADIUS_RAMP_RATE =
      RadiansPerSecondPerSecond.of(0.05);

  /** Moment of inertia of the robot in kilogram-square meters. */
  public static final MomentOfInertia ROBOT_MOI = KilogramSquareMeters.of(6.883);

  /** Coefficient of friction for the wheels. */
  public static final double WHEEL_COF = 1.1;
}
