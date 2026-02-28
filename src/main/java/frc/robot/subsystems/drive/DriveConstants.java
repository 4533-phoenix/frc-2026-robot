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

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.geometry.Rotation2d;
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
  // Physical constants
  /** The maximum achievable linear velocity of the robot in meters per second. */
  public static final LinearVelocity maxLinearVelocity = MetersPerSecond.of(4.0);
  /** The frequency at which odometry calculations are updated. */
  public static final Frequency odometryFrequency = Hertz.of(100);
  /** Less critacal odometry frequency */
  public static final Frequency odometryLowFrequency = Hertz.of(50);
  /** The distance between the left and right wheels. */
  public static final Distance trackWidth = Inches.of(20.5);
  /** The distance between the front and back wheels. */
  public static final Distance wheelBase = Inches.of(20.5);
  /** The radius of the circle defined by the module locations. */
  public static final Distance driveBaseRadius =
      Meters.of(Math.hypot(trackWidth.in(Meters) / 2.0, wheelBase.in(Meters) / 2.0));

  /**
   * Represents the hardware configuration for a single swerve module.
   *
   * @param driveCanId The CAN ID of the drive motor controller.
   * @param turnCanId The CAN ID of the turn motor controller.
   * @param encoderCanId The CAN ID of the absolute CANcoder.
   * @param zeroOffset The absolute encoder offset to read zero when the wheel is facing forward.
   * @param translation The physical location of the module relative to the center of the robot.
   */
  public record SwerveModuleConfig(
      int driveCanId,
      int turnCanId,
      int encoderCanId,
      Rotation2d zeroOffset,
      Translation2d translation) {}

  /**
   * Hardware configurations for all four swerve modules.
   *
   * <p>Index Order: Front Left (0), Front Right (1), Back Left (2), Back Right (3)
   */
  public static final SwerveModuleConfig[] moduleConfigs = {
    // Front Left (Module 0)
    new SwerveModuleConfig(
        2,
        3,
        4,
        Rotation2d.fromDegrees(35.51),
        new Translation2d(trackWidth.in(Meters) / 2.0, wheelBase.in(Meters) / 2.0)),
    // Front Right (Module 1)
    new SwerveModuleConfig(
        5,
        6,
        7,
        Rotation2d.fromDegrees(293.03),
        new Translation2d(trackWidth.in(Meters) / 2.0, -wheelBase.in(Meters) / 2.0)),
    // Back Left (Module 2)
    new SwerveModuleConfig(
        8,
        9,
        10,
        Rotation2d.fromDegrees(123.49),
        new Translation2d(-trackWidth.in(Meters) / 2.0, wheelBase.in(Meters) / 2.0)),
    // Back Right (Module 3)
    new SwerveModuleConfig(
        11,
        12,
        13,
        Rotation2d.fromDegrees(7.73),
        new Translation2d(-trackWidth.in(Meters) / 2.0, -wheelBase.in(Meters) / 2.0))
  };

  /**
   * The locations of the modules relative to the center of the robot.
   *
   * <p>Order: Front Left, Front Right, Back Left, Back Right
   */
  public static final Translation2d[] moduleTranslations =
      new Translation2d[] {
        moduleConfigs[0].translation(),
        moduleConfigs[1].translation(),
        moduleConfigs[2].translation(),
        moduleConfigs[3].translation()
      };

  // Device CAN IDs
  /** CAN ID for the IMU (gyroscope). */
  public static final int imuCanId = 14;

  // Dual gyro parameters for drift compensation
  /** The gain for correcting drift based on the secondary gyro. */
  public static final double driftGain = 0.01;
  /** The threshold for angular error before correction is applied. */
  public static final Angle errorThreshold = Degrees.of(0.5);
  /** The maximum angular correction allowed per control frame. */
  public static final Angle maxCorrectionPerFrame = Degrees.of(0.1);
  /** The velocity threshold below which the robot is considered stationary. */
  public static final AngularVelocity velocityGate = DegreesPerSecond.of(1.0);

  // Drive motor configuration
  /** Maximum current limit for the drive motors. */
  public static final Current driveMotorCurrentLimit = Amps.of(40);
  /** Secondary current limit for the drive motors. */
  public static final Current driveMotorSecondaryCurrentLimit = Amps.of(80);
  /** The radius of the drive wheels. */
  public static final Distance wheelRadius = Inches.of(1.5);
  /** The gear reduction between the drive motor and the wheel. */
  public static final double driveMotorReduction = 6.75;
  /** The gearbox model for the drive motor. */
  public static final DCMotor driveGearbox = DCMotor.getNEO(1);

  // Drive encoder configuration
  /** Conversion factor for drive position from motor rotations to meters. */
  public static final double driveEncoderPositionFactor = 2 * Math.PI / driveMotorReduction;
  /** Conversion factor for drive velocity from motor RPM to meters per second. */
  public static final double driveEncoderVelocityFactor =
      (2 * Math.PI) / 60.0 / driveMotorReduction;

  // Drive PID configuration (Real)
  public static final double driveKp = 0.01;
  public static final double driveKd = 0.0;
  public static final double driveKs = 0.0;
  public static final double driveKv = 0.1;

  // Drive PID configuration (Simulation)
  public static final double driveSimP = 0.05;
  public static final double driveSimD = 0.0;
  public static final double driveSimKs = 0.0;
  public static final double driveSimKv = 0.0789;

  // Turn motor configuration
  /** Whether the turn motor is inverted. */
  public static final boolean turnInverted = false;
  /** Maximum current limit for the turn motors. */
  public static final Current turnMotorCurrentLimit = Amps.of(40);
  /** Secondary current limit for the turn motors. */
  public static final Current turnMotorSecondaryCurrentLimit = Amps.of(80);
  /** The gear reduction between the turn motor and the module. */
  public static final double turnMotorReduction = 12.8;
  /** The gearbox model for the turn motor. */
  public static final DCMotor turnGearbox = DCMotor.getNEO(1);

  // Turn encoder configuration
  /** Whether the turn encoder is inverted. */
  public static final boolean turnEncoderInverted = false;
  /** Conversion factor for turn position from encoder rotations to radians. */
  public static final double turnEncoderPositionFactor = 2 * Math.PI; // Rotations -> Radians
  /** Conversion factor for turn velocity from encoder RPM to radians per second. */
  public static final double turnEncoderVelocityFactor = 2 * Math.PI; // Rotations/Sec -> Rad/Sec

  // Turn PID configuration (Real)
  public static final double turnKp = 0.4;
  public static final double turnKd = 0.01;

  // Turn PID configuration (Simulation)
  public static final double turnSimP = 8.0;
  public static final double turnSimD = 0.0;
  /** Minimum input range for turn PID (in radians). */
  public static final double turnPIDMinInput = -Math.PI;
  /** Maximum input range for turn PID (in radians). */
  public static final double turnPIDMaxInput = Math.PI;

  // PathPlanner configuration
  /** Total mass of the robot in kilograms. */
  public static final Mass robotMass = Kilograms.of(74.088);
  /** Moment of inertia of the robot in kilogram-square meters. */
  public static final MomentOfInertia robotMOI = KilogramSquareMeters.of(6.883);
  /** Coefficient of friction for the wheels. */
  public static final double wheelCOF = 1.1;
  /** PathPlanner configuration object. */
  public static final RobotConfig ppConfig =
      new RobotConfig(
          robotMass,
          robotMOI,
          new ModuleConfig(
              wheelRadius.in(Meters),
              maxLinearVelocity.in(MetersPerSecond),
              wheelCOF,
              driveGearbox.withReduction(driveMotorReduction),
              driveMotorCurrentLimit.in(Amps),
              1),
          moduleTranslations);
}
