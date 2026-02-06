// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Modified by FRC Team 4533 (Phoenix) 2026
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class DriveConstants {
  public static final double maxSpeedMetersPerSec = 4.8;
  public static final double odometryFrequency = 100.0; // Hz
  public static final double trackWidth = Units.inchesToMeters(26.5);
  public static final double wheelBase = Units.inchesToMeters(26.5);
  public static final double driveBaseRadius = Math.hypot(trackWidth / 2.0, wheelBase / 2.0);
  public static final Translation2d[] moduleTranslations =
      new Translation2d[] {
        new Translation2d(trackWidth / 2.0, wheelBase / 2.0),
        new Translation2d(trackWidth / 2.0, -wheelBase / 2.0),
        new Translation2d(-trackWidth / 2.0, wheelBase / 2.0),
        new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0)
      };

  // Zeroed rotation values for each module, see setup instructions
  public static final Rotation2d frontLeftZeroRotation = Rotation2d.fromDegrees(-144.49);
  public static final Rotation2d frontRightZeroRotation = Rotation2d.fromDegrees(113.03);
  public static final Rotation2d backLeftZeroRotation = Rotation2d.fromDegrees(-56.51);
  public static final Rotation2d backRightZeroRotation = Rotation2d.fromDegrees(-172.27);

  // Device CAN IDs
  public static final int imuCanId = 14;

  // Dual gyro parameters
  public static final double driftGain = 0.01;
  public static final double errorThresholdRad = Math.toRadians(0.5);
  public static final double maxCorrectionRadPerFrame = Math.toRadians(0.1);
  public static final double velocityGateRadPerSec = Math.toRadians(1.0);

  public static final int frontLeftDriveCanId = 2;
  public static final int backLeftDriveCanId = 8;
  public static final int frontRightDriveCanId = 5;
  public static final int backRightDriveCanId = 11;

  public static final int frontLeftTurnCanId = 3;
  public static final int backLeftTurnCanId = 9;
  public static final int frontRightTurnCanId = 6;
  public static final int backRightTurnCanId = 12;

  public static final int frontLeftEncoderCanId = 4;
  public static final int backLeftEncoderCanId = 10;
  public static final int frontRightEncoderCanId = 7;
  public static final int backRightEncoderCanId = 13;

  // Drive motor configuration
  public static final int driveMotorCurrentLimit = 40;
  public static final int driveMotorSecondaryCurrentLimit = 80;
  public static final double wheelRadiusMeters = Units.inchesToMeters(1.5);
  public static final double driveMotorReduction = 6.75; // Random BS values we got from somewhere
  public static final DCMotor driveGearbox = DCMotor.getNEO(1);

  // Drive encoder configuration
  public static final double driveEncoderPositionFactor =
      2 * Math.PI / driveMotorReduction; // Rotor Rotations ->
  // Wheel Radians
  public static final double driveEncoderVelocityFactor =
      (2 * Math.PI) / 60.0 / driveMotorReduction; // Rotor RPM ->
  // Wheel Rad/Sec

  // Drive PID configuration
  public static final double driveKp = 0.01;
  public static final double driveKd = 0.0;
  public static final double driveKs = 0.0;
  public static final double driveKv = 0.1;
  public static final double driveSimP = 0.05;
  public static final double driveSimD = 0.0;
  public static final double driveSimKs = 0.0;
  public static final double driveSimKv = 0.0789;

  // Turn motor configuration
  public static final boolean turnInverted = false;
  public static final int turnMotorCurrentLimit = 40;
  public static final int turnMotorSecondaryCurrentLimit = 80;
  public static final double turnMotorReduction = 12.8; // Random BS values we got from somewhere
  public static final DCMotor turnGearbox = DCMotor.getNEO(1);

  // Turn encoder configuration
  public static final boolean turnEncoderInverted = false;
  public static final double turnEncoderPositionFactor = 2 * Math.PI; // Rotations -> Radians
  public static final double turnEncoderVelocityFactor = 2 * Math.PI; // Rotations/Sec -> Rad/Sec

  // Turn PID configuration
  public static final double turnKp = 0.4;
  public static final double turnKd = 0.01;
  public static final double turnSimP = 8.0;
  public static final double turnSimD = 0.0;
  public static final double turnPIDMinInput = -Math.PI; // Radians
  public static final double turnPIDMaxInput = Math.PI; // Radians

  // PathPlanner configuration
  public static final double robotMassKg = 74.088; // TODO: Update this value
  public static final double robotMOI = 6.883;
  public static final double wheelCOF = 1.1; // TODO: Update this value
  public static final RobotConfig ppConfig =
      new RobotConfig(
          robotMassKg,
          robotMOI,
          new ModuleConfig(
              wheelRadiusMeters,
              maxSpeedMetersPerSec,
              wheelCOF,
              driveGearbox.withReduction(driveMotorReduction),
              driveMotorCurrentLimit,
              1),
          moduleTranslations);
}
