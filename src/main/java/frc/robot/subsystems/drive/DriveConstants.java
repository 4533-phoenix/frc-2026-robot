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

public class DriveConstants {
  public static final LinearVelocity maxLinearVelocity = MetersPerSecond.of(4.0);
  public static final Frequency odometryFrequency = Hertz.of(100);
  public static final Distance trackWidth = Inches.of(20.5);
  public static final Distance wheelBase = Inches.of(20.5);
  public static final Distance driveBaseRadius =
      Meters.of(Math.hypot(trackWidth.in(Meters) / 2.0, wheelBase.in(Meters) / 2.0));
  public static final Translation2d[] moduleTranslations =
      new Translation2d[] {
        new Translation2d(trackWidth.in(Meters) / 2.0, wheelBase.in(Meters) / 2.0),
        new Translation2d(trackWidth.in(Meters) / 2.0, -wheelBase.in(Meters) / 2.0),
        new Translation2d(-trackWidth.in(Meters) / 2.0, wheelBase.in(Meters) / 2.0),
        new Translation2d(-trackWidth.in(Meters) / 2.0, -wheelBase.in(Meters) / 2.0)
      };

  // Zeroed rotation values for each module
  public static final Rotation2d frontLeftZeroRotation = Rotation2d.fromDegrees(35.51);
  public static final Rotation2d frontRightZeroRotation = Rotation2d.fromDegrees(293.03);
  public static final Rotation2d backLeftZeroRotation = Rotation2d.fromDegrees(123.49);
  public static final Rotation2d backRightZeroRotation = Rotation2d.fromDegrees(7.73);

  // Device CAN IDs
  public static final int imuCanId = 14;

  // Dual gyro parameters
  public static final double driftGain = 0.01;
  public static final Angle errorThreshold = Degrees.of(0.5);
  public static final Angle maxCorrectionPerFrame = Degrees.of(0.1);
  public static final AngularVelocity velocityGate = DegreesPerSecond.of(1.0);

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
  public static final Distance wheelRadius = Inches.of(1.5);
  public static final double driveMotorReduction = 6.75; // Random BS values we got from somewhere
  public static final DCMotor driveGearbox = DCMotor.getNEO(1);

  // Drive encoder configuration
  public static final double driveEncoderPositionFactor = 2 * Math.PI / driveMotorReduction;
  public static final double driveEncoderVelocityFactor =
      (2 * Math.PI) / 60.0 / driveMotorReduction;

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
  public static final Current turnMotorCurrentLimit = Amps.of(40);
  public static final Current turnMotorSecondaryCurrentLimit = Amps.of(80);
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

  // PathPlanner configuration (TODO: Tune these values)
  public static final Mass robotMass = Kilograms.of(74.088);
  public static final MomentOfInertia robotMOI = KilogramSquareMeters.of(6.883);
  public static final double wheelCOF = 1.1;
  public static final RobotConfig ppConfig =
      new RobotConfig(
          robotMass,
          robotMOI,
          new ModuleConfig(
              wheelRadius.in(Meters),
              maxLinearVelocity.in(MetersPerSecond),
              wheelCOF,
              driveGearbox.withReduction(driveMotorReduction),
              driveMotorCurrentLimit,
              1),
          moduleTranslations);
}
