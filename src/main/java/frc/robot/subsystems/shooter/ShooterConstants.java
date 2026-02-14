// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.*;

public final class ShooterConstants {
  public static final int flywheelMotorId = 18;
  public static final int indexerMotorId = 17;
  public static final int hoodServoChannel = 0;

  public static final Distance crankArmLength = Inches.of(6.403);
  public static final Distance groundLinkDistance = Inches.of(7.521);
  public static final Distance servoMinLength = Inches.of(6.925);
  public static final Distance servoMaxLength = Inches.of(10.5);
  public static final Angle crankTangentToLaunchAngle = Degrees.of(12.875);
  public static final Angle mechanismTotalAngle = Degrees.of(149.007);
  public static final LinearVelocity maxServoVelocity = MetersPerSecond.of(0.02);

  public static final double flywheelReduction = 1.0;
  public static final MomentOfInertia flywheelMOI = KilogramSquareMeters.of(0.0042);
  public static final Distance flywheelWheelRadius = Inches.of(2.05);

  public static final Current indexerMotorCurrentLimit = Amps.of(20.0);
  public static final Voltage indexerOnVoltage = Volts.of(3.0);
  public static final Voltage indexerOffVoltage = Volts.of(0.0);
  public static final MomentOfInertia indexerMOI = KilogramSquareMeters.of(0.0005);
  public static final double indexerReduction = 1.0;

  public static final Current flywheelMotorCurrentLimit = Amps.of(60.0);

  public static final DCMotor flywheelGearbox = DCMotor.getFalcon500(1);
  public static final DCMotor indexerGearbox = DCMotor.getNeo550(1);

  // PID constants for Flywheel velocity control
  public static final double flywheelKp = 0.25;
  public static final double flywheelKi = 0.0;
  public static final double flywheelKd = 0.0;
  public static final double flywheelKs = 0.2;
  public static final double flywheelKv = 0.113;
  public static final double flywheelKa = 0.04;

  // New simple control limits for operator bindings
  // Flywheel target range in rotations per second (rps)
  public static final AngularVelocity flywheelMinRps = RotationsPerSecond.of(0.0);
  public static final AngularVelocity flywheelMaxRps = RotationsPerSecond.of(100.0);
}
