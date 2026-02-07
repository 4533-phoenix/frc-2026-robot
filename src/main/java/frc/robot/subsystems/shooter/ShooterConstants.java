// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.system.plant.DCMotor;

public final class ShooterConstants {
  public static final int flywheelMotorId = 20;
  public static final int indexerMotorId = 21;
  public static final int hoodServoChannel = 0;

  public static final double flywheelReduction = 1.0;
  public static final double flywheelMOI = 0.0042;

  public static final int indexerMotorCurrentLimit = 20;
  public static final double indexerOnVoltage = 6.0;

  public static final double flywheelMotorCurrentLimit = 40.0;

  public static final DCMotor flywheelGearbox = DCMotor.getFalcon500(1);
  public static final DCMotor indexerGearbox = DCMotor.getNeo550(1);

  // PID constants for Flywheel velocity control
  public static final double flywheelKp = 0.1;
  public static final double flywheelKi = 0.0;
  public static final double flywheelKd = 0.0;
  public static final double flywheelKs = 0.0;
  public static final double flywheelKv = 0.11;
}
