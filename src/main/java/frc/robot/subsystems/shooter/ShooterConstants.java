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
  public static final int hoodServoChannel = 0;

  public static final double flywheelReduction = 1.0;
  public static final double flywheelMomentOfInertia = 0.01;

  public static final DCMotor flywheelGearbox = DCMotor.getFalcon500(1);

  public static final double defaultFlywheelVoltage = 8.0;
  public static final double hoodExtendPosition = 0.8;
  public static final double hoodRetractPosition = 0.2;
}
