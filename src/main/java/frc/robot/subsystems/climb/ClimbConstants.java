// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.climb;

import edu.wpi.first.math.system.plant.DCMotor;

/** Hardware and tuning constants for the climb subsystem. */
public final class ClimbConstants {
  // CAN IDs
  public static final int liftMotorCanId = 14;
  public static final int rotateMotorCanId = 16;

  // Motor and current limits
  public static final int liftMotorCurrentLimit = 30;
  public static final int rotateMotorCurrentLimit = 30;
  public static final DCMotor liftGearbox = DCMotor.getNEO(2);
  public static final DCMotor rotateGearbox = DCMotor.getNEO(2);

  public static final double defaultLiftVoltage = 6.0;
  public static final double defaultRotateVoltage = 6.0;
}
