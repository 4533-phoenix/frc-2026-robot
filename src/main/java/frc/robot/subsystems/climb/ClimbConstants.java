// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.ExtraMotors;

/** Hardware and tuning constants for the climb subsystem. */
public final class ClimbConstants {
  // CAN IDs
  public static final int liftMotorCanId = 60;

  // Motor and current limits
  public static final Current liftMotorCurrentLimit = Amps.of(30);
  public static final DCMotor liftGearbox = ExtraMotors.getSnowBlower(1);

  public static final Voltage defaultLiftVoltage = Volts.of(6.0);
}
