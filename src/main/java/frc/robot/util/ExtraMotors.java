// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.util;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.system.plant.DCMotor;

public final class ExtraMotors {
  public static DCMotor getSnowBlower(int numMotors) {
    return new DCMotor(
        12.0, 7.91, 24.0, 5.0, RotationsPerSecond.of(100).in(RadiansPerSecond), numMotors);
  }
}
