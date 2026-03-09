// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Modified by FRC Team 4533 (Phoenix) 2026
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import static org.junit.jupiter.api.Assertions.*;

import frc.robot.subsystems.shooter.ShooterKinematics;
import frc.robot.subsystems.shooter.ShooterState;
import org.junit.jupiter.api.Test;

public class ShooterKinematicsTest {

  @Test
  public void testExtrapolationBounds() {
    // If the robot is closer than the closest map point,
    // it should flatten out and return the 2.159m values, NOT extrapolate linearly to 0.
    ShooterState tooClose = ShooterKinematics.calculateShooterState(Meters.of(1.0));
    assertEquals(50.0, tooClose.flywheelSpeed().in(RotationsPerSecond), 1e-6);

    // If the robot is further than the furthest map point,
    // it should return the max distance values.
    ShooterState tooFar = ShooterKinematics.calculateShooterState(Meters.of(10.0));
    assertEquals(100.0, tooFar.flywheelSpeed().in(RotationsPerSecond), 1e-6);
    assertEquals(55.0, tooFar.hoodAngle().in(Degrees), 1e-6);
  }
}
