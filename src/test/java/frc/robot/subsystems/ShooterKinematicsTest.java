// Copyright (c) 2026 FRC Team 4533 (Phoenix)
//
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import static org.junit.jupiter.api.Assertions.*;

import frc.robot.subsystems.shooter.Shooter.ShooterState;
import frc.robot.subsystems.shooter.ShooterKinematics;
import org.junit.jupiter.api.Test;

public class ShooterKinematicsTest {
  // Delta for floating point comparisons
  private static final double kDelta = 1e-6;

  @Test
  public void testShooterStateLogic() {
    // Test a known distance (e.g., 2.0 meters)
    // RPS = (11.4894 * 2.0) + 18.6636 = 41.6424
    double distance = 2.0;
    double expectedRps =
        (ShooterKinematics.FLYWHEEL_SLOPE * distance) + ShooterKinematics.FLYWHEEL_INTERCEPT;

    ShooterState state = ShooterKinematics.calculateShooterState(Meters.of(distance));

    assertEquals(expectedRps, state.flywheelSpeed().in(RotationsPerSecond), kDelta);
    assertEquals(ShooterKinematics.DEFAULT_HOOD_ANGLE, state.hoodAngle().in(Degrees), kDelta);
  }

  @Test
  public void testTOFLogic() {
    // Test a known distance (e.g., 4.0 meters)
    // TOF = (0.3223 * 4.0) + 0.3617 = 1.6509
    double distance = 4.0;
    double expectedTof = (ShooterKinematics.TOF_SLOPE * distance) + ShooterKinematics.TOF_INTERCEPT;

    double calculatedTof = ShooterKinematics.estimateTOF(Meters.of(distance)).in(Seconds);

    assertEquals(expectedTof, calculatedTof, kDelta);
  }

  @Test
  public void testLinearity() {
    // Ensure that moving from 1m to 2m results in exactly the Slope's worth of increase
    double dist1 = 1.0;
    double dist2 = 2.0;

    double rps1 =
        ShooterKinematics.calculateShooterState(Meters.of(dist1))
            .flywheelSpeed()
            .in(RotationsPerSecond);
    double rps2 =
        ShooterKinematics.calculateShooterState(Meters.of(dist2))
            .flywheelSpeed()
            .in(RotationsPerSecond);

    assertEquals(ShooterKinematics.FLYWHEEL_SLOPE, rps2 - rps1, kDelta);
  }
}
