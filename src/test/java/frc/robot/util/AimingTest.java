// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Modified by FRC Team 4533 (Phoenix) 2026
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.util;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.geometry.Translation2d;
import org.junit.jupiter.api.Test;

public class AimingTest {

  @Test
  public void testCalculateClampedLead_NormalVelocity() {
    Translation2d shooterPos = new Translation2d(0, 0);
    Translation2d targetPos = new Translation2d(10, 0); // 10 meters away
    Translation2d robotVelocity = new Translation2d(2, 0); // moving 2 m/s towards target
    double timeOfFlight = 1.0;

    Translation2d lead =
        Util.calculateClampedLead(shooterPos, targetPos, robotVelocity, timeOfFlight);

    // Normal lead: 2 m/s * 1.0 s = 2.0 meters
    assertEquals(2.0, lead.getX(), 1e-6);
    assertEquals(0.0, lead.getY(), 1e-6);
  }

  @Test
  public void testCalculateClampedLead_ExceedsClamp() {
    Translation2d shooterPos = new Translation2d(0, 0);
    Translation2d targetPos = new Translation2d(4, 0); // 4 meters away (Max lead allowed = 2m)
    Translation2d robotVelocity = new Translation2d(0, 5); // moving 5 m/s sideways
    double timeOfFlight = 1.0;

    Translation2d lead =
        Util.calculateClampedLead(shooterPos, targetPos, robotVelocity, timeOfFlight);

    // Raw lead is (0, 5). Max lead is 4 * 0.5 = 2.0.
    // It should be scaled down to magnitude 2.0
    assertEquals(0.0, lead.getX(), 1e-6);
    assertEquals(2.0, lead.getY(), 1e-6);
    assertEquals(2.0, lead.getNorm(), 1e-6);
  }

  @Test
  public void testClosestPointOnLobLine() {
    Translation2d center = new Translation2d(5.0, 5.0);
    double halfLen = 1.0; // Line spans from Y=4 to Y=6 at X=5

    // Test a point above the line
    Translation2d pointAbove = new Translation2d(0, 8.0);
    Translation2d closestAbove = Util.closestPointOnLobLine(pointAbove, center, halfLen);
    assertEquals(5.0, closestAbove.getX(), 1e-6);
    assertEquals(6.0, closestAbove.getY(), 1e-6); // Clamped to max Y

    // Test a point aligned with the line
    Translation2d pointBeside = new Translation2d(0, 4.5);
    Translation2d closestBeside = Util.closestPointOnLobLine(pointBeside, center, halfLen);
    assertEquals(5.0, closestBeside.getX(), 1e-6);
    assertEquals(4.5, closestBeside.getY(), 1e-6); // Y remains the same
  }
}
