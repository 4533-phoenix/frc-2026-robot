// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Modified by FRC Team 4533 (Phoenix) 2026
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.lib;

import static edu.wpi.first.units.Units.Meters;
import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.lib.util.FieldUtil;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

public class FieldUtilTest {

  @BeforeAll
  public static void setup() {
    // Initialize the WPILib Hardware Abstraction Layer (HAL)
    // This prevents wpiHal.dll from crashing when using DriverStation, Timers, or Alerts
    assert HAL.initialize(500, 0);
  }

  @Test
  public void testFlipAllianceTranslation() {
    // Assuming field is 16.54m long and 8.07m wide
    Translation2d blueTranslation = new Translation2d(2.0, 3.0);
    Translation2d flipped = FieldUtil.flipAlliance(blueTranslation);

    // X should be mirrored across the center of the field length
    assertEquals(FieldUtil.FIELD_LENGTH.in(Meters) - 2.0, flipped.getX(), 1e-6);
    // Y should be mirrored across the center of the field width
    assertEquals(FieldUtil.FIELD_WIDTH.in(Meters) - 3.0, flipped.getY(), 1e-6);
  }

  @Test
  public void testFlipAllianceRotation() {
    Rotation2d facingRight = Rotation2d.fromDegrees(0);
    Rotation2d flipped = FieldUtil.flipAlliance(facingRight);

    // Flipped rotation should face completely opposite (180 degrees)
    assertEquals(180.0, flipped.getDegrees(), 1e-6);
  }
}
