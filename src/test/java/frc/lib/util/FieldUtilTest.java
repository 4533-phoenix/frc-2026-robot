// Copyright (c) 2026 FRC Team 4533 (Phoenix)
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.lib.util;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class FieldUtilTest {

  @BeforeEach
  void setUp() {
    HAL.initialize(500, 0);
    // Reset driver station state before each test
    DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
    DriverStationSim.notifyNewData();
  }

  @AfterEach
  void tearDown() {
    DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
    DriverStationSim.notifyNewData();
  }

  @Test
  void testShouldFlipWhenBlue() {
    DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
    DriverStationSim.notifyNewData();
    assertFalse(FieldUtil.shouldFlip());
  }

  @Test
  void testShouldFlipWhenRed() {
    DriverStationSim.setAllianceStationId(AllianceStationID.Red1);
    DriverStationSim.notifyNewData();
    assertTrue(FieldUtil.shouldFlip());
  }

  @Test
  void testFlipAllianceTranslation() {
    Translation2d start = new Translation2d(2.0, 1.0);
    Translation2d flipped = FieldUtil.flipAlliance(start);
    assertEquals(
        FieldUtil.FIELD_LENGTH.in(edu.wpi.first.units.Units.Meters) - 2.0, flipped.getX(), 1e-6);
    assertEquals(
        FieldUtil.FIELD_WIDTH.in(edu.wpi.first.units.Units.Meters) - 1.0, flipped.getY(), 1e-6);
  }

  @Test
  void testFlipAllianceRotation() {
    Rotation2d start = Rotation2d.fromDegrees(45);
    Rotation2d flipped = FieldUtil.flipAlliance(start);
    assertEquals(-135.0, flipped.getDegrees(), 1e-6);
  }

  @Test
  void testFlipAlliancePose() {
    Pose2d start = new Pose2d(2.0, 1.0, Rotation2d.fromDegrees(45));
    Pose2d flipped = FieldUtil.flipAlliance(start);
    assertEquals(
        FieldUtil.FIELD_LENGTH.in(edu.wpi.first.units.Units.Meters) - 2.0, flipped.getX(), 1e-6);
    assertEquals(
        FieldUtil.FIELD_WIDTH.in(edu.wpi.first.units.Units.Meters) - 1.0, flipped.getY(), 1e-6);
    assertEquals(-135.0, flipped.getRotation().getDegrees(), 1e-6);
  }

  @Test
  void testFlipAllianceIfNeededWhenBlue() {
    DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
    DriverStationSim.notifyNewData();
    Translation2d start = new Translation2d(2.0, 1.0);
    Translation2d result = FieldUtil.flipAllianceIfNeeded(start);
    assertEquals(2.0, result.getX(), 1e-6);
    assertEquals(1.0, result.getY(), 1e-6);
  }

  @Test
  void testFlipAllianceIfNeededWhenRed() {
    DriverStationSim.setAllianceStationId(AllianceStationID.Red1);
    DriverStationSim.notifyNewData();
    Translation2d start = new Translation2d(2.0, 1.0);
    Translation2d result = FieldUtil.flipAllianceIfNeeded(start);
    assertEquals(
        FieldUtil.FIELD_LENGTH.in(edu.wpi.first.units.Units.Meters) - 2.0, result.getX(), 1e-6);
    assertEquals(
        FieldUtil.FIELD_WIDTH.in(edu.wpi.first.units.Units.Meters) - 1.0, result.getY(), 1e-6);
  }
}
