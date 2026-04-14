// Copyright (c) 2026 FRC Team 4533 (Phoenix)
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterState;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.flywheel.FlywheelIO;
import frc.robot.subsystems.shooter.hood.HoodIO;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

public class ShooterTest {

  @BeforeAll
  public static void setup() {
    assert HAL.initialize(500, 0);
  }

  private static class DummyHoodIO implements HoodIO {
    public Distance lastCommandedLength = Inches.of(0);

    @Override
    public void setLength(Distance length) {
      lastCommandedLength = length;
    }
  }

  private static class DummyFlywheelIO implements FlywheelIO {
    public AngularVelocity lastCommandedVelocity = RadiansPerSecond.of(0);

    @Override
    public void setAngularVelocity(AngularVelocity velocity) {
      lastCommandedVelocity = velocity;
    }
  }

  @Test
  public void testLawOfCosinesServoBoundsAndFlywheelSpeed() {
    DummyFlywheelIO flywheelIO = new DummyFlywheelIO();
    DummyHoodIO hoodIO = new DummyHoodIO();
    Shooter shooter = new Shooter(flywheelIO, hoodIO);

    // Test a steep shot (e.g. 50 RPS, 85 degrees)
    shooter.setShooterState(new ShooterState(RotationsPerSecond.of(50), Degrees.of(85)));
    shooter.run().initialize();
    shooter.periodic();

    double length85 = hoodIO.lastCommandedLength.in(Inches);
    double speed85 = flywheelIO.lastCommandedVelocity.in(RotationsPerSecond);

    // Check that the flywheel was commanded to the correct speed
    assertEquals(50.0, speed85, 1e-6, "Flywheel speed was not commanded correctly for steep shot!");

    // Test a shallow shot (e.g. 60 RPS, 40 degrees)
    shooter.setShooterState(new ShooterState(RotationsPerSecond.of(60), Degrees.of(40)));
    shooter.periodic();

    double length40 = hoodIO.lastCommandedLength.in(Inches);
    double speed40 = flywheelIO.lastCommandedVelocity.in(RotationsPerSecond);

    // Check that the flywheel was commanded to the correct new speed
    assertEquals(
        60.0, speed40, 1e-6, "Flywheel speed was not commanded correctly for shallow shot!");

    // Ensure the calculated lengths NEVER exceed the physical limits of the servo
    assertTrue(
        length85 >= ShooterConstants.SERVO_MIN_LENGTH.in(Inches),
        "Steep shot violated min servo limit!");
    assertTrue(
        length85 <= ShooterConstants.SERVO_MAX_LENGTH.in(Inches),
        "Steep shot violated max servo limit!");

    assertTrue(
        length40 >= ShooterConstants.SERVO_MIN_LENGTH.in(Inches),
        "Shallow shot violated min servo limit!");
    assertTrue(
        length40 <= ShooterConstants.SERVO_MAX_LENGTH.in(Inches),
        "Shallow shot violated max servo limit!");

    // A steeper launch angle should require a different servo extension than a shallow one.
    assertNotEquals(length85, length40, 0.01, "Servo length did not change for different angles!");
  }

  @Test
  public void testSetShooterStateNullSafety() {
    DummyFlywheelIO flywheelIO = new DummyFlywheelIO();
    DummyHoodIO hoodIO = new DummyHoodIO();
    Shooter shooter = new Shooter(flywheelIO, hoodIO);

    // Set a valid state first
    shooter.setShooterState(new ShooterState(RotationsPerSecond.of(50), Degrees.of(85)));
    shooter.run().initialize();
    shooter.periodic();

    // Now set the state to null and ensure it defaults to the safe state
    shooter.setShooterState(null);
    shooter.periodic();

    double speedAfterNull = flywheelIO.lastCommandedVelocity.in(RotationsPerSecond);
    double lengthAfterNull = hoodIO.lastCommandedLength.in(Inches);

    // The flywheel should be commanded to 0 RPS and the hood should retract to the default position
    assertEquals(
        0.0, speedAfterNull, 1e-6, "Flywheel speed was not set to 0 when null state was provided!");
    assertEquals(
        ShooterConstants.SERVO_MIN_LENGTH.in(Inches),
        lengthAfterNull,
        1e-6,
        "Hood length was not set to default when null state was provided!");
  }
}
