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
