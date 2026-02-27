package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Distance;

public class ShooterKinematics {
  private static final InterpolatingDoubleTreeMap flywheelMap = new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap hoodMap = new InterpolatingDoubleTreeMap();

  static {
    // TODO: Tune these
    // flywheelMap.put(1.5, 50.0);
    // hoodMap.put(1.5, 45.0);

    // flywheelMap.put(3.0, 70.0);
    // hoodMap.put(3.0, 55.0);

    // flywheelMap.put(5.0, 90.0);
    // hoodMap.put(5.0, 65.0);

    flywheelMap.put(0.0, 50.0);
    hoodMap.put(0.0, 85.0);
  }

  /** Calculates the optimal Shooter State for a given distance to the speaker. */
  public static ShooterState calculateShooterState(Distance distanceToTarget) {
    double distMeters = distanceToTarget.in(Meters);

    return new ShooterState(
        RotationsPerSecond.of(flywheelMap.get(distMeters)), Degrees.of(hoodMap.get(distMeters)));
  }
}
