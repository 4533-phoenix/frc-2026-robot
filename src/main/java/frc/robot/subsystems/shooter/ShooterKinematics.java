package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Distance;

/**
 * Calculates optimal shooter settings based on the distance to the target.
 *
 * <p>Uses {@link InterpolatingDoubleTreeMap} to interpolate between known good shooter states
 * (flywheel speed and hood angle) for specific distances, ensuring smooth transitions as the robot
 * moves.
 */
public class ShooterKinematics {
  // Maps to store calibrated distances and corresponding motor speeds/angles
  private static final InterpolatingDoubleTreeMap flywheelMap = new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap hoodMap = new InterpolatingDoubleTreeMap();

  static {
    // Rough shooter tune (342 Scrimage 2/28/26)
    flywheelMap.put(2.159, 50.0);
    hoodMap.put(2.159, 85.0);

    flywheelMap.put(3.02895, 75.0);
    hoodMap.put(3.02895, 85.0);

    flywheelMap.put(4.0513, 100.0);
    hoodMap.put(4.0513, 55.0);
  }

  /**
   * Calculates the optimal {@link ShooterState} for a given distance to the target.
   *
   * @param distanceToTarget The distance from the shooter to the target.
   * @return The calculated ShooterState containing target flywheel speed and hood angle.
   */
  public static ShooterState calculateShooterState(Distance distanceToTarget) {
    double distMeters = distanceToTarget.in(Meters);

    // Get interpolated values from the maps based on current distance
    return new ShooterState(
        RotationsPerSecond.of(flywheelMap.get(distMeters)), Degrees.of(hoodMap.get(distMeters)));
  }
}
