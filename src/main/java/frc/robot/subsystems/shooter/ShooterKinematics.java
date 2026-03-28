package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Distance;
import frc.robot.subsystems.shooter.Shooter.ShooterState;

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
    flywheelMap.put(1.672, 42.0);
    flywheelMap.put(1.912, 43.0);
    flywheelMap.put(2.130, 45.0);
    flywheelMap.put(2.423, 47.0);
    flywheelMap.put(2.575, 49.0);
    flywheelMap.put(2.779, 51.0);
    flywheelMap.put(3.112, 54.0);
    flywheelMap.put(3.335, 58.0);
    flywheelMap.put(3.641, 62.0);

    hoodMap.put(1.672, 85.0);
    hoodMap.put(1.912, 85.0);
    hoodMap.put(2.130, 85.0);
    hoodMap.put(2.423, 85.0);
    hoodMap.put(2.575, 85.0);
    hoodMap.put(2.779, 85.0);
    hoodMap.put(3.112, 85.0);
    hoodMap.put(3.335, 85.0);
    hoodMap.put(3.641, 70.0);
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
