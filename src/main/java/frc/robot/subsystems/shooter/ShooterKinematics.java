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
    flywheelMap.put(1.307, 45.0);
    flywheelMap.put(1.734, 45.0);
    flywheelMap.put(2.459, 53.0);
    flywheelMap.put(2.666, 58.0);
    flywheelMap.put(3.155, 67.0);

    hoodMap.put(1.307, 85.0);
    hoodMap.put(1.734, 85.0);
    hoodMap.put(2.459, 85.0);
    hoodMap.put(2.666, 85.0);
    hoodMap.put(3.155, 62.0);
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
