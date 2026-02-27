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
    // TODO: Tune these calibration points based on real robot data
    // Example format:
    // flywheelMap.put(distanceInMeters, flywheelSpeedInRotationsPerSecond);
    // hoodMap.put(distanceInMeters, hoodAngleInDegrees);

    // Initial dummy values
    flywheelMap.put(0.0, 50.0);
    hoodMap.put(0.0, 85.0);
  }

  /**
   * Calculates the optimal {@link ShooterState} for a given distance to the speaker.
   *
   * @param distanceToTarget The distance from the robot to the target in meters.
   * @return The calculated ShooterState containing target flywheel speed and hood angle.
   */
  public static ShooterState calculateShooterState(Distance distanceToTarget) {
    double distMeters = distanceToTarget.in(Meters);

    // Get interpolated values from the maps based on current distance
    return new ShooterState(
        RotationsPerSecond.of(flywheelMap.get(distMeters)), 
        Degrees.of(hoodMap.get(distMeters)));
  }
}
