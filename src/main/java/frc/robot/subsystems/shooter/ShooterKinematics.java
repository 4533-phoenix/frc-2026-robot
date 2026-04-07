package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
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
  private static final InterpolatingDoubleTreeMap tofMap = new InterpolatingDoubleTreeMap();

  static {
    flywheelMap.put(1.672, 42.0);
    flywheelMap.put(1.891, 42.0);
    flywheelMap.put(2.152, 43.0);
    flywheelMap.put(2.315, 44.0);
    flywheelMap.put(2.499, 46.5);
    flywheelMap.put(2.713, 48.0);
    flywheelMap.put(2.713, 48.0);
    flywheelMap.put(3.003, 52.0);
    flywheelMap.put(3.211, 54.0);
    flywheelMap.put(3.430, 56.0);
    flywheelMap.put(3.560, 58.0);
    flywheelMap.put(3.746, 62.0);
    flywheelMap.put(4.018, 66.0);
    flywheelMap.put(4.343, 69.0);
    flywheelMap.put(4.520, 71.0);
    flywheelMap.put(4.667, 74.5);
    flywheelMap.put(4.951, 76.0);
    flywheelMap.put(4.951, 76.0);
    flywheelMap.put(5.251, 79.0);

    hoodMap.put(0.0, 85.0);

    tofMap.put(1.672, 0.901);
    tofMap.put(1.891, 0.971);
    tofMap.put(2.152, 1.055);
    tofMap.put(2.315, 1.108);
    tofMap.put(2.499, 1.167);
    tofMap.put(2.713, 1.236);
    tofMap.put(3.003, 1.330);
    tofMap.put(3.211, 1.397);
    tofMap.put(3.430, 1.467);
    tofMap.put(3.560, 1.509);
    tofMap.put(3.746, 1.569);
    tofMap.put(4.018, 1.657);
    tofMap.put(4.343, 1.762);
    tofMap.put(4.520, 1.819);
    tofMap.put(4.667, 1.866);
    tofMap.put(4.951, 1.957);
    tofMap.put(5.251, 2.054);
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

  /**
   * Estimates the time of flight (TOF) for a given distance to the target.
   *
   * @param distanceToTarget The distance from the shooter to the target.
   * @return The estimated Time of Flight in Seconds.
   */
  public static Time estimateTOF(Distance distanceToTarget) {
    return Seconds.of(tofMap.get(distanceToTarget.in(Meters)));
  }
}
