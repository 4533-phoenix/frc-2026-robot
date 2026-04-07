package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import frc.robot.subsystems.shooter.Shooter.ShooterState;

/**
 * Calculates optimal shooter settings based on the distance to the target.
 *
 * <p>Uses linear regression formulas derived from calibrated data to provide smooth, continuous
 * target states for the flywheel and time-of-flight estimation.
 */
public class ShooterKinematics {
  // Tunable Constants
  // Formula: RPS = (SLOPE * distance) + INTERCEPT
  public static final double FLYWHEEL_SLOPE = 11.4894;
  public static final double FLYWHEEL_INTERCEPT = 18.6636;

  // Formula: TOF = (SLOPE * distance) + INTERCEPT
  public static final double TOF_SLOPE = 0.3223;
  public static final double TOF_INTERCEPT = 0.3617;

  // Constant hood angle (as per your current map)
  public static final double DEFAULT_HOOD_ANGLE = 85.0;

  /**
   * Calculates the optimal {@link ShooterState} for a given distance to the target.
   *
   * @param distanceToTarget The distance from the shooter to the target.
   * @return The calculated ShooterState containing target flywheel speed and hood angle.
   */
  public static ShooterState calculateShooterState(Distance distanceToTarget) {
    double distMeters = distanceToTarget.in(Meters);

    double targetRps = (FLYWHEEL_SLOPE * distMeters) + FLYWHEEL_INTERCEPT;

    return new ShooterState(RotationsPerSecond.of(targetRps), Degrees.of(DEFAULT_HOOD_ANGLE));
  }

  /**
   * Estimates the time of flight (TOF) for a given distance to the target.
   *
   * @param distanceToTarget The distance from the shooter to the target.
   * @return The estimated Time of Flight in Seconds.
   */
  public static Time estimateTOF(Distance distanceToTarget) {
    double distMeters = distanceToTarget.in(Meters);
    double estimatedTof = (TOF_SLOPE * distMeters) + TOF_INTERCEPT;

    return Seconds.of(estimatedTof);
  }
}
