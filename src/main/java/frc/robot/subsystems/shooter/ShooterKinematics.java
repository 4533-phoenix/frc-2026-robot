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

  /** Slope for flywheel speed calculation. */
  public static final double FLYWHEEL_SLOPE = 11.4894;

  /** Intercept for flywheel speed calculation. */
  public static final double FLYWHEEL_INTERCEPT = 19.6636;

  /** Slope for time-of-flight calculation. */
  public static final double TOF_SLOPE = 0.3223;

  /** Intercept for time-of-flight calculation. */
  public static final double TOF_INTERCEPT = 0.3617;

  /** Default hood angle for all shots. */
  public static final double DEFAULT_HOOD_ANGLE = 85.0;

  /**
   * Calculates the optimal {@link ShooterState} for a given distance to the target.
   *
   * @param distanceToTarget The distance from the shooter to the target.
   * @return The calculated ShooterState containing target flywheel speed and hood angle.
   */
  public static ShooterState calculateShooterState(Distance distanceToTarget) {
    double targetRps = (FLYWHEEL_SLOPE * distanceToTarget.in(Meters)) + FLYWHEEL_INTERCEPT;

    return new ShooterState(RotationsPerSecond.of(targetRps), Degrees.of(DEFAULT_HOOD_ANGLE));
  }

  /**
   * Estimates the time of flight (TOF) for a given distance to the target.
   *
   * @param distanceToTarget The distance from the shooter to the target.
   * @return The estimated Time of Flight in Seconds.
   */
  public static Time estimateTOF(Distance distanceToTarget) {
    double estimatedTof = (TOF_SLOPE * distanceToTarget.in(Meters)) + TOF_INTERCEPT;

    return Seconds.of(estimatedTof);
  }
}
