package frc.robot.util;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import java.util.function.Function;
import org.littletonrobotics.junction.Logger;

/** Utility class for calculating aiming solutions for the shooter. */
public class Aiming {

  /** Cached result of the aiming pipeline. */
  public record AimingResult(Rotation2d targetRotation, Distance distanceToTarget) {}

  /** Record for returning aiming result along with updated hysteresis state for lobbing. */
  public record LobAimingResult(AimingResult aimingResult, boolean lobTargetIsLeft) {}
  
  /**
   * Computes all aiming outputs for a direct hub shot.
   *
   * @param robotCenter The current center position of the robot.
   * @param currentRobotRotation The current rotation of the robot.
   * @param fieldVelocity The robot's field-relative velocity.
   * @param targetPosition The blue-alliance target position.
   * @param shooterRobotOffset The physical offset of the shooter from the robot's center.
   * @param estimatedTimeOfFlightSeconds Estimated time for the note to travel to the target.
   * @param log Whether to publish outputs to AdvantageKit for visualization.
   * @return The computed aiming result.
   */
  public static AimingResult computeHubAiming(
      Translation2d robotCenter,
      Rotation2d currentRobotRotation,
      Translation2d fieldVelocity,
      Translation2d targetPosition,
      Translation2d shooterRobotOffset,
      double estimatedTimeOfFlightSeconds,
      boolean log) {

    Translation2d targetTranslation = Util.flipAllianceIfNeeded(targetPosition);

    return calculateTwoPassAiming(
        robotCenter,
        currentRobotRotation,
        shooterRobotOffset,
        (shooterPos) -> {
          Translation2d lead =
              Util.calculateClampedLead(
                  shooterPos, targetTranslation, fieldVelocity, estimatedTimeOfFlightSeconds);
          return targetTranslation.minus(lead);
        },
        log);
  }

  /**
   * Computes aiming for lobbed shots.
   *
   * @param robotCenter The current center position of the robot.
   * @param currentRobotRotation The current rotation of the robot.
   * @param shooterRobotOffset The physical offset of the shooter from the robot's center.
   * @param lobTargetLeftCenter Blue alliance left lobbing target center.
   * @param lobTargetRightCenter Blue alliance right lobbing target center.
   * @param lobTargetHalfLengthMeters Half length of the lobbing target line segment.
   * @param lobTargetIsLeft Hysteresis state: true if currently targeting the left side.
   * @param log Whether to publish outputs to AdvantageKit for visualization.
   * @return The computed aiming result and updated hysteresis state.
   */
  public static LobAimingResult computeLobAiming(
      Translation2d robotCenter,
      Rotation2d currentRobotRotation,
      Translation2d shooterRobotOffset,
      Translation2d lobTargetLeftCenter,
      Translation2d lobTargetRightCenter,
      double lobTargetHalfLengthMeters,
      boolean lobTargetIsLeft,
      boolean log) {

    Translation2d leftCenter = Util.flipAllianceIfNeeded(lobTargetLeftCenter);
    Translation2d rightCenter = Util.flipAllianceIfNeeded(lobTargetRightCenter);
    Translation2d currentShooterPos =
        robotCenter.plus(shooterRobotOffset.rotateBy(currentRobotRotation));

    // Find closest point on each line segment to the current shooter position
    Translation2d closestLeft =
        Util.closestPointOnLobLine(currentShooterPos, leftCenter, lobTargetHalfLengthMeters);
    Translation2d closestRight =
        Util.closestPointOnLobLine(currentShooterPos, rightCenter, lobTargetHalfLengthMeters);

    double distLeft = currentShooterPos.getDistance(closestLeft);
    double distRight = currentShooterPos.getDistance(closestRight);

    // Only switch targets when the other is at least 0.5m closer
    double hysteresis = 0.5;
    boolean newLobTargetIsLeft = lobTargetIsLeft;
    if (newLobTargetIsLeft && distRight < distLeft - hysteresis) {
      newLobTargetIsLeft = false;
    } else if (!newLobTargetIsLeft && distLeft < distRight - hysteresis) {
      newLobTargetIsLeft = true;
    }

    final boolean targetIsLeftFinal = newLobTargetIsLeft;

    AimingResult result =
        calculateTwoPassAiming(
            robotCenter,
            currentRobotRotation,
            shooterRobotOffset,
            (shooterPos) -> {
              Translation2d lineCenter = targetIsLeftFinal ? leftCenter : rightCenter;
              return Util.closestPointOnLobLine(shooterPos, lineCenter, lobTargetHalfLengthMeters);
            },
            log);

    return new LobAimingResult(result, targetIsLeftFinal);
  }

  /**
   * Computes aiming outputs using a two-pass approach to account for the physical offset of the
   * shooter relative to the robot's center.
   *
   * @param robotCenter The current center position of the robot.
   * @param currentRobotRotation The current rotation of the robot.
   * @param shooterRobotOffset The physical offset of the shooter from the robot's center.
   * @param virtualTargetProvider A function that takes a shooter position and returns the target to
   *     aim at.
   * @param log Whether to publish outputs to AdvantageKit for visualization.
   * @return The computed aiming result.
   */
  public static AimingResult calculateTwoPassAiming(
      Translation2d robotCenter,
      Rotation2d currentRobotRotation,
      Translation2d shooterRobotOffset,
      Function<Translation2d, Translation2d> virtualTargetProvider,
      boolean log) {

    // Estimate rotation from current shooter position
    Translation2d currentShooterPos =
        robotCenter.plus(shooterRobotOffset.rotateBy(currentRobotRotation));
    Translation2d initialVirtualTarget = virtualTargetProvider.apply(currentShooterPos);
    Rotation2d estimatedRotation = initialVirtualTarget.minus(currentShooterPos).getAngle();

    // Predict shooter position at the estimated rotation and recompute
    Translation2d predictedShooterPos =
        robotCenter.plus(shooterRobotOffset.rotateBy(estimatedRotation));
    Translation2d finalVirtualTarget = virtualTargetProvider.apply(predictedShooterPos);

    // Final calculation
    Translation2d shooterToTarget = finalVirtualTarget.minus(predictedShooterPos);
    Rotation2d targetRotation = shooterToTarget.getAngle();
    double distanceMeters = shooterToTarget.getNorm();

    if (log) {
      Logger.recordOutput("Aiming/VirtualTarget", new Pose2d(finalVirtualTarget, Rotation2d.kZero));
      Logger.recordOutput(
          "Aiming/ShooterPosition", new Pose2d(currentShooterPos, currentRobotRotation));
      Logger.recordOutput(
          "Aiming/PredictedShooterPosition", new Pose2d(predictedShooterPos, estimatedRotation));
      Logger.recordOutput("Aiming/TargetRotation", targetRotation.getDegrees());
      Logger.recordOutput("Aiming/DistanceToTarget", distanceMeters);
    }

    return new AimingResult(targetRotation, Meters.of(distanceMeters));
  }
}
