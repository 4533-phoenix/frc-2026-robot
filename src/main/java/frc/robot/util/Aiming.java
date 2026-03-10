// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.util;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import java.util.function.Function;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/** Utility class for calculating aiming solutions for the shooter. */
public class Aiming {
  private Aiming() {} // Prevent instantiation

  /** Cached result of the aiming pipeline. */
  public record AimingResult(
      Rotation2d targetRotation, Distance distanceToTarget, boolean hasTarget) {}

  /** A default empty result to prevent NullPointerExceptions */
  public static final AimingResult noTarget =
      new AimingResult(new Rotation2d(), Meters.of(0), false);

  /**
   * Computes all aiming outputs for a direct hub shot.
   *
   * @param robotCenter The current center position of the robot.
   * @param currentRobotRotation The current rotation of the robot.
   * @param fieldVelocity The robot's field-relative velocity.
   * @param targetPosition The blue-alliance target position.
   * @param shooterRobotOffset The physical offset of the shooter from the robot's center.
   * @param estimatedTimeOfFlight Estimated time for the note to travel to the target.
   * @param log Whether to publish outputs to AdvantageKit for visualization.
   * @return The computed aiming result.
   */
  public static AimingResult computeHubAiming(
      Translation2d robotCenter,
      Rotation2d currentRobotRotation,
      Translation2d fieldVelocity,
      Translation2d targetPosition,
      Translation2d shooterRobotOffset,
      Time estimatedTimeOfFlight,
      boolean log) {

    Translation2d targetTranslation = Util.flipAllianceIfNeeded(targetPosition);

    return calculateTwoPassAiming(
        robotCenter,
        currentRobotRotation,
        shooterRobotOffset,
        (shooterPos) -> {
          Translation2d lead =
              Util.calculateClampedLead(
                  shooterPos, targetTranslation, fieldVelocity, estimatedTimeOfFlight.in(Seconds));
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
   * @param lobTargetHalfLength Half length of the lobbing target line segment.
   * @param log Whether to publish outputs to AdvantageKit for visualization.
   * @return The computed aiming result.
   */
  public static AimingResult computeLobAiming(
      Translation2d robotCenter,
      Rotation2d currentRobotRotation,
      Translation2d shooterRobotOffset,
      Translation2d lobTargetLeftCenter,
      Translation2d lobTargetRightCenter,
      Distance lobTargetHalfLength,
      boolean log) {

    Translation2d leftCenter = Util.flipAllianceIfNeeded(lobTargetLeftCenter);
    Translation2d rightCenter = Util.flipAllianceIfNeeded(lobTargetRightCenter);
    Translation2d currentShooterPos =
        robotCenter.plus(shooterRobotOffset.rotateBy(currentRobotRotation));

    // Find closest point on each line segment to the current shooter position
    Translation2d closestLeft =
        Util.closestPointOnLobLine(currentShooterPos, leftCenter, lobTargetHalfLength.in(Meters));
    Translation2d closestRight =
        Util.closestPointOnLobLine(currentShooterPos, rightCenter, lobTargetHalfLength.in(Meters));

    double distLeft = currentShooterPos.getDistance(closestLeft);
    double distRight = currentShooterPos.getDistance(closestRight);

    final Translation2d closestLineCenter = distLeft < distRight ? leftCenter : rightCenter;

    return calculateTwoPassAiming(
        robotCenter,
        currentRobotRotation,
        shooterRobotOffset,
        (shooterPos) -> {
          return Util.closestPointOnLobLine(
              shooterPos, closestLineCenter, lobTargetHalfLength.in(Meters));
        },
        log);
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

    // Pass true for hasTarget
    return new AimingResult(targetRotation, Meters.of(distanceMeters), true);
  }

  /**
   * Creates a supplier that calculates aiming results for the central hub. The results are cached
   * per control loop cycle to optimize performance.
   *
   * @param robotPoseSupplier A supplier for the robot's current {@link Pose2d}.
   * @param fieldVelocitySupplier A supplier for the robot's field-relative velocity.
   * @param shooterRobotOffset The physical offset of the shooter from the robot's center.
   * @param estimatedTimeOfFlight The estimated time for the projectile to reach the target.
   * @return A supplier that provides the calculated {@link AimingResult} for the hub.
   */
  public static Supplier<AimingResult> hubAimingSupplier(
      Supplier<Pose2d> robotPoseSupplier,
      Supplier<Translation2d> fieldVelocitySupplier,
      Translation2d shooterRobotOffset,
      Time estimatedTimeOfFlight) {
    return new Supplier<AimingResult>() {
      private AimingResult lastResult = noTarget;
      private double lastTimestamp = -1.0;

      @Override
      public AimingResult get() {
        double currentTime = Math.round(Timer.getFPGATimestamp() * 50.0) / 50.0;

        if (currentTime != lastTimestamp) {
          Pose2d robotPose = robotPoseSupplier.get();
          lastResult =
              computeHubAiming(
                  robotPose.getTranslation(),
                  robotPose.getRotation(),
                  fieldVelocitySupplier.get(),
                  Constants.hubPosition,
                  shooterRobotOffset,
                  estimatedTimeOfFlight,
                  true);
          lastTimestamp = currentTime;
        }

        return lastResult;
      }
    };
  }

  /**
   * Creates a supplier that calculates aiming results for lobbing. The results are cached per
   * control loop cycle to optimize performance.
   *
   * @param robotPoseSupplier A supplier for the robot's current {@link Pose2d}.
   * @param shooterRobotOffset The physical offset of the shooter from the robot's center.
   * @return A supplier that provides the calculated {@link AimingResult} for lobbing.
   */
  public static Supplier<AimingResult> lobAimingSupplier(
      Supplier<Pose2d> robotPoseSupplier, Translation2d shooterRobotOffset) {
    return new Supplier<AimingResult>() {
      private AimingResult lastResult = noTarget;
      private double lastTimestamp = -1.0;

      @Override
      public AimingResult get() {
        double currentTime = Math.round(Timer.getFPGATimestamp() * 50.0) / 50.0;

        if (currentTime != lastTimestamp) {
          Pose2d robotPose = robotPoseSupplier.get();
          lastResult =
              computeLobAiming(
                  robotPose.getTranslation(),
                  robotPose.getRotation(),
                  shooterRobotOffset,
                  Constants.lobbingTargetLeftCenter,
                  Constants.lobbingTargetRightCenter,
                  Constants.lobbingTargetHalfLength,
                  true);
          lastTimestamp = currentTime;
        }

        return lastResult;
      }
    };
  }
}
