// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.util;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import frc.lib.util.FieldUtil;
import frc.robot.Constants;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/** Utility class for calculating aiming solutions for the shooter. */
public class Aiming {
  private Aiming() {} // Prevent instantiation

  /**
   * Cached result of the aiming pipeline, including target rotation, distance, and target presence.
   */
  public record AimingResult(
      Rotation2d targetRotation, double distanceToTargetMeters, boolean hasTarget) {}

  /** A default empty result to prevent NullPointerExceptions */
  public static final AimingResult NO_TARGET = new AimingResult(new Rotation2d(), 0.0, false);

  /**
   * Gets the amount of angle to compensate for curve of the ball in the air.
   *
   * @param distanceMeters The distance in meters to the target.
   * @return The compensation angle in radians.
   */
  private static Rotation2d getCurveCompensation(double distanceMeters) {
    // Define our data points
    double minDist = 1.307;
    double maxDist = 3.155;
    double minCurveDeg = 0.0;
    double maxCurveDeg = 5.0;

    // Clamp the distance to our known range
    double clampedDist = Math.max(minDist, Math.min(maxDist, distanceMeters));

    // Linear Interpolation
    double curveDegrees =
        minCurveDeg + (clampedDist - minDist) * ((maxCurveDeg - minCurveDeg) / (maxDist - minDist));

    // The result is added to the target angle
    return Rotation2d.fromDegrees(curveDegrees);
  }

  /**
   * Computes all aiming outputs for a direct hub shot with lead compensation.
   *
   * <p>This method uses a two-pass approach: Pass 1 estimates the shooter's position at the current
   * robot heading to find a rough target angle. Pass 2 re-calculates the shooter's position at that
   * estimated heading to provide a final, precise aiming solution.
   *
   * @param robotCenter The current center position of the robot.
   * @param currentRobotRotation The current rotation of the robot.
   * @param fieldVelocity The robot's field-relative velocity (m/s).
   * @param targetPosition The blue-alliance target position.
   * @param shooterRobotOffset The physical offset of the shooter from the robot's center.
   * @param estimatedTimeOfFlight Estimated time for the projectile to travel to the target.
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

    Translation2d target = FieldUtil.flipAllianceIfNeeded(targetPosition);
    double targetX = target.getX();
    double targetY = target.getY();
    double robotX = robotCenter.getX();
    double robotY = robotCenter.getY();
    double robotAngle = currentRobotRotation.getRadians();
    double offsetX = shooterRobotOffset.getX();
    double offsetY = shooterRobotOffset.getY();
    double velX = fieldVelocity.getX();
    double velY = fieldVelocity.getY();
    double tof = estimatedTimeOfFlight.in(edu.wpi.first.units.Units.Seconds);

    // Estimate rotation from current position
    double cos = Math.cos(robotAngle);
    double sin = Math.sin(robotAngle);
    double shooterX = robotX + (offsetX * cos - offsetY * sin);
    double shooterY = robotY + (offsetX * sin + offsetY * cos);

    double virtualTargetX = targetX - (velX * tof);
    double virtualTargetY = targetY - (velY * tof);

    // Clamp lead
    double dxRaw = virtualTargetX - shooterX;
    double dyRaw = virtualTargetY - shooterY;
    double distToVirtual = Math.sqrt(dxRaw * dxRaw + dyRaw * dyRaw);
    double leadX = velX * tof;
    double leadY = velY * tof;
    double leadMag = Math.sqrt(leadX * leadX + leadY * leadY);
    double maxLead = distToVirtual * 0.5;

    if (leadMag > maxLead && leadMag > 1e-6) {
      double scale = maxLead / leadMag;
      virtualTargetX = targetX - (leadX * scale);
      virtualTargetY = targetY - (leadY * scale);
    }

    double estimatedAngle = Math.atan2(virtualTargetY - shooterY, virtualTargetX - shooterX);

    // Recompute with predicted shooter position
    cos = Math.cos(estimatedAngle);
    sin = Math.sin(estimatedAngle);
    double predShooterX = robotX + (offsetX * cos - offsetY * sin);
    double predShooterY = robotY + (offsetX * sin + offsetY * cos);

    double finalDx = virtualTargetX - predShooterX;
    double finalDy = virtualTargetY - predShooterY;
    double finalAngle = Math.atan2(finalDy, finalDx);
    double finalDist = Math.sqrt(finalDx * finalDx + finalDy * finalDy);

    Rotation2d baseRotation = Rotation2d.fromRadians(finalAngle);
    Rotation2d finalRotation = baseRotation.plus(getCurveCompensation(finalDist));

    if (log) {
      Logger.recordOutput(
          "Aiming/VirtualTarget", new Pose2d(virtualTargetX, virtualTargetY, Rotation2d.kZero));
      Logger.recordOutput(
          "Aiming/ShooterPosition", new Pose2d(shooterX, shooterY, currentRobotRotation));
      Logger.recordOutput(
          "Aiming/PredictedShooterPosition",
          new Pose2d(predShooterX, predShooterY, Rotation2d.fromRadians(estimatedAngle)));
      Logger.recordOutput("Aiming/TargetRotation", finalRotation.getDegrees());
      Logger.recordOutput("Aiming/CurveCompDegrees", getCurveCompensation(finalDist).getDegrees());
      Logger.recordOutput("Aiming/DistanceToTarget", finalDist);
    }

    return new AimingResult(finalRotation, finalDist, true);
  }

  /**
   * Computes aiming for lobbed shots targeted at specific field line segments.
   *
   * <p>The robot will automatically select the closest of the two provided lob targets and
   * calculate the closest point on that line segment to the shooter.
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

    Translation2d leftCenter = FieldUtil.flipAllianceIfNeeded(lobTargetLeftCenter);
    Translation2d rightCenter = FieldUtil.flipAllianceIfNeeded(lobTargetRightCenter);
    double robotX = robotCenter.getX();
    double robotY = robotCenter.getY();
    double robotAngle = currentRobotRotation.getRadians();
    double offsetX = shooterRobotOffset.getX();
    double offsetY = shooterRobotOffset.getY();
    double halfLen = lobTargetHalfLength.in(Meters);

    double cos = Math.cos(robotAngle);
    double sin = Math.sin(robotAngle);
    double shooterX = robotX + (offsetX * cos - offsetY * sin);
    double shooterY = robotY + (offsetX * sin + offsetY * cos);

    double leftClampedY =
        Math.max(leftCenter.getY() - halfLen, Math.min(leftCenter.getY() + halfLen, shooterY));
    double rightClampedY =
        Math.max(rightCenter.getY() - halfLen, Math.min(rightCenter.getY() + halfLen, shooterY));

    double dxL = leftCenter.getX() - shooterX;
    double dyL = leftClampedY - shooterY;
    double dxR = rightCenter.getX() - shooterX;
    double dyR = rightClampedY - shooterY;

    boolean useLeft = (dxL * dxL + dyL * dyL) < (dxR * dxR + dyR * dyR);
    double targetX = useLeft ? leftCenter.getX() : rightCenter.getX();
    double targetCenterY = useLeft ? leftCenter.getY() : rightCenter.getY();

    // Use shooter position at current robot rotation
    double clampedY =
        Math.max(targetCenterY - halfLen, Math.min(targetCenterY + halfLen, shooterY));
    double estimatedAngle = Math.atan2(clampedY - shooterY, targetX - shooterX);

    // Predict shooter position
    cos = Math.cos(estimatedAngle);
    sin = Math.sin(estimatedAngle);
    double predShooterX = robotX + (offsetX * cos - offsetY * sin);
    double predShooterY = robotY + (offsetX * sin + offsetY * cos);
    double finalClampedY =
        Math.max(targetCenterY - halfLen, Math.min(targetCenterY + halfLen, predShooterY));

    double finalDx = targetX - predShooterX;
    double finalDy = finalClampedY - predShooterY;
    double finalAngle = Math.atan2(finalDy, finalDx);
    double finalDist = Math.sqrt(finalDx * finalDx + finalDy * finalDy);

    Rotation2d baseRotation = Rotation2d.fromRadians(finalAngle);
    Rotation2d finalRotation = baseRotation.plus(getCurveCompensation(finalDist));

    if (log) {
      Logger.recordOutput(
          "Aiming/VirtualTarget", new Pose2d(targetX, finalClampedY, Rotation2d.kZero));
      Logger.recordOutput("Aiming/TargetRotation", finalRotation.getDegrees());
      Logger.recordOutput("Aiming/CurveCompDegrees", getCurveCompensation(finalDist).getDegrees());
      Logger.recordOutput("Aiming/DistanceToTarget", finalDist);
    }

    return new AimingResult(finalRotation, finalDist, true);
  }

  /**
   * Creates a supplier for central hub aiming results. Caches results based on 20ms timestamps to
   * prevent redundant calculation within a single control loop.
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

      @Override
      public AimingResult get() {
        Pose2d robotPose = robotPoseSupplier.get();
        return computeHubAiming(
            robotPose.getTranslation(),
            robotPose.getRotation(),
            fieldVelocitySupplier.get(),
            Constants.HUB_POSITION,
            shooterRobotOffset,
            estimatedTimeOfFlight,
            true);
      }
    };
  }

  /**
   * Creates a supplier for lobbing aiming results. Caches results based on 20ms timestamps.
   *
   * @param robotPoseSupplier A supplier for the robot's current {@link Pose2d}.
   * @param shooterRobotOffset The physical offset of the shooter from the robot's center.
   * @return A supplier that provides the calculated {@link AimingResult} for lobbing.
   */
  public static Supplier<AimingResult> lobAimingSupplier(
      Supplier<Pose2d> robotPoseSupplier, Translation2d shooterRobotOffset) {
    return new Supplier<AimingResult>() {
      @Override
      public AimingResult get() {
        Pose2d robotPose = robotPoseSupplier.get();
        return computeLobAiming(
            robotPose.getTranslation(),
            robotPose.getRotation(),
            shooterRobotOffset,
            Constants.LOBBING_TARGET_LEFT_CENTER,
            Constants.LOBBING_TARGET_RIGHT_CENTER,
            Constants.LOBBING_TARGET_HALF_LENGTH,
            true);
      }
    };
  }
}
