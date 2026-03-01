// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.subsystems.shooter.ShooterConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterKinematics;
import frc.robot.util.Util;
import org.littletonrobotics.junction.Logger;

/**
 * The Superstructure class manages complex interactions between subsystems that depend on holistic
 * field state, robot pose, or match time.
 *
 * <p>It provides Triggers for conditional command execution and Commands for automated aiming and
 * alliance-specific logic.
 */
public class Superstructure {
  /**
   * Returns the field-space position of the shooter mechanism, accounting for the shooter's offset
   * from the robot center.
   *
   * @param drive The drive subsystem for current pose feedback.
   * @return The shooter's position on the field.
   */
  public static Translation2d getShooterFieldPosition(Drive drive) {
    Pose2d robotPose = drive.getPose();
    // Transform robot-relative shooter offset into field coordinates
    return robotPose.getTranslation().plus(shooterRobotOffset.rotateBy(robotPose.getRotation()));
  }

  /**
   * Calculates the lead offset to apply to the static target, accounting for the robot's current
   * velocity and the estimated time of flight of the game piece.
   *
   * <p>The lead offset is clamped so it can never exceed 50% of the distance from the shooter to
   * the target. Without this clamp, a fast-moving robot could push the virtual target behind the
   * shooter, flipping the aim direction by 180°.
   *
   * @param shooterPos The shooter's field-space position.
   * @param targetTranslation The static target position on the field.
   * @param fieldVelocity The robot's field-relative velocity.
   * @return The clamped lead offset to subtract from the static target.
   */
  private static Translation2d calculateClampedLead(
      Translation2d shooterPos, Translation2d targetTranslation, Translation2d fieldVelocity) {
    double tof = estimatedTimeOfFlight.in(Seconds);

    // Compute the raw lead offset (velocity * time-of-flight)
    Translation2d rawLead =
        new Translation2d(fieldVelocity.getX() * tof, fieldVelocity.getY() * tof);

    // Clamp the lead magnitude to at most half the shooter-to-target distance so the
    // virtual target can never overshoot past the real target (which would flip aim 180°).
    double distToTarget = shooterPos.getDistance(targetTranslation);
    double maxLead = distToTarget * 0.5;
    double leadMag = rawLead.getNorm();

    if (leadMag > maxLead && leadMag > 1e-6) {
      return rawLead.times(maxLead / leadMag);
    }
    return rawLead;
  }

  /**
   * Computes all aiming outputs in a single pass to avoid redundant calculations. Returns the
   * target rotation for the drivetrain and the distance from the shooter to the virtual target for
   * the shooter kinematics.
   *
   * @param drive The drive subsystem for current pose and velocity feedback.
   * @param log Whether to record outputs to AdvantageKit for visualization.
   * @return An {@link AimingResult} containing the target heading and distance.
   */
  private static AimingResult computeAiming(Drive drive, boolean log) {
    Translation2d targetTranslation = Util.flipAllianceIfNeeded(Constants.outpostPosition);
    Translation2d shooterPos = getShooterFieldPosition(drive);
    Translation2d fieldVelocity = drive.getFieldRelativeVelocity();

    // Subtract lead: if we move right the ball drifts right, so aim left of the static target.
    Translation2d clampedLead = calculateClampedLead(shooterPos, targetTranslation, fieldVelocity);
    Translation2d virtualTarget = targetTranslation.minus(clampedLead);

    Translation2d shooterToTarget = virtualTarget.minus(shooterPos);
    Rotation2d targetRotation = shooterToTarget.getAngle();
    double distanceMeters = shooterToTarget.getNorm();

    if (log) {
      Logger.recordOutput("Aiming/VirtualTarget", new Pose2d(virtualTarget, Rotation2d.kZero));
      Logger.recordOutput("Aiming/StaticTarget", new Pose2d(targetTranslation, Rotation2d.kZero));
      Logger.recordOutput("Aiming/ShooterPosition", new Pose2d(shooterPos, drive.getRotation()));
      Logger.recordOutput("Aiming/TargetRotation", targetRotation.getDegrees());
      Logger.recordOutput("Aiming/DistanceToTarget", distanceMeters);
    }

    return new AimingResult(targetRotation, Meters.of(distanceMeters));
  }

  /** Cached result of the aiming pipeline to avoid redundant computation across subsystems. */
  private record AimingResult(Rotation2d targetRotation, Distance distanceToTarget) {}

  /**
   * Calculates the target rotation needed to face the outpost. This is the lightweight variant used
   * by triggers (e.g. isReadyToFire) that only need the heading and should not spam logs.
   *
   * @param drive The drive subsystem for current pose feedback.
   * @return The required Rotation2d to aim the shooter at the virtual target.
   */
  public static Rotation2d getTargetRotation(Drive drive) {
    return computeAiming(drive, false).targetRotation();
  }

  /**
   * Creates a trigger that activates when the robot is aligned with the target and the shooter is
   * up to speed.
   *
   * @param drive The drive subsystem.
   * @param shooter The shooter subsystem.
   * @return A trigger representing the "ready to fire" state.
   */
  public static Trigger isReadyToFire(Drive drive, Shooter shooter) {
    return new Trigger(
        () -> {
          Rotation2d targetAngle = getTargetRotation(drive);
          // Check if drive is within 2 degrees of target angle
          boolean driveReady = drive.isAlignedWithTarget(targetAngle, Rotation2d.fromDegrees(2.0));
          boolean shooterReady = shooter.isReadyToShoot();

          return driveReady && shooterReady;
        });
  }

  /**
   * Trigger that is active when the robot is within the designated shooting zone.
   *
   * @param drive The drive subsystem.
   * @return A trigger for the shooting zone.
   */
  public static Trigger isInShootingZone(Drive drive) {
    return new Trigger(
        () -> {
          Rectangle2d shootingZone = Util.flipAllianceIfNeeded(Constants.shootingZone);
          return shootingZone.contains(drive.getPose().getTranslation());
        });
  }

  /**
   * Trigger that is active when the robot is within the designated lobbing zone.
   *
   * @param drive The drive subsystem.
   * @return A trigger for the lobbing zone.
   */
  public static Trigger isInLobbingZone(Drive drive) {
    return new Trigger(
        () -> {
          Rectangle2d lobbingZone = Util.flipAllianceIfNeeded(Constants.lobbingZone);
          return lobbingZone.contains(drive.getPose().getTranslation());
        });
  }

  /**
   * Trigger that is active during the last 30 seconds of a match.
   *
   * @return A trigger for the endgame period.
   */
  public static Trigger isEndgame() {
    return new Trigger(
        () -> {
          if (DriverStation.isDisabled()) return false;
          if (DriverStation.isAutonomous()) return false;

          // Check if we are in a timed session (FMS or Practice Mode)
          double time = DriverStation.getMatchTime();
          boolean isTimedMatch = DriverStation.isFMSAttached() || time > 0;

          // Return true only if a timer is running and we are at or below 30s.
          return isTimedMatch && time <= 30.0 && time > 0;
        });
  }

  /**
   * Trigger that determines if the outpost is active based on 2026 match shift rules.
   *
   * @return A trigger for outpost activation status.
   */
  public static Trigger isOutpostEnabled() {
    return new Trigger(
        () -> {
          double time = DriverStation.getMatchTime();
          boolean isTimedMatch = DriverStation.isFMSAttached() || time > 0;

          // If we are just enabling the bot (e.g., testing in pits)
          if (!isTimedMatch) {
            return DriverStation.isEnabled();
          }

          // We are in a Match or Practice Session
          if (DriverStation.isDisabled()) return false;
          // Outpost active in Auto or outside specific shift windows
          if (DriverStation.isAutonomous() || time > 125 || time <= 30) return true;

          // FMS Game Data Logic
          String data = DriverStation.getGameSpecificMessage();
          var alliance = DriverStation.getAlliance();

          // If in Practice Mode and forgot to type Game Data, default to ENABLED
          if (data == null || data.isEmpty() || alliance.isEmpty()) return true;

          char inactiveInShift1 = data.charAt(0);
          char myColor = (alliance.get() == DriverStation.Alliance.Red) ? 'R' : 'B';
          boolean amIInactiveInShift1 = (inactiveInShift1 == myColor);

          // Standard 2026 Shift Windows
          if (time > 100) return !amIInactiveInShift1; // Shift 1
          else if (time > 75) return amIInactiveInShift1; // Shift 2
          else if (time > 50) return !amIInactiveInShift1; // Shift 3
          else return amIInactiveInShift1; // Shift 4
        });
  }

  /**
   * Creates a command that automatically rotates the robot to face the target while allowing for
   * teleop translation, and concurrently aims the shooter. The aiming pipeline is computed once per
   * cycle and shared between the drive heading and shooter kinematics.
   *
   * <p>Rotation is given priority over translation: the PID-controlled heading correction claims as
   * much of the module speed budget as it needs, and translation fills whatever remains.
   *
   * @param drive The drive subsystem.
   * @param shooter The shooter subsystem.
   * @param controller The Xbox controller for translation inputs.
   * @return An auto-aiming command.
   */
  public static Command getAutoAimCommand(
      Drive drive, Shooter shooter, CommandXboxController controller) {
    // Shared reference so the aiming result is computed once and read by both subsystem commands.
    final AimingResult[] cachedResult = {computeAiming(drive, false)};

    return Commands.parallel(
        // Update the cached aiming result and set shooter state each cycle
        Commands.run(
            () -> {
              cachedResult[0] = computeAiming(drive, true);
              shooter.setTargetState(
                  ShooterKinematics.calculateShooterState(cachedResult[0].distanceToTarget()));
            },
            shooter),
        // Drive with rotation priority, reading heading from the cache
        DriveCommands.joystickDriveWithRotationPriority(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> cachedResult[0].targetRotation()));
  }

  /**
   * Creates a standalone command to aim the shooter without controlling the drivetrain. Useful for
   * autonomous routines or testing where the drive heading is managed separately.
   *
   * @param drive The drive subsystem.
   * @param shooter The shooter subsystem.
   * @return A command that continuously sets the shooter to the optimal state.
   */
  public static Command getShooterAimCommand(Drive drive, Shooter shooter) {
    return Commands.run(
        () -> {
          AimingResult result = computeAiming(drive, true);
          shooter.setTargetState(ShooterKinematics.calculateShooterState(result.distanceToTarget()));
        },
        shooter);
  }

  /**
   * Creates a command to rumble a controller for a specific duration.
   *
   * @param controller The controller to rumble.
   * @param type The type of rumble (Left or Right).
   * @param intensity The intensity of the rumble (0.0 to 1.0).
   * @param duration The duration of the rumble in seconds.
   * @return A command to rumble the controller.
   */
  public static Command rumbleCommand(
      CommandGenericHID controller, RumbleType type, double intensity, double duration) {
    return Commands.startEnd(
            () -> controller.setRumble(type, intensity), () -> controller.setRumble(type, 0.0))
        .withTimeout(duration);
  }
}
