// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meters;

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

/**
 * The Superstructure class manages complex interactions between subsystems that depend on holistic
 * field state, robot pose, or match time.
 *
 * <p>It provides Triggers for conditional command execution and Commands for automated aiming and
 * alliance-specific logic.
 */
public class Superstructure {
  /**
   * Calculates the target rotation needed to face the outpost.
   *
   * @param drive The drive subsystem for current pose feedback.
   * @return The required Rotation2d to aim at the outpost.
   */
  public static Rotation2d getTargetRotation(Drive drive) {
    Translation2d targetTranslation = Util.flipAllianceIfNeeded(Constants.outpostPosition);
    return targetTranslation.minus(drive.getPose().getTranslation()).getAngle();
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
          return Constants.lobbingZone.contains(drive.getPose().getTranslation());
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
   * teleop translation, and concurrently aims the shooter.
   *
   * @param drive The drive subsystem.
   * @param shooter The shooter subsystem.
   * @param controller The Xbox controller for translation inputs.
   * @return An auto-aiming command.
   */
  public static Command getAutoAimCommand(
      Drive drive, Shooter shooter, CommandXboxController controller) {
    return Commands.parallel(
        DriveCommands.joystickDriveAtAngle(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> Superstructure.getTargetRotation(drive)),
        getShooterAimCommand(drive, shooter));
  }

  /**
   * Creates a command to constantly update the shooter target state based on the robot's distance
   * from the target.
   *
   * @param drive The drive subsystem.
   * @param shooter The shooter subsystem.
   * @return A command to calculate and set shooter kinematics.
   */
  public static Command getShooterAimCommand(Drive drive, Shooter shooter) {
    return Commands.run(
        () -> {
          Translation2d targetTranslation = Util.flipAllianceIfNeeded(Constants.outpostPosition);
          Distance distanceToHub =
              Meters.of(drive.getPose().getTranslation().getDistance(targetTranslation));
          // Calculate required hood angle and flywheel speed
          shooter.setTargetState(ShooterKinematics.calculateShooterState(distanceToHub));
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
