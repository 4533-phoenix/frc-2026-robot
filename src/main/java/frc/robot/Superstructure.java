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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterKinematics;
import frc.robot.util.Util;

public class Superstructure {
  public static Rotation2d getTargetRotation(Drive drive) {
    Translation2d targetTranslation = Util.flipAllianceIfNeeded(Constants.outpostPosition);
    return targetTranslation.minus(drive.getPose().getTranslation()).getAngle();
  }

  public static Trigger isReadyToFire(Drive drive, Shooter shooter) {
    return new Trigger(
        () -> {
          Rotation2d targetAngle = getTargetRotation(drive);
          boolean driveReady = drive.isAlignedWithTarget(targetAngle, Rotation2d.fromDegrees(2.0));
          boolean shooterReady = shooter.isReadyToShoot();

          return driveReady && shooterReady;
        });
  }

  public static Trigger isInShootingZone(Drive drive) {
    return new Trigger(
        () -> {
          Rectangle2d shootingZone = Util.flipAllianceIfNeeded(Constants.shootingZone);
          return shootingZone.contains(drive.getPose().getTranslation());
        });
  }

  public static Trigger isInLobbingZone(Drive drive) {
    return new Trigger(
        () -> {
          return Constants.lobbingZone.contains(drive.getPose().getTranslation());
        });
  }

  public static Trigger isOutpostEnabled() {
    return new Trigger(
        () -> {
          double time = DriverStation.getMatchTime();
          boolean isTimedMatch = DriverStation.isFMSAttached() || time > 0;

          // If we are just enabling the bot
          if (!isTimedMatch) {
            return DriverStation.isEnabled();
          }

          // We are in a Match or Practice Session
          if (DriverStation.isDisabled()) return false;
          if (DriverStation.isAutonomous() || time > 125 || time <= 30) return true;

          // Shift Logic
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

  public static Command getShooterAimCommand(Drive drive, Shooter shooter) {
    return Commands.run(
        () -> {
          Translation2d targetTranslation = Util.flipAllianceIfNeeded(Constants.outpostPosition);
          Distance distanceToHub =
              Meters.of(drive.getPose().getTranslation().getDistance(targetTranslation));
          shooter.setTargetState(ShooterKinematics.calculateShooterState(distanceToHub));
        },
        shooter);
  }
}
