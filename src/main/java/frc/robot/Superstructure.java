// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
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
    Translation2d targetTranslation = Util.flipAllianceIfNeeded(Constants.hubPosition);
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

  public static Command getAutoAimCommand(
      Drive drive, Shooter shooter, CommandXboxController controller) {
    return Commands.parallel(
        DriveCommands.joystickDriveAtAngle(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> Superstructure.getTargetRotation(drive)),

        Commands.run(
            () -> {
              Translation2d targetTranslation = Util.flipAllianceIfNeeded(Constants.hubPosition);
              Distance distanceToHub =
                  Meters.of(drive.getPose().getTranslation().getDistance(targetTranslation));
              shooter.setTargetState(ShooterKinematics.calculateShooterState(distanceToHub));
            },
            shooter));
  }
}
