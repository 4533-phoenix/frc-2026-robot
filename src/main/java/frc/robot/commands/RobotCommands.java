package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterKinematics;
import frc.robot.util.Util;

public class RobotCommands {
  public static Rotation2d getTargetRotation(Drive drive) {
    Translation2d targetTranslation = Util.flipAllianceIfNeeded(Constants.hubPosition);
    return targetTranslation.minus(drive.getPose().getTranslation()).getAngle();
  }

  public static Command getAutoAimCommand(
      Drive drive, Shooter shooter, CommandXboxController controller) {
    return Commands.parallel(
        DriveCommands.joystickDriveAtAngle(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> RobotCommands.getTargetRotation(drive)),

        // 2. SHOOTER: Update RPM and Hood based on distance
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
