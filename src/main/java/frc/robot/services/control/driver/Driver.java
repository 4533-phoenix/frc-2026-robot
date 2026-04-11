// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.services.control.driver;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.services.control.ControlService;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/** Subsystem for handling driver controls and input processing. */
public class Driver extends ControlService<DriverProfile, DriverIO, DriverIOInputsAutoLogged> {

  /**
   * Constructs the Driver subsystem.
   *
   * @param io The DriverIO implementation.
   * @param chooser The dashboard chooser for driver profiles.
   */
  public Driver(DriverIO io, LoggedDashboardChooser<DriverProfile> chooser) {
    super("Driver", io, new DriverIOInputsAutoLogged(), chooser);
  }

  @Override
  protected void updateInputs(DriverIO io, DriverIOInputsAutoLogged inputs, DriverProfile profile) {
    io.updateInputs(inputs, profile);
  }

  /**
   * Creates a command to drive the robot using the current driver profile inputs.
   *
   * @param drive The drive subsystem.
   * @return The drive command.
   */
  public Command createDriveCommand(Drive drive) {
    return Commands.run(
            () -> {
              ChassisSpeeds speeds =
                  new ChassisSpeeds(
                      inputs.vxMetersPerSecond,
                      inputs.vyMetersPerSecond,
                      inputs.omegaRadiansPerSecond);

              boolean isFlipped = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
              Rotation2d rot =
                  isFlipped ? drive.getRotation().plus(Rotation2d.kPi) : drive.getRotation();

              drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, rot));
            },
            drive)
        .beforeStarting(() -> drive.setHeadingOverrideSupplier(null));
  }

  /**
   * Returns a trigger that is active when the driver wants to aim.
   *
   * @return The aim trigger.
   */
  public Trigger wantsAim() {
    return new Trigger(() -> inputs.wantsAim);
  }

  /**
   * Returns a trigger that is active when the driver wants to shoot.
   *
   * @return The shoot trigger.
   */
  public Trigger wantsShoot() {
    return new Trigger(() -> inputs.wantsShoot);
  }

  /**
   * Returns a trigger that is active when the driver wants to reset the robot pose.
   *
   * @return The reset trigger.
   */
  public Trigger wantsReset() {
    return new Trigger(() -> inputs.wantsReset);
  }

  /**
   * Returns a trigger that is active when the driver wants to unjam the indexer.
   *
   * @return The unjam trigger.
   */
  public Trigger wantsUnjam() {
    return new Trigger(() -> inputs.wantsUnjam);
  }
}
