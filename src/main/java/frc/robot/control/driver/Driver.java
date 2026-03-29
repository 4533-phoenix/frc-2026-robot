// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.control.driver;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class Driver extends SubsystemBase {
  private final DriverIO io;
  private final DriverIOInputsAutoLogged inputs = new DriverIOInputsAutoLogged();
  private final LoggedDashboardChooser<DriverProfile> chooser;

  public Driver(DriverIO io, LoggedDashboardChooser<DriverProfile> chooser) {
    this.io = io;
    this.chooser = chooser;
  }

  @Override
  public void periodic() {
    DriverProfile profile = chooser.get();
    if (profile == null) return;
    io.updateInputs(inputs, profile);
    Logger.processInputs("Driver", inputs);
    Logger.recordOutput("Driver/ActiveProfile", chooser.getSendableChooser().getSelected());

    GenericHID hid = profile.getHID();
    hid.setRumble(GenericHID.RumbleType.kLeftRumble, profile.getLeftRumble());
    hid.setRumble(GenericHID.RumbleType.kRightRumble, profile.getRightRumble());
  }

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

  public Trigger wantsAim() {
    return new Trigger(() -> inputs.wantsAim);
  }

  public Trigger wantsShoot() {
    return new Trigger(() -> inputs.wantsShoot);
  }

  public Trigger wantsReset() {
    return new Trigger(() -> inputs.wantsReset);
  }
}
