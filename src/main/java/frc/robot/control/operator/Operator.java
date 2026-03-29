// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.control.operator;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class Operator extends SubsystemBase {
  private final OperatorIO io;
  private final OperatorIOInputsAutoLogged inputs = new OperatorIOInputsAutoLogged();
  private final LoggedDashboardChooser<OperatorProfile> chooser;

  public Operator(OperatorIO io, LoggedDashboardChooser<OperatorProfile> chooser) {
    this.io = io;
    this.chooser = chooser;
  }

  @Override
  public void periodic() {
    OperatorProfile profile = chooser.get();
    if (profile == null) return;
    io.updateInputs(inputs, profile);
    Logger.processInputs("Operator", inputs);
    Logger.recordOutput("Operator/ActiveProfile", chooser.getSendableChooser().getSelected());

    GenericHID hid = profile.getHID();
    hid.setRumble(GenericHID.RumbleType.kLeftRumble, profile.getLeftRumble());
    hid.setRumble(GenericHID.RumbleType.kRightRumble, profile.getRightRumble());
  }

  public Trigger wantsArmDeployment() {
    return new Trigger(() -> inputs.armDeployment);
  }

  public Trigger wantsArmRetraction() {
    return new Trigger(() -> inputs.armRetraction);
  }

  public Trigger wantsIntake() {
    return new Trigger(() -> inputs.intake);
  }

  public Trigger wantsExtake() {
    return new Trigger(() -> inputs.extake);
  }

  public Trigger wantsClimb() {
    return new Trigger(() -> inputs.climb);
  }

  public Trigger wantsClimberUp() {
    return new Trigger(() -> inputs.climberUp);
  }

  public Trigger wantsClimberDown() {
    return new Trigger(() -> inputs.climberDown);
  }
}
