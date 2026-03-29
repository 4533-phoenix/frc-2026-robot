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

/** Subsystem for handling operator controls and input processing. */
public class Operator extends SubsystemBase {
  private final OperatorIO io;
  private final OperatorIOInputsAutoLogged inputs = new OperatorIOInputsAutoLogged();
  private final LoggedDashboardChooser<OperatorProfile> chooser;

  /**
   * Constructs the Operator subsystem.
   *
   * @param io The OperatorIO implementation.
   * @param chooser The dashboard chooser for operator profiles.
   */
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

  /**
   * Returns a trigger that is active when the operator wants to deploy the arm.
   *
   * @return The arm deployment trigger.
   */
  public Trigger wantsArmDeployment() {
    return new Trigger(() -> inputs.armDeployment);
  }

  /**
   * Returns a trigger that is active when the operator wants to retract the arm.
   *
   * @return The arm retraction trigger.
   */
  public Trigger wantsArmRetraction() {
    return new Trigger(() -> inputs.armRetraction);
  }

  /**
   * Returns a trigger that is active when the operator wants to intake.
   *
   * @return The intake trigger.
   */
  public Trigger wantsIntake() {
    return new Trigger(() -> inputs.intake);
  }

  /**
   * Returns a trigger that is active when the operator wants to extake.
   *
   * @return The extake trigger.
   */
  public Trigger wantsExtake() {
    return new Trigger(() -> inputs.extake);
  }

  /**
   * Returns a trigger that is active when the operator wants to climb.
   *
   * @return The climb trigger.
   */
  public Trigger wantsClimb() {
    return new Trigger(() -> inputs.climb);
  }

  /**
   * Returns a trigger that is active when the operator wants to raise the climber.
   *
   * @return The climber up trigger.
   */
  public Trigger wantsClimberUp() {
    return new Trigger(() -> inputs.climberUp);
  }

  /**
   * Returns a trigger that is active when the operator wants to lower the climber.
   *
   * @return The climber down trigger.
   */
  public Trigger wantsClimberDown() {
    return new Trigger(() -> inputs.climberDown);
  }
}
