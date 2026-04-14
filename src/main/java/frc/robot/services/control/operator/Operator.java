// Copyright (c) 2026 FRC Team 4533 (Phoenix)
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.services.control.operator;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.services.control.ControlService;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/** Subsystem for handling operator controls and input processing. */
public class Operator
    extends ControlService<OperatorProfile, OperatorIO, OperatorIOInputsAutoLogged> {

  /**
   * Constructs the Operator subsystem.
   *
   * @param io The OperatorIO implementation.
   * @param chooser The dashboard chooser for operator profiles.
   */
  public Operator(OperatorIO io, LoggedDashboardChooser<OperatorProfile> chooser) {
    super("Operator", io, new OperatorIOInputsAutoLogged(), chooser);
  }

  @Override
  protected void updateInputs(
      OperatorIO io, OperatorIOInputsAutoLogged inputs, OperatorProfile profile) {
    io.updateInputs(inputs, profile);
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
