// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.intake.arm;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.intake.arm.ArmConstants.*;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * Subsystem for controlling the robot's intake mechanism.
 *
 * <p>Responsible for deploying/retracting the intake arm and controlling the rollers to pull in or
 * push out game pieces.
 */
public class Arm extends SubsystemBase {
  private final ArmIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

  /** High-level goals for the intake arm. */
  public enum Goal {
    /** Retract the arm inside the robot perimeter. */
    RETRACT,
    /** Goal state when the current position hasn't been determined yet. */
    UNKNOWN,
    /** Deploy the arm for intaking. */
    DEPLOY
  }

  @AutoLogOutput private Goal goal = Goal.UNKNOWN;

  // Alerts for hardware monitoring
  private final Alert disconnectedAlert =
      new Alert("Intake arm motor disconnected", AlertType.kWarning);

  private final Trigger deployedTrigger;
  private final Trigger retractedTrigger;

  /**
   * Creates a new Arm subsystem.
   *
   * @param io The abstraction layer for the arm hardware.
   */
  public Arm(ArmIO io) {
    this.io = io;

    deployedTrigger =
        new Trigger(
            () ->
                deployedPosition.minus(inputs.position).abs(Degrees) < intakeTolerance.in(Degrees));
    retractedTrigger =
        new Trigger(
            () ->
                retractedPosition.minus(inputs.position).abs(Degrees)
                    < intakeTolerance.in(Degrees));
  }

  /**
   * Sets the high-level goal for the intake arm.
   *
   * @param goal The target goal for the arm.
   */
  public void setGoal(Goal goal) {
    this.goal = goal;
  }

  /** Updates hardware inputs, logs data, and updates status alerts. */
  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);
    disconnectedAlert.set(!inputs.connected);

    switch (goal) {
      case RETRACT -> io.setPosition(retractedPosition);
      case DEPLOY -> io.setPosition(deployedPosition);
      case UNKNOWN -> {
        if (deployedTrigger.getAsBoolean()) {
          setGoal(Goal.DEPLOY);
        } else if (retractedTrigger.getAsBoolean()) {
          setGoal(Goal.RETRACT);
        }
      }
    }
  }

  /**
   * Returns a trigger that is true when the arm is in the deployed position.
   *
   * @return The deployed trigger.
   */
  public Trigger isDeployed() {
    return deployedTrigger;
  }

  /**
   * Returns a trigger that is true when the arm is in the retracted position.
   *
   * @return The retracted trigger.
   */
  public Trigger isRetracted() {
    return retractedTrigger;
  }

  /**
   * Returns a command to move the arm to the deployed position.
   *
   * @return The deploy command.
   */
  public Command deploy() {
    return this.runOnce(() -> setGoal(Goal.DEPLOY));
  }

  /**
   * Returns a command to move the arm to the retracted position.
   *
   * @return The retract command.
   */
  public Command retract() {
    return this.runOnce(() -> setGoal(Goal.RETRACT));
  }
}
