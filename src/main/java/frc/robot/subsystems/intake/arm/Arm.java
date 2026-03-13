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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.FaultUtil;
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
      new Alert("Intake arm motor disconnected", AlertType.kError);
  private final Alert faultAlert = new Alert("Intake arm motor fault detected", AlertType.kError);
  private final Alert warningAlert =
      new Alert("Intake arm motor warning detected", AlertType.kWarning);
  private final Alert stickyFaultAlert =
      new Alert("Intake arm motor sticky fault detected", AlertType.kInfo);
  private final Alert stickyWarningAlert =
      new Alert("Intake arm motor sticky warning detected", AlertType.kInfo);

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
                DEPLOYED_POSITION.minus(inputs.position).abs(Degrees)
                    < STATE_TOLERANCE.in(Degrees));
    retractedTrigger =
        new Trigger(
            () ->
                RETRACTED_POSITION.minus(inputs.position).abs(Degrees)
                    < STATE_TOLERANCE.in(Degrees));
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

    // Check for faults
    if (inputs.connected) {
      faultAlert.set(!inputs.healthy);
      if (!inputs.healthy) {
        faultAlert.setText(
            FaultUtil.getArrayString(
                "Intake Arm Motor Faults: ", FaultUtil.getSparkFaults(inputs.status[0])));
      }
      warningAlert.set(inputs.status[2] != 0);
      if (inputs.status[2] != 0) {
        warningAlert.setText(
            FaultUtil.getArrayString(
                "Intake Arm Motor Warnings: ", FaultUtil.getSparkWarnings(inputs.status[2])));
      }

      stickyFaultAlert.set(inputs.status[1] != 0);
      if (inputs.status[1] != 0) {
        stickyFaultAlert.setText(
            FaultUtil.getArrayString(
                "Intake Arm Motor Sticky Faults: ", FaultUtil.getSparkFaults(inputs.status[1])));
      }
      stickyWarningAlert.set(inputs.status[3] != 0);
      if (inputs.status[3] != 0) {
        stickyWarningAlert.setText(
            FaultUtil.getArrayString(
                "Intake Arm Motor Sticky Warnings: ",
                FaultUtil.getSparkWarnings(inputs.status[3])));
      }
    } else {
      faultAlert.set(false);
      warningAlert.set(false);
      stickyFaultAlert.set(false);
      stickyWarningAlert.set(false);
    }

    // If the robot is disabled we lose control over the arm
    if (goal != Goal.UNKNOWN && DriverStation.isDisabled()) {
      setGoal(Goal.UNKNOWN);
    }

    switch (goal) {
      case RETRACT -> io.setPosition(RETRACTED_POSITION);
      case DEPLOY -> io.setPosition(DEPLOYED_POSITION);
      case UNKNOWN -> {
        io.stop();
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

  /** Convenience method to set the retract goal directly. */
  public void setRetract() {
    setGoal(Goal.RETRACT);
  }

  /** Convenience method to set the deploy goal directly. */
  public void setDeploy() {
    setGoal(Goal.DEPLOY);
  }

  /**
   * Returns whether or not the subsystem is healthy
   *
   * @return True if the subsystem is healthy, false otherwise.
   */
  public boolean isHealthy() {
    return inputs.healthy && inputs.connected;
  }

  /** Clears all faults and warnings. */
  public void clearFaults() {
    io.clearFaults();
  }
}
