// Copyright (c) 2026 FRC Team 4533 (Phoenix)
//
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.intake.arm;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.intake.arm.ArmConstants.*;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.monitor.MonitoredSubsystemBase;
import frc.lib.monitor.checkers.SparkMonitor;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * Subsystem for controlling the robot's intake mechanism.
 *
 * <p>Responsible for deploying/retracting the intake arm and controlling the rollers to pull in or
 * push out game pieces.
 */
public class Arm extends MonitoredSubsystemBase {
  private final ArmIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
  private final SparkMonitor healthMonitor = new SparkMonitor("Intake Arm");

  /** High-level goals for the intake arm. */
  public enum Goal {
    /** Retract the arm inside the robot perimeter. */
    RETRACT,
    /** Goal state when the current position hasn't been determined yet. */
    UNKNOWN,
    /** Deploy the arm for intaking. */
    DEPLOY
  }

  @AutoLogOutput(key = "Intake/Arm/Goal")
  private Goal goal = Goal.UNKNOWN;

  @AutoLogOutput(key = "Intake/Arm/Oscillating")
  private boolean oscillate = false;

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
    Logger.processInputs("Intake/Arm", inputs);
    healthMonitor.update(inputs.connected, inputs.status);

    // If the robot is disabled we lose control over the arm
    if (DriverStation.isDisabled()) {
      if (deployedTrigger.getAsBoolean()) goal = Goal.DEPLOY;
      else if (retractedTrigger.getAsBoolean()) goal = Goal.RETRACT;
      else goal = Goal.UNKNOWN;
    } else if (goal == Goal.UNKNOWN) {
      if (deployedTrigger.getAsBoolean()) goal = Goal.DEPLOY;
      else if (retractedTrigger.getAsBoolean()) goal = Goal.RETRACT;
    }

    switch (goal) {
      case RETRACT -> io.setPosition(RETRACTED_POSITION);
      case DEPLOY -> {
        if (oscillate) {
          boolean atApex = (Timer.getFPGATimestamp() * OSCILLATE_FREQUENCY.in(Hertz)) % 1.0 > 0.5;
          io.setPosition(
              atApex ? DEPLOYED_POSITION.plus(OSCILLATE_POSITION_OFFSET) : DEPLOYED_POSITION);
        } else {
          io.setPosition(DEPLOYED_POSITION);
        }
      }
      case UNKNOWN -> io.stop();
    }
  }

  /**
   * Sets whether the arm should oscillate when deployed.
   *
   * @param oscillate True to oscillate, false to remain still.
   */
  public void setOscillate(boolean oscillate) {
    this.oscillate = oscillate;
  }

  /**
   * Command to deploy and oscillate the arm until canceled.
   *
   * @return The oscillation command.
   */
  public Command oscillate() {
    return this.startEnd(
        () -> {
          setOscillate(true);
        },
        () -> setOscillate(false));
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
