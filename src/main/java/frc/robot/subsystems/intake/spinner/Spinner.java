// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.intake.spinner;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.intake.spinner.SpinnerConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.monitor.Monitored;
import frc.lib.monitor.checkers.SparkMonitor;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/** Subsystem for controlling the intake spinner rollers. */
public class Spinner extends SubsystemBase implements Monitored {
  private final SpinnerIO io;
  private final SpinnerIOInputsAutoLogged inputs = new SpinnerIOInputsAutoLogged();
  private final SparkMonitor healthMonitor = new SparkMonitor("Intake Spinner");

  /** Possible goals for the spinner. */
  public enum Goal {
    /** Stop the rollers. */
    STOP,
    /** Spin the rollers to pull game pieces in. */
    INTAKE,
    /** Spin the rollers to push game pieces out. */
    EXTAKE
  }

  @AutoLogOutput(key = "Intake/Spinner/Goal")
  private Goal goal = Goal.STOP;

  /**
   * Creates a new Spinner subsystem.
   *
   * @param io The IO implementation to use.
   */
  public Spinner(SpinnerIO io) {
    this.io = io;
  }

  /**
   * Sets the current requested behavior for the rollers.
   *
   * @param goal The target goal for the spinner.
   */
  public void setGoal(Goal goal) {
    this.goal = goal;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake/Spinner", inputs);
    healthMonitor.update(inputs.connected, inputs.status);

    // Apply the voltage based on the current goal
    switch (goal) {
      case INTAKE -> io.setVoltage(INTAKE_VOLTAGE);
      case EXTAKE -> io.setVoltage(EXTAKE_VOLTAGE);
      case STOP -> io.setVoltage(Volts.zero());
    }
  }

  /**
   * Returns a command to run the intake.
   *
   * @return The intake command.
   */
  public Command intake() {
    return this.startEnd(() -> setGoal(Goal.INTAKE), () -> setGoal(Goal.STOP));
  }

  /**
   * Returns a command to start the intake.
   *
   * @return The start intake command.
   */
  public Command startIntake() {
    return this.runOnce(() -> setGoal(Goal.INTAKE));
  }

  /**
   * Returns a command to run the extake.
   *
   * @return The extake command.
   */
  public Command extake() {
    return this.startEnd(() -> setGoal(Goal.EXTAKE), () -> setGoal(Goal.STOP));
  }

  /**
   * Returns a command to start the extake.
   *
   * @return The start extake command.
   */
  public Command startExtake() {
    return this.runOnce(() -> setGoal(Goal.EXTAKE));
  }

  /**
   * Returns a command to stop the spinner.
   *
   * @return The stop command.
   */
  public Command stop() {
    return this.runOnce(() -> setGoal(Goal.STOP));
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
