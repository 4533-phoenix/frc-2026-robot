// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.indexer.IndexerConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.monitor.MonitoredSubsystemBase;
import frc.lib.monitor.checkers.SparkMonitor;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * Subsystem for the robot's indexer mechanism.
 *
 * <p>Responsible for controlling the speed of the motor driving the indexer to transfer game pieces
 * from the intake to the shooter.
 */
public class Indexer extends MonitoredSubsystemBase {
  private final IndexerIO io;
  private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();
  private final SparkMonitor healthMonitor = new SparkMonitor("Indexer");

  /** High-level goals for the indexer. */
  public enum Goal {
    /** Stop the indexer motor. */
    STOP,
    /** Run the indexer to move game pieces toward the shooter. */
    RUNNING,
    /** Unjam the indexer. */
    UNJAM
  }

  @AutoLogOutput private Goal goal = Goal.STOP;

  /**
   * Creates a new Indexer subsystem.
   *
   * @param io The abstraction layer for the indexer hardware.
   */
  public Indexer(IndexerIO io) {
    this.io = io;
  }

  /**
   * Sets the current goal for the indexer.
   *
   * @param goal The target goal.
   */
  public void setGoal(Goal goal) {
    this.goal = goal;
  }

  /** Sets the indexer to run. */
  public void setRunning() {
    setGoal(Goal.RUNNING);
  }

  /** Sets the indexer to stop. */
  public void setStop() {
    setGoal(Goal.STOP);
  }

  /** Sets the indexer to unjam. */
  public void setUnjam() {
    setGoal(Goal.UNJAM);
  }

  /** Updates hardware inputs, logs data, and updates status alerts. */
  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Indexer", inputs);
    healthMonitor.update(inputs.connected, inputs.status);

    switch (goal) {
      case RUNNING -> io.setVoltage(DEFAULT_VOLTAGE);
      case STOP -> io.setVoltage(Volts.zero());
      case UNJAM -> io.setVoltage(DEFAULT_VOLTAGE.unaryMinus());
    }
  }

  /**
   * Returns a command to run the indexer while the command is held.
   *
   * @return The run command.
   */
  public Command run() {
    return this.startEnd(() -> setGoal(Goal.RUNNING), () -> setGoal(Goal.STOP));
  }

  /**
   * Returns a command to unjam the indexer while the command is held.
   *
   * @return The unjam command.
   */
  public Command unjam() {
    return this.startEnd(() -> setGoal(Goal.UNJAM), () -> setGoal(Goal.STOP));
  }

  /**
   * Returns a command to start running the indexer.
   *
   * @return The start run command.
   */
  public Command startRun() {
    return this.run(() -> setGoal(Goal.RUNNING));
  }

  /**
   * Returns a command to stop the indexer.
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
