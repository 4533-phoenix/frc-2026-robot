// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.indexer.IndexerConstants.*;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * Subsystem for the robot's indexer mechanism.
 *
 * <p>Responsible for controlling the speed of the motor driving the indexer to transfer game pieces
 * from the intake to the shooter.
 */
public class Indexer extends SubsystemBase {
  private final IndexerIO io;
  private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

  /** High-level goals for the indexer. */
  public enum Goal {
    /** Stop the indexer motor. */
    STOP,
    /** Run the indexer to move game pieces toward the shooter. */
    RUNNING
  }

  @AutoLogOutput private Goal goal = Goal.STOP;

  private final Alert disconnectedAlert = new Alert("Indexer IO disconnected", AlertType.kWarning);

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

  /** Updates hardware inputs, logs data, and updates status alerts. */
  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Indexer", inputs);
    disconnectedAlert.set(!inputs.connected);

    switch (goal) {
      case RUNNING -> io.setVoltage(DEFAULT_VOLTAGE);
      case STOP -> io.setVoltage(Volts.zero());
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
   * Returns a command to stop the indexer.
   *
   * @return The stop command.
   */
  public Command stop() {
    return this.runOnce(() -> setGoal(Goal.STOP));
  }
}
