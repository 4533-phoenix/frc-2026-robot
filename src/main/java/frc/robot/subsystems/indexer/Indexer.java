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

  public enum Goal {
    STOP,
    RUNNING
  }

  private Goal currentGoal = Goal.STOP;

  private final Alert disconnectedAlert = new Alert("Indexer IO disconnected", AlertType.kWarning);

  /**
   * Creates a new Indexer subsystem.
   *
   * @param io The abstraction layer for the indexer hardware.
   */
  public Indexer(IndexerIO io) {
    this.io = io;
  }

  public void setGoal(Goal goal) {
    this.currentGoal = goal;
  }

  /** Updates hardware inputs, logs data, and updates status alerts. */
  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Indexer", inputs);
    disconnectedAlert.set(!inputs.connected);

    switch (currentGoal) {
      case RUNNING -> io.setVoltage(indexerOnVoltage);
      case STOP -> io.setVoltage(Volts.zero());
    }

    Logger.recordOutput("Indexer/Goal", currentGoal.toString());
  }

  public Command run() {
    return this.runEnd(() -> setGoal(Goal.RUNNING), () -> setGoal(Goal.STOP));
  }

  public Command stop() {
    return this.runOnce(() -> setGoal(Goal.STOP));
  }
}
