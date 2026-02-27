// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.indexer.Indexer;

/**
 * Factory class for creating commands related to the indexer subsystem.
 *
 * <p>Provides methods to control the conveyor rollers that transport game pieces
 * from the intake to the shooter.
 */
public class IndexerCommands {

  private IndexerCommands() {}

  /**
   * Runs the indexer continuously, and stops the motor when the command ends or is interrupted.
   *
   * @param indexer The indexer subsystem.
   * @return A command that runs the indexer.
   */
  public static Command runIndexer(Indexer indexer) {
    return Commands.runEnd(indexer::run, indexer::stop, indexer);
  }

  /**
   * Continuously ensures the indexer is stopped. Useful as a default command.
   *
   * @param indexer The indexer subsystem.
   * @return A command that stops the indexer.
   */
  public static Command stopIndexer(Indexer indexer) {
    return Commands.run(indexer::stop, indexer);
  }
}
