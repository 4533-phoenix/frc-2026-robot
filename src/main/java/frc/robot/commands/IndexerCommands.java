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

public class IndexerCommands {

  private IndexerCommands() {}

  /** Runs the indexer continuously, and stops the motor when the command ends or is interrupted. */
  public static Command runIndexer(Indexer indexer) {
    return Commands.runEnd(indexer::run, indexer::stop, indexer);
  }

  /** Continuously ensures the indexer is stopped. Useful as a default command. */
  public static Command stopIndexer(Indexer indexer) {
    return Commands.run(indexer::stop, indexer);
  }
}
