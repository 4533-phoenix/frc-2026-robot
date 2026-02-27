// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.Intake;

/**
 * Factory class for creating commands related to the intake subsystem.
 *
 * <p>Provides methods to control the intake arm position and the spinner rollers for collecting or
 * ejecting game pieces.
 */
public class IntakeCommands {
  private IntakeCommands() {}

  /**
   * Deploys the intake arm and runs the spinner rollers for intaking.
   *
   * <p>The rollers will only spin if the arm is within the deployed tolerance to prevent damage.
   *
   * @param intake The intake subsystem.
   * @return A command that deploys the intake and activates the spinners.
   */
  public static Command deploy(Intake intake) {
    return Commands.runEnd(
        () -> {
          intake.deploy();
          // Safety check: only run rollers if the arm is actually down
          if (intake.armDeployed()) {
            intake.intake();
          }
        },
        intake::stopSpinner);
  }

  /**
   * Continuously holds the intake arm at the retracted position and stops the rollers. Useful as a
   * default command to ensure the intake is stowed.
   *
   * @param intake The intake subsystem.
   * @return A command to hold the intake retracted.
   */
  public static Command holdRetracted(Intake intake) {
    return Commands.run(
        () -> {
          intake.retract();
          intake.stopSpinner();
        },
        intake);
  }
}
