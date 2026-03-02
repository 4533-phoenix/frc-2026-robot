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
 * <p>The intake arm deploys once and stays deployed for the entire match. These commands only
 * control the spinner rollers. Arm retraction is handled exclusively by the climbing transition.
 */
public class IntakeCommands {
  private IntakeCommands() {}

  /**
   * Runs the intake spinners inward to collect game pieces. The spinners stop when the command
   * ends.
   *
   * @param intake The intake subsystem.
   * @return A command that runs the spinners inward while held.
   */
  public static Command runSpinnersIn(Intake intake) {
    return Commands.runEnd(intake::intake, intake::stopSpinner, intake);
  }

  /**
   * Runs the intake spinners outward to eject game pieces. The spinners stop when the command ends.
   *
   * @param intake The intake subsystem.
   * @return A command that runs the spinners outward while held.
   */
  public static Command runSpinnersOut(Intake intake) {
    return Commands.runEnd(intake::extake, intake::stopSpinner, intake);
  }

  /**
   * Holds the intake arm at the deployed position with spinners off. Intended as the default
   * command for the intake subsystem during normal match play.
   *
   * @param intake The intake subsystem.
   * @return A command that continuously holds the arm deployed.
   */
  public static Command holdDeployed(Intake intake) {
    return Commands.run(
        () -> {
          intake.deploy();
          intake.stopSpinner();
        },
        intake);
  }

  /**
   * Holds the intake arm at the retracted position with spinners off. Used only during the climbing
   * state to clear the arm for the climb mechanism.
   *
   * @param intake The intake subsystem.
   * @return A command that continuously holds the arm retracted.
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
