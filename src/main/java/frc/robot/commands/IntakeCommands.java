// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.arm.Arm;
import frc.robot.subsystems.intake.spinner.Spinner;

/**
 * Factory class for creating commands related to the intake subsystems (arm and spinner).
 *
 * <p>The intake arm starts retracted and deploys on the first bumper press via the {@code
 * deployAndRunSpinners*} commands. Once deployed it stays deployed for the rest of the match (the
 * default command maintains it). Arm retraction is handled exclusively by the climbing state.
 *
 * <p>Because the arm and spinner are separate subsystems, spinner commands do not interfere with
 * the arm default command and vice-versa.
 */
public class IntakeCommands {
  private IntakeCommands() {}

  /**
   * Runs the intake spinners inward to collect game pieces. The spinners stop when the command
   * ends.
   *
   * @param spinner The spinner subsystem.
   * @return A command that runs the spinners inward while held.
   */
  public static Command intake(Spinner spinner) {
    return Commands.runEnd(spinner::intake, spinner::stopSpinner, spinner);
  }

  /**
   * Runs the intake spinners outward to eject game pieces. The spinners stop when the command ends.
   *
   * @param spinner The spinner subsystem.
   * @return A command that runs the spinners outward while held.
   */
  public static Command extake(Spinner spinner) {
    return Commands.runEnd(spinner::extake, spinner::stopSpinner, spinner);
  }

  /**
   * Holds the intake arm at the deployed position. Intended as the default command for the arm
   * subsystem during normal match play.
   *
   * @param arm The arm subsystem.
   * @return A command that continuously holds the arm deployed.
   */
  public static Command holdDeployed(Arm arm) {
    return Commands.run(arm::deploy, arm);
  }

  /**
   * Holds the intake arm at the retracted position. Used only during the climbing state to clear
   * the arm for the climb mechanism.
   *
   * @param arm The arm subsystem.
   * @return A command that continuously holds the arm retracted.
   */
  public static Command holdRetracted(Arm arm) {
    return Commands.run(arm::retract, arm);
  }
}
