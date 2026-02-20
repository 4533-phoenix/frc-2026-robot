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

public class IntakeCommands {
  private IntakeCommands() {}

  /** Deploys the intake and runs the spinner at intake voltage. */
  public static Command deploy(Intake intake) {
    return Commands.runEnd(
        () -> {
          intake.deploy();
          if (intake.armDeployed()) {
            intake.intake();
          }
        },
        intake::stopSpinner);
  }

  /** Continuously holds the intake at the retracted position. Useful as a default command. */
  public static Command holdRetracted(Intake intake) {
    return Commands.run(
        () -> {
          intake.retract();
          intake.stopSpinner();
        },
        intake);
  }
}
