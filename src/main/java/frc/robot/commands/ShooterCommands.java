// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.shooter.Shooter;

/**
 * Factory class for creating commands related to the shooter subsystem.
 *
 * <p>Provides methods to control the speed of the shooter flywheels and kicker rollers.
 */
public class ShooterCommands {

  private ShooterCommands() {}

  /**
   * Continuously ensures the shooter is stopped. Useful as a default command to
   * ensure safety when not actively shooting.
   *
   * @param shooter The shooter subsystem.
   * @return A command that stops the shooter motors.
   */
  public static Command stopShooter(Shooter shooter) {
    return Commands.run(shooter::stop, shooter);
  }
}
