package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.climb.Climb;

/**
 * Factory class for creating commands related to the climbing subsystem.
 *
 * <p>Uses WPILib's Command composition factory methods to create safe, race-conditioned commands
 * that stop when limit switches are triggered.
 */
public class ClimbCommands {
  public static Command liftUp(Climb climb) {
    return Commands.sequence(
        Commands.race(
            Commands.runEnd(climb::startLiftUp, climb::stopLift, climb),
            Commands.waitUntil(climb::liftUpperLimit)));
  }

  public static Command liftDown(Climb climb) {
    return Commands.race(
        Commands.runEnd(climb::startLiftDown, climb::stopLift, climb),
        Commands.waitUntil(climb::liftLowerLimit));
  }
}
