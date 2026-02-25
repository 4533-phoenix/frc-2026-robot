package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.climb.Climb;

public class ClimbCommands {
  public static Command liftUp(Climb climb) {
    return Commands.race(
        Commands.runEnd(climb::startLiftUp, climb::stopLift, climb),
        Commands.waitUntil(climb::liftUpperLimit));
  }

  public static Command liftDown(Climb climb) {
    return Commands.race(
        Commands.runEnd(climb::startLiftDown, climb::stopLift, climb),
        Commands.waitUntil(climb::liftLowerLimit));
  }
}
