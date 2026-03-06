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
  /**
   * Creates a command to lift the robot up until the upper limit switch is triggered.
   *
   * <p>The climb will not extend if the intake arm is not fully retracted. The command waits for
   * the intake to retract before starting the lift motor.
   *
   * @param climb The climbing subsystem.
   * @return A command that waits for the intake to retract, then runs the lift upward until the
   *     upper limit is reached.
   */
  public static Command liftUp(Climb climb) {
    return Commands.sequence(
        Commands.race(
            Commands.runEnd(climb::startLiftUp, climb::stopLift, climb),
            Commands.waitUntil(climb::liftUpperLimit)));
  }

  /**
   * Creates a command to lower the robot down until the lower limit switch is triggered.
   *
   * @param climb The climbing subsystem.
   * @return A command that runs the lift downward until the lower limit is reached.
   */
  public static Command liftDown(Climb climb) {
    return Commands.race(
        Commands.runEnd(climb::startLiftDown, climb::stopLift, climb),
        Commands.waitUntil(climb::liftLowerLimit));
  }
}
