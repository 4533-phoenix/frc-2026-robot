package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.intake.Intake;

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
   * @param intake The intake subsystem, used to check that the arm is retracted before extending.
   * @return A command that waits for the intake to retract, then runs the lift upward until the
   *     upper limit is reached.
   */
  public static Command liftUp(Climb climb, Intake intake) {
    return Commands.sequence(
        // Block until the intake arm is confirmed retracted
        Commands.waitUntil(intake::armRetracted),
        Commands.race(
            // Run the motor up, and stop it when the command ends
            Commands.runEnd(climb::startLiftUp, climb::stopLift, climb),
            // End the race when the limit switch is triggered
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
        // Run the motor down, and stop it when the command ends
        Commands.runEnd(climb::startLiftDown, climb::stopLift, climb),
        // End the race when the limit switch is triggered
        Commands.waitUntil(climb::liftLowerLimit));
  }
}
