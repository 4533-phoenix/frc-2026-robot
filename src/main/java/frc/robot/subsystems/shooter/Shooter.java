// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.shooter.ShooterConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.shooter.flywheel.FlywheelIO;
import frc.robot.subsystems.shooter.flywheel.FlywheelIOInputsAutoLogged;
import frc.robot.subsystems.shooter.hood.HoodIO;
import frc.robot.subsystems.shooter.hood.HoodIOInputsAutoLogged;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * Subsystem for controlling the robot's shooter mechanism.
 *
 * <p>Responsible for controlling the angular velocity of the flywheels and the position of the
 * adjustable hood to regulate launch angle and distance.
 */
public class Shooter extends SubsystemBase {
  /**
   * Represents the desired physical state of the shooter subsystem.
   *
   * <p>This record encapsulates the target speed for the flywheels and the target angle for the
   * adjustable hood to achieve a specific shot.
   *
   * @param flywheelSpeed The target angular velocity of the shooter flywheels.
   * @param hoodAngle The target angle of the adjustable hood mechanism.
   */
  public record ShooterState(AngularVelocity flywheelSpeed, Angle hoodAngle) {}

  private final FlywheelIO flywheelIO;
  private final FlywheelIOInputsAutoLogged flywheelInputs = new FlywheelIOInputsAutoLogged();

  private final HoodIO hoodIO;
  private final HoodIOInputsAutoLogged hoodInputs = new HoodIOInputsAutoLogged();

  /** High-level goals for the shooter subsystem. */
  public enum Goal {
    /** Stop all movement and motors. */
    STOP,
    /** Active and tracking a target state. */
    RUNNING
  }

  @AutoLogOutput private Goal goal = Goal.STOP;
  private ShooterState state = DEFAULT_STATE;

  private final Trigger flywheelReadyTrigger;
  private final Trigger hoodReadyTrigger;
  private final Trigger readyToShootTrigger;

  private final Alert flywheelDisconnectedAlert =
      new Alert("Flywheel IO disconnected", AlertType.kWarning);

  /**
   * Creates a new Shooter subsystem.
   *
   * @param flywheelIO The abstraction layer for the flywheel hardware.
   * @param hoodIO The abstraction layer for the hood hardware.
   */
  public Shooter(FlywheelIO flywheelIO, HoodIO hoodIO) {
    this.flywheelIO = flywheelIO;
    this.hoodIO = hoodIO;

    // Build triggers once; lambdas capture 'this' and evaluate live state each poll
    flywheelReadyTrigger =
        new Trigger(
            () -> Math.abs(getFlywheelErrorRadPerSec()) <= ANGULAR_TOLERANCE.in(RadiansPerSecond));
    hoodReadyTrigger = new Trigger(() -> hoodInputs.atSetpoint);
    readyToShootTrigger =
        flywheelReadyTrigger.and(hoodReadyTrigger).and(() -> goal == Goal.RUNNING);
  }

  /**
   * Sets the high-level goal for the shooter.
   *
   * @param goal The target goal.
   */
  public void setGoal(Goal goal) {
    this.goal = goal;
  }

  /** Updates hardware inputs, logs data, and updates status alerts. */
  @Override
  public void periodic() {
    flywheelIO.updateInputs(flywheelInputs);
    Logger.processInputs("Shooter/Flywheel", flywheelInputs);
    flywheelDisconnectedAlert.set(!flywheelInputs.connected);

    hoodIO.updateInputs(hoodInputs);
    Logger.processInputs("Shooter/Hood", hoodInputs);

    switch (goal) {
      case STOP -> {
        flywheelIO.stop();
        hoodIO.retract();
      }
      case RUNNING -> {
        flywheelIO.setAngularVelocity(state.flywheelSpeed());
        hoodIO.setLength(convertHoodAngleToServoLength(state.hoodAngle()));
      }
    }
  }

  /**
   * Sets the target state for aiming (flywheel speed and hood angle).
   *
   * @param state The target shooter state.
   */
  public void setShooterState(ShooterState state) {
    if (state == null) {
      this.state = DEFAULT_STATE;
      DriverStation.reportWarning(
          "Attempted to set shooter state to null. Defaulting to safe state.", false);
    } else {
      this.state = state;
    }
  }

  /**
   * Converts the desired Hood Angle to Servo Length based on the physical mechanism. Uses the Law
   * of Cosines to determine the required servo length.
   *
   * @param hoodAngle The desired launch angle.
   * @return The required length for the hood actuator.
   */
  private static Distance convertHoodAngleToServoLength(Angle hoodAngle) {
    // Kinematic calculation for the hood mechanism
    Angle plateAngle = hoodAngle.plus(CRANK_TANGENT_TO_LAUNCH_ANGLE);
    Angle internalTheta = HOOD_TOTAL_ANGLE.minus(plateAngle);

    double a = GROUND_LINK_DISTANCE.in(Inches);
    double b = CRANK_ARM_LENGTH.in(Inches);
    double cosTheta = Math.cos(internalTheta.in(Radians));

    // Law of cosines: c^2 = a^2 + b^2 - 2ab*cos(C)
    double servoLengthSquared = (a * a) + (b * b) - (2 * a * b * cosTheta);
    double servoLength = Math.sqrt(Math.max(0, servoLengthSquared));

    return Inches.of(
        MathUtil.clamp(servoLength, SERVO_MIN_LENGTH.in(Inches), SERVO_MAX_LENGTH.in(Inches)));
  }

  /**
   * Checks if the shooter is ready to launch a game piece.
   *
   * @return True if the flywheels are spun up, the hood is in position, and the shooter is active.
   */
  public Trigger isShooterReady() {
    return readyToShootTrigger;
  }

  /**
   * Returns a trigger that is true when the flywheel velocity is within tolerance.
   *
   * @return The flywheel ready trigger.
   */
  public Trigger isFlywheelReady() {
    return flywheelReadyTrigger;
  }

  /**
   * Returns a trigger that is true when the hood actuator has reached its setpoint.
   *
   * @return The hood ready trigger.
   */
  public Trigger isHoodReady() {
    return hoodReadyTrigger;
  }

  /**
   * Returns the flywheel velocity error in radians per second (target minus actual).
   *
   * @return The flywheel velocity error in radians per second.
   */
  public double getFlywheelErrorRadPerSec() {
    return state.flywheelSpeed().in(RadiansPerSecond)
        - flywheelInputs.velocity.in(RadiansPerSecond);
  }

  /**
   * Safely stops the flywheels and retracts the hood.
   *
   * @return A command to stop the shooter.
   */
  public Command stop() {
    return this.runOnce(() -> setGoal(Goal.STOP));
  }

  /**
   * Runs the shooter while the command is held.
   *
   * @return A command to run the shooter while held.
   */
  public Command runHeld() {
    return this.startEnd(() -> setGoal(Goal.RUNNING), () -> setGoal(Goal.STOP));
  }

  /**
   * Sets the shooter goal to RUNNING.
   *
   * @return A command to start the shooter.
   */
  public Command run() {
    return this.runOnce(() -> setGoal(Goal.RUNNING));
  }

  /** Convenience method to set the running goal directly. */
  public void setRunning() {
    setGoal(Goal.RUNNING);
  }

  /** Convenience method to set the stop goal directly. */
  public void setStop() {
    setGoal(Goal.STOP);
  }
}
