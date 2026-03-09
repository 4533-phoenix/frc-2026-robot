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
  private ShooterState lastTargetState = new ShooterState(RadiansPerSecond.of(0), Degrees.of(0));

  private AngularVelocity targetVelocity = RadiansPerSecond.zero();

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
            () ->
                Math.abs(getFlywheelErrorRadPerSec())
                    <= flywheelAngularTolerance.in(RadiansPerSecond));
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
        targetVelocity = RadiansPerSecond.zero();
        flywheelIO.stop();
        hoodIO.retract();
      }
      case RUNNING -> setTargetState(lastTargetState);
    }
  }

  /**
   * Commands the shooter mechanisms to match a calculated state.
   *
   * @param state The desired {@link ShooterState} containing target flywheel speed and hood angle.
   */
  private void setTargetState(ShooterState state) {
    targetVelocity = state.flywheelSpeed();
    flywheelIO.setAngularVelocity(state.flywheelSpeed());
    hoodIO.setLength(convertHoodAngleToServoLength(state.hoodAngle()));
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
    Angle plateAngle = hoodAngle.plus(crankTangentToLaunchAngle);
    Angle internalTheta = mechanismTotalAngle.minus(plateAngle);

    double a = groundLinkDistance.in(Inches);
    double b = crankArmLength.in(Inches);
    double cosTheta = Math.cos(internalTheta.in(Radians));

    // Law of cosines: c^2 = a^2 + b^2 - 2ab*cos(C)
    double servoLengthSquared = (a * a) + (b * b) - (2 * a * b * cosTheta);
    double servoLength = Math.sqrt(Math.max(0, servoLengthSquared));

    return Inches.of(
        MathUtil.clamp(servoLength, servoMinLength.in(Inches), servoMaxLength.in(Inches)));
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
    return targetVelocity.in(RadiansPerSecond) - flywheelInputs.velocity.in(RadiansPerSecond);
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

  /**
   * Sets the target state for aiming (flywheel speed and hood angle).
   *
   * @param state The target shooter state.
   */
  public void setAimingParameters(ShooterState state) {
    this.lastTargetState = state;
  }
}
