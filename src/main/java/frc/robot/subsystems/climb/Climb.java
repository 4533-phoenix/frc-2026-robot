// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.climb.ClimbConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.monitors.SparkHealthMonitor;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * Subsystem for the robot's climb mechanism.
 *
 * <p>Handles controlling the lift motor voltage and monitoring limit switches to prevent
 * over-extension or damage to the mechanism.
 */
public class Climb extends SubsystemBase {
  private final ClimbIO io;
  private final ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();
  private final SparkHealthMonitor monitor = new SparkHealthMonitor("Climb");

  /** High-level goals for the climb subsystem. */
  public enum Goal {
    /** Stop the climb motor. */
    STOP,
    /** Extend the climber upwards. */
    UP,
    /** Retract the climber downwards. */
    DOWN
  }

  @AutoLogOutput private Goal goal = Goal.STOP;

  private final Trigger upTrigger;
  private final Trigger downTrigger;

  /**
   * Creates a new Climb subsystem.
   *
   * @param io The abstraction layer for the climb hardware.
   */
  public Climb(ClimbIO io) {
    this.io = io;

    upTrigger = new Trigger(() -> inputs.upperLimit);
    downTrigger = new Trigger(() -> inputs.lowerLimit);
  }

  /**
   * Sets the current requested goal for the climber.
   *
   * @param goal The target goal for the climber.
   */
  public void setGoal(Goal goal) {
    this.goal = goal;
  }

  /** Updates hardware inputs, logs data, and updates status alerts. */
  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climb", inputs);
    monitor.update(inputs.connected, inputs.status);

    switch (goal) {
      case UP -> {
        if (upTrigger.getAsBoolean()) io.setLiftVoltage(Volts.zero());
        else io.setLiftVoltage(DEFAULT_VOLTAGE);
      }
      case DOWN -> {
        if (downTrigger.getAsBoolean()) io.setLiftVoltage(Volts.zero());
        else io.setLiftVoltage(DEFAULT_VOLTAGE.unaryMinus());
      }
      case STOP -> io.setLiftVoltage(Volts.zero());
    }
  }

  /**
   * Returns a trigger that is true when the upper limit switch is triggered.
   *
   * @return The upper limit trigger.
   */
  public Trigger isUp() {
    return upTrigger;
  }

  /**
   * Returns a trigger that is true when the lower limit switch is triggered.
   *
   * @return The lower limit trigger.
   */
  public Trigger isDown() {
    return downTrigger;
  }

  /**
   * Returns a command to raise the climber until it hits the upper limit.
   *
   * @return The raise command.
   */
  public Command raise() {
    return this.startEnd(() -> setGoal(Goal.UP), () -> setGoal(Goal.STOP)).until(upTrigger);
  }

  /**
   * Returns a command to lower the climber until it hits the lower limit.
   *
   * @return The lower command.
   */
  public Command lower() {
    return this.startEnd(() -> setGoal(Goal.DOWN), () -> setGoal(Goal.STOP)).until(downTrigger);
  }

  /**
   * Returns a command to stop the climber.
   *
   * @return The stop command.
   */
  public Command stop() {
    return this.runOnce(() -> setGoal(Goal.STOP));
  }

  /**
   * Returns whether or not the subsystem is healthy
   *
   * @return True if the subsystem is healthy, false otherwise.
   */
  public boolean isHealthy() {
    return inputs.healthy && inputs.connected;
  }

  /** Clears all faults and warnings. */
  public void clearFaults() {
    io.clearFaults();
  }
}
