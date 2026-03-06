// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.intake.arm;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.intake.arm.ArmConstants.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.junction.Logger;

/**
 * Subsystem for controlling the robot's intake mechanism.
 *
 * <p>Responsible for deploying/retracting the intake arm and controlling the rollers to pull in or
 * push out game pieces.
 */
public class Arm extends SubsystemBase {
  private final ArmIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

  // Alerts for hardware monitoring
  private final Alert armDisconnectedAlert =
      new Alert("Intake arm motor disconnected", AlertType.kWarning);

  // Cached Triggers — created once, lambdas evaluate live state each poll
  private final Trigger deployedTrigger;
  private final Trigger retractedTrigger;

  /**
   * Creates a new Arm subsystem.
   *
   * @param io The abstraction layer for the arm hardware.
   */
  public Arm(ArmIO io) {
    this.io = io;

    deployedTrigger =
        new Trigger(
            () ->
                deployedPosition.minus(inputs.position).abs(Degrees) < intakeTolerance.in(Degrees));
    retractedTrigger =
        new Trigger(
            () ->
                retractedPosition.minus(inputs.position).abs(Degrees)
                    < intakeTolerance.in(Degrees));
  }

  /** Updates hardware inputs, logs data, and updates status alerts. */
  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);

    // Update connection alerts based on hardware feedback
    armDisconnectedAlert.set(!inputs.connected);
  }

  /** Deploys the intake arm to the operational position. */
  public void deploy() {
    setSetpoint(deployedPosition);
  }

  /** Retracts the intake arm to the stowed position. */
  public void retract() {
    setSetpoint(retractedPosition);
  }

  /**
   * Sets the target position for the arm actuator.
   *
   * @param newTarget The target angle for the arm.
   */
  private void setSetpoint(Angle newTarget) {
    io.setPosition(newTarget);
  }

  public Trigger isDeployed() {
    return deployedTrigger;
  }

  public Trigger isRetracted() {
    return retractedTrigger;
  }
}
