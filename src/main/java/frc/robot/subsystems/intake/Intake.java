// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.intake.IntakeConstants.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/**
 * Subsystem for controlling the robot's intake mechanism.
 *
 * <p>Responsible for deploying/retracting the intake arm and controlling the rollers to pull in or
 * push out game pieces.
 */
public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  // Target the user requested but not necessarily sent to the hardware yet
  private Angle pendingTarget = null;
  // Last target that was actually sent to the IO layer
  private Angle lastSentTarget = null;

  // Alerts for hardware monitoring
  private final Alert armDisconnectedAlert =
      new Alert("Intake arm motor disconnected", AlertType.kWarning);
  private final Alert spinnerDisconnectedAlert =
      new Alert("Intake spinner motor disconnected", AlertType.kWarning);

  /**
   * Creates a new Intake subsystem.
   *
   * @param io The abstraction layer for the intake hardware.
   */
  public Intake(IntakeIO io) {
    this.io = io;
  }

  /** Updates hardware inputs, logs data, and updates status alerts. */
  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    // Update connection alerts based on hardware feedback
    armDisconnectedAlert.set(!inputs.armConnected);
    spinnerDisconnectedAlert.set(!inputs.spinnerConnected);

    // If we have a pending arm target and the hardware is connected, ensure the
    // IO layer receives the setpoint. This handles the case where setSetpoint()
    // was called before the hardware/configuration was ready.
    if (pendingTarget != null && inputs.armConnected) {
      if (lastSentTarget == null || !pendingTarget.equals(lastSentTarget)) {
        io.setArmPosition(pendingTarget);
        lastSentTarget = pendingTarget;
      }
    }
  }

  /** Deploys the intake arm to the operational position. */
  public void deploy() {
    setSetpoint(armDeployedPosition);
  }

  /** Retracts the intake arm to the stowed position. */
  public void retract() {
    setSetpoint(armRetractedPosition);
  }

  /**
   * Sets the target position for the arm actuator.
   *
   * @param newTarget The target angle for the arm.
   */
  private void setSetpoint(Angle newTarget) {
    // Only update pending target if different
    if (pendingTarget == null || !newTarget.equals(pendingTarget)) {
      pendingTarget = newTarget;
      // Attempt an immediate send if hardware already connected. Otherwise the
      // periodic() resend logic will deliver the setpoint once the hardware is
      // ready.
      if (inputs.armConnected) {
        io.setArmPosition(newTarget);
        lastSentTarget = newTarget;
      } else {
        lastSentTarget = null;
      }
    }
  }

  /** Spins the spinner rollers to bring game pieces into the robot. */
  public void intake() {
    io.setSpinnerVoltage(spinnerIntakeVoltage);
  }

  /** Spins the spinner rollers in reverse to eject game pieces. */
  public void extake() {
    io.setSpinnerVoltage(spinnerExtakeVoltage);
  }

  /** Stops the spinner rollers. */
  public void stopSpinner() {
    io.setSpinnerVoltage(Volts.of(0.0));
  }

  /**
   * Checks if the intake arm is close enough to the deployed position to start intaking.
   *
   * @return True if the arm is within tolerance of the deployed position.
   */
  public boolean armDeployed() {
    return armDeployedPosition.minus(inputs.armPosition).abs(Degrees)
        < armPositionIntakeTolerance.in(Degrees);
  }

  /**
   * Checks if the intake arm is close enough to the retracted position.
   *
   * @return True if the arm is within tolerance of the retracted position.
   */
  public boolean armRetracted() {
    return armRetractedPosition.minus(inputs.armPosition).abs(Degrees)
        < armPositionPIDTolerance.in(Degrees);
  }
}
