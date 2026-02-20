// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.intake.IntakeConstants.*;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();


  private final Alert armDisconnectedAlert =
      new Alert("Intake arm motor disconnected", AlertType.kWarning);
  private final Alert spinnerDisconnectedAlert =
      new Alert("Intake spinner motor disconnected", AlertType.kWarning);

  public Intake(IntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    armDisconnectedAlert.set(!inputs.armConnected);
    spinnerDisconnectedAlert.set(!inputs.spinnerConnected);
  }

  /** Deploy the intake arm and spin the rollers. */
  public void deploy() {
    io.setArmPosition(armDeployedPosition);
    io.setSpinnerVoltage(spinnerIntakeVoltage);
  }

  /** Retract the intake arm and stop the rollers. */
  public void retract() {
    io.setArmPosition(armRetractedPosition);
    io.setSpinnerVoltage(Volts.of(0.0));
  }

  @AutoLogOutput(key = "Intake/ArmPositionDeg")
  public double getArmPositionDeg() {
    return inputs.armPosition.in(Degrees);
  }
}
