// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.intake.spinner;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.intake.spinner.SpinnerConstants.*;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Spinner extends SubsystemBase {
  private final SpinnerIO io;
  private final SpinnerIOInputsAutoLogged inputs = new SpinnerIOInputsAutoLogged();

  // Alerts for hardware monitoring
  private final Alert spinnerDisconnectedAlert =
      new Alert("Intake spinner motor disconnected", AlertType.kWarning);

  public Spinner(SpinnerIO io) {
    this.io = io;
  }

  /** Updates hardware inputs, logs data, and updates status alerts. */
  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Spinner", inputs);

    spinnerDisconnectedAlert.set(!inputs.connected);
  }

  /** Spins the spinner rollers to bring game pieces into the robot. */
  public void intake() {
    io.setVoltage(intakeVoltage);
  }

  /** Spins the spinner rollers in reverse to eject game pieces. */
  public void extake() {
    io.setVoltage(extakeVoltage);
  }

  /** Stops the spinner rollers. */
  public void stop() {
    io.setVoltage(Volts.of(0.0));
  }
}
