// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  private final Alert disconnectedAlert = new Alert("Shooter IO disconnected", AlertType.kWarning);

  public Shooter(ShooterIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
    disconnectedAlert.set(!inputs.flywheelConnected);
  }

  public void runFlywheel(double volts) {
    io.setFlywheelVolts(volts);
  }

  public void stopFlywheel() {
    io.setFlywheelVolts(0.0);
  }

  public void setHoodPosition(double position) {
    io.setHoodPosition(position);
  }
}
