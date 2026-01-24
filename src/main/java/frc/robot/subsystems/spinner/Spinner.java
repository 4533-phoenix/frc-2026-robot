// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.spinner;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Spinner extends SubsystemBase {
  private final SpinnerIO spinnerIO;
  private final SpinnerIO.SpinnerIOInputs inputs = new SpinnerIO.SpinnerIOInputs();

  public Spinner(SpinnerIO spinnerIO) {
    this.spinnerIO = spinnerIO;
  }

  @Override
  public void periodic() {
    spinnerIO.updateInputs(inputs);
  }

  public void setVoltage(double volts) {
    spinnerIO.setVoltage(volts);
  }

  public void stop() {
    spinnerIO.setVoltage(0.0);
  }

  public boolean isSpinning() {
    return Math.abs(inputs.appliedVolts) > 0.01;
  }
}
