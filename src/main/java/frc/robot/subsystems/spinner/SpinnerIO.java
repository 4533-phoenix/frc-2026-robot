// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.spinner;

import org.littletonrobotics.junction.AutoLog;

public interface SpinnerIO {
  @AutoLog
  public static class SpinnerIOInputs {
    public boolean connected = false;
    public double appliedVolts = 0.0;
  }

  public default void updateInputs(SpinnerIOInputs inputs) {}

  public default void setVoltage(double appliedVolts) {}
}
