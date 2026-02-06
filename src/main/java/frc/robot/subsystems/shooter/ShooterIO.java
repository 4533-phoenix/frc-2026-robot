// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    public boolean flywheelConnected = false;
    public double flywheelPositionRad = 0.0;
    public double flywheelVelocityRadPerSec = 0.0;
    public double flywheelAppliedVolts = 0.0;
    public double flywheelCurrentAmps = 0.0;

    public double hoodPosition = 0.0;
  }

  public default void updateInputs(ShooterIOInputs inputs) {}

  public default void setFlywheelVolts(double volts) {}

  public default void setHoodPosition(double position) {}
}
