// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.shooter.flywheel;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {
  @AutoLog
  public static class FlywheelIOInputs {
    public boolean connected = false;
    public AngularVelocity velocity = RadiansPerSecond.of(0.0);
    public Voltage appliedVoltage = Volts.of(0.0);
    public Current appliedCurrent = Amps.of(0.0);
  }

  public default void updateInputs(FlywheelIOInputs inputs) {}

  public default void setAngularVelocity(AngularVelocity velocity) {}

  public default void stop() {}
}
