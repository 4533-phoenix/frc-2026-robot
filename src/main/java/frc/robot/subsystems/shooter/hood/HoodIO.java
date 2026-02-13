// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.shooter.hood;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.units.measure.Distance;
import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {
  @AutoLog
  public static class HoodIOInputs {
    public Distance currentLength = Inches.of(0);
    public Distance targetLength = Inches.of(0);
    public boolean atSetpoint = false;
  }

  public default void updateInputs(HoodIOInputs inputs) {}

  public default void setLength(Distance length) {}
}
