// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.shooter.hood;

import edu.wpi.first.math.MathUtil;

public class HoodIOSim implements HoodIO {
  private double position = 0.0;

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    inputs.position = position;
  }

  @Override
  public void setPosition(double position) {
    this.position = MathUtil.clamp(position, 0.0, 1.0);
  }
}
