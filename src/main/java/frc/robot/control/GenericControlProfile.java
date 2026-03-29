// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.control;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public abstract class GenericControlProfile implements ControlProfile {
  protected final CommandXboxController controller;

  public GenericControlProfile(CommandXboxController controller) {
    this.controller = controller;
  }

  @Override
  public GenericHID getHID() {
    return controller.getHID();
  }

  @Override
  public double getLeftRumble() {
    return 0.0;
  }

  @Override
  public double getRightRumble() {
    return 0.0;
  }
}
