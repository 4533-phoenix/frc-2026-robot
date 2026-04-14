// Copyright (c) 2026 FRC Team 4533 (Phoenix)
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.services.control;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;

/** Abstract base class for control profiles using a CommandXboxController. */
public abstract class GenericControlProfile implements ControlProfile {
  /** The Xbox controller used for input. */
  protected final XboxController controller;

  /**
   * Constructs a GenericControlProfile.
   *
   * @param controller The Xbox controller used for input.
   */
  public GenericControlProfile(XboxController controller) {
    this.controller = controller;
  }

  @Override
  public GenericHID getHID() {
    return controller;
  }

  @Override
  public double getLeftRumble() {
    return 0.0;
  }

  @Override
  public double getRightRumble() {
    return 0.0;
  }

  @Override
  public boolean isConnected() {
    return controller.isConnected();
  }
}
