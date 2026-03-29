// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.control;

import edu.wpi.first.wpilibj.GenericHID;

public interface ControlProfile {
  /** Returns the physical HID device. */
  GenericHID getHID();

  /** Feedback values. */
  double getLeftRumble();

  double getRightRumble();
}
