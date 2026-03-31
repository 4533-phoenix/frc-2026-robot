// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.services.control;

import edu.wpi.first.wpilibj.GenericHID;

/** Represents a control profile for a physical HID device. */
public interface ControlProfile {
  /**
   * Returns the physical HID device.
   *
   * @return The GenericHID device.
   */
  GenericHID getHID();

  /**
   * Returns the left rumble value for the controller.
   *
   * @return The left rumble intensity (0.0-1.0).
   */
  double getLeftRumble();

  /**
   * Returns the right rumble value for the controller.
   *
   * @return The right rumble intensity (0.0-1.0).
   */
  double getRightRumble();

  /**
   * Returns whether or not the controller is connected.
   *
   * @return True if the controller is connected, false otherwise.
   */
  boolean isConnected();
}
