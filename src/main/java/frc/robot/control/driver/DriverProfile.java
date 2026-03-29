// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.control.driver;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.control.ControlProfile;

/** Represents a driver profile for controlling the robot drivetrain. */
public interface DriverProfile extends ControlProfile {
  /**
   * Returns the desired chassis speeds based on joystick input.
   *
   * @return The desired {@link ChassisSpeeds}.
   */
  ChassisSpeeds getDesiredSpeeds();

  /**
   * Returns whether the driver wants to aim.
   *
   * @return True if aiming is requested.
   */
  boolean wantsAim();

  /**
   * Returns whether the driver wants to shoot.
   *
   * @return True if shooting is requested.
   */
  boolean wantsShoot();

  /**
   * Returns whether the driver wants to reset the robot pose.
   *
   * @return True if reset is requested.
   */
  boolean wantsReset();
}
