// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.services.control.operator;

import frc.robot.services.control.ControlProfile;

/** Represents an operator profile for controlling the robot superstructure. */
public interface OperatorProfile extends ControlProfile {
  /**
   * Returns true if the operator wants to deploy the arm.
   *
   * @return True if arm deployment is requested.
   */
  boolean wantsArmDeployment();

  /**
   * Returns true if the operator wants to retract the arm.
   *
   * @return True if arm retraction is requested.
   */
  boolean wantsArmRetraction();

  /**
   * Returns true if the operator wants to intake.
   *
   * @return True if intake is requested.
   */
  boolean wantsIntake();

  /**
   * Returns true if the operator wants to extake.
   *
   * @return True if extake is requested.
   */
  boolean wantsExtake();

  /**
   * Returns true if the operator wants to climb.
   *
   * @return True if climb is requested.
   */
  boolean wantsClimb();

  /**
   * Returns true if the operator wants to raise the climber.
   *
   * @return True if climber up is requested.
   */
  boolean wantsClimberUp();

  /**
   * Returns true if the operator wants to lower the climber.
   *
   * @return True if climber down is requested.
   */
  boolean wantsClimberDown();
}
