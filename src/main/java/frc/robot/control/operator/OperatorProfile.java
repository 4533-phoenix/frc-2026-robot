// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.control.operator;

import frc.robot.control.ControlProfile;

public interface OperatorProfile extends ControlProfile {
  /** Returns true if the operator wants to deploy the arm. */
  boolean wantsArmDeployment();

  /** Returns true if the operator wants to retract the arm. */
  boolean wantsArmRetraction();

  /** Returns true if the operator wants to intake. */
  boolean wantsIntake();

  /** Returns true if the operator wants to extake. */
  boolean wantsExtake();

  /** Returns true if the operator wants to climb. */
  boolean wantsClimb();

  /** Returns true if the operator wants to raise the climber. */
  boolean wantsClimberUp();

  /** Returns true if the operator wants to lower the climber. */
  boolean wantsClimberDown();
}
