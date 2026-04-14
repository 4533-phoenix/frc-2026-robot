// Copyright (c) 2026 FRC Team 4533 (Phoenix)
//
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.services.control.operator;

import org.littletonrobotics.junction.AutoLog;

/** Interface for operator input/output hardware abstraction. */
public interface OperatorIO {
  /** Container for operator input values. */
  @AutoLog
  public static class OperatorIOInputs {
    /** Whether the operator wants to deploy the arm. */
    public boolean armDeployment;

    /** Whether the operator wants to retract the arm. */
    public boolean armRetraction;

    /** Whether the operator wants to intake. */
    public boolean intake;

    /** Whether the operator wants to extake. */
    public boolean extake;

    /** Whether the operator wants to climb. */
    public boolean climb;

    /** Whether the operator wants to raise the climber. */
    public boolean climberUp;

    /** Whether the operator wants to lower the climber. */
    public boolean climberDown;

    /** Whether the operator is connected. */
    public boolean connected;
  }

  /**
   * Updates the inputs based on the active profile.
   *
   * @param inputs The inputs object to populate.
   * @param profile The active operator profile.
   */
  public default void updateInputs(OperatorIOInputs inputs, OperatorProfile profile) {
    if (profile == null) return;

    inputs.armDeployment = profile.wantsArmDeployment();
    inputs.armRetraction = profile.wantsArmRetraction();
    inputs.intake = profile.wantsIntake();
    inputs.extake = profile.wantsExtake();
    inputs.climb = profile.wantsClimb();
    inputs.climberUp = profile.wantsClimberUp();
    inputs.climberDown = profile.wantsClimberDown();
    inputs.connected = profile.isConnected();
  }
}
