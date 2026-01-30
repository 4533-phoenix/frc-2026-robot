// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;

/** IO interface for the climb hardware. */
public interface ClimbIO {
  @AutoLog
  public static class ClimbIOInputs {
    // Lift mechanism
    public boolean liftConnected = false;
    public double liftAppliedVolts = 0.0;
    public double liftCurrentAmps = 0.0;

    // Rotate mechanism
    public boolean rotateConnected = false;
    public double rotateAppliedVolts = 0.0;
    public double rotateCurrentAmps = 0.0;

    // Limit switches
    public boolean liftLowerLimit = false;
    public boolean liftUpperLimit = false;
    public boolean rotateMinLimit = false;
    public boolean rotateMaxLimit = false;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ClimbIOInputs inputs) {}

  /** Run the lift motors at the specified open loop voltage. */
  public default void setLiftOpenLoop(double volts) {}

  /** Run the rotate motors at the specified open loop voltage. */
  public default void setRotateOpenLoop(double volts) {}
}
