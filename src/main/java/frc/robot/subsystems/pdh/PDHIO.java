// Copyright (c) 2026 FRC Team 4533 (Phoenix)
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.pdh;

import org.littletonrobotics.junction.AutoLog;

/** IO interface for the PDH subsystem. */
public interface PDHIO {
  /** Loggable inputs for the PDH subsystem. */
  @AutoLog
  public static class PDHIOInputs {
    /** Whether the pdh is currently connected and communicating. */
    public boolean connected = false;

    /** Whether the pdh is functioning correctly. */
    public boolean healthy = true;

    /** The full status of the pdh. */
    public int[] status = new int[] {0, 0};
  }

  /**
   * Updates the set of loggable inputs.
   *
   * @param inputs The inputs object to update.
   */
  public default void updateInputs(PDHIOInputs inputs) {}

  /** Clears all faults and warnings. */
  public default void clearFaults() {}
}
