// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

/**
 * Interface for the indexer mechanism input/output abstraction.
 *
 * <p>This interface allows for interchangeable indexer hardware (e.g., different motor controllers)
 * and comprehensive simulation support.
 */
public interface IndexerIO {
  /** Contains all of the inputs received from the indexer hardware. */
  @AutoLog
  public static class IndexerIOInputs {
    /** Whether the indexer motor controller is currently connected and communicating. */
    public boolean connected = false;

    /** The voltage currently being applied to the indexer motor. */
    public Voltage appliedVoltage = Volts.zero();

    /** The current being drawn by the indexer motor. */
    public Current appliedCurrent = Amps.zero();

    /** Whether the indexer motor is functioning correctly. */
    public boolean healthy = true;

    /** The full status of the indexer motor controller. */
    public int[] status = new int[] {0, 0, 0, 0};
  }

  /**
   * Updates the set of loggable inputs.
   *
   * @param inputs The inputs object to update.
   */
  public default void updateInputs(IndexerIOInputs inputs) {}

  /**
   * Run the indexer motor at the specified voltage.
   *
   * @param voltage The voltage to apply to the motor controller.
   */
  public default void setVoltage(Voltage voltage) {}
}
