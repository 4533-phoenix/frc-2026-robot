// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

/**
 * Interface for the climb mechanism input/output abstraction.
 *
 * <p>This interface allows for interchangeable climb hardware (e.g., different motor controllers)
 * and comprehensive simulation support.
 */
public interface ClimbIO {
  /** Contains all of the inputs received from the climb hardware. */
  @AutoLog
  public static class ClimbIOInputs {
    // Lift mechanism
    /** Whether the lift motor controller is currently connected and communicating. */
    public boolean connected = false;
    /** The voltage currently being applied to the lift motor. */
    public Voltage appliedVoltage = Volts.of(0.0);
    /** The current being drawn by the lift motor. */
    public Current appliedCurrent = Amps.of(0.0);

    // Limit switches
    /** Whether the lower limit switch is currently pressed. */
    public boolean lowerLimit = false;
    /** Whether the upper limit switch is currently pressed. */
    public boolean upperLimit = false;
  }

  /**
   * Updates the set of loggable inputs.
   *
   * @param inputs The inputs object to update.
   */
  public default void updateInputs(ClimbIOInputs inputs) {}

  /**
   * Run the lift motors at the specified voltage.
   *
   * @param voltage The voltage to apply to the motor controller.
   */
  public default void setLiftVoltage(Voltage voltage) {}
}
