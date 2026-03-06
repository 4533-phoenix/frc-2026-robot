// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.intake.spinner;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

/**
 * IO interface for the intake hardware abstraction.
 *
 * <p>This interface allows for interchangeable intake hardware (e.g., NEO, TalonFX) and
 * comprehensive simulation support by standardizing how the intake arm position and spinner
 * velocity are set and monitored.
 */
public interface SpinnerIO {
  /** Contains all of the inputs received from the intake hardware. */
  @AutoLog
  public static class SpinnerIOInputs {
    // ---------- Spinner Motor Inputs ----------
    /** Whether the spinner motor controller is connected. */
    public boolean connected = false;
    /** The current angular velocity of the intake spinner rollers. */
    public AngularVelocity velocity = RadiansPerSecond.of(0.0);
    /** The voltage currently being applied to the spinner motor. */
    public Voltage appliedVoltage = Volts.of(0.0);
    /** The current drawn by the spinner motor. */
    public Current appliedCurrent = Amps.of(0.0);
  }

  /**
   * Updates the set of loggable inputs with the latest data from the intake hardware.
   *
   * @param inputs The inputs object to update.
   */
  public default void updateInputs(SpinnerIOInputs inputs) {}

  /**
   * Commands the spinner motor to run at a specific voltage.
   *
   * @param voltage The target voltage for the spinner rollers.
   */
  public default void setVoltage(Voltage voltage) {}
}
