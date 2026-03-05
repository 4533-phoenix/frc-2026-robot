// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
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
public interface IntakeIO {
  /** Contains all of the inputs received from the intake hardware. */
  @AutoLog
  public static class IntakeIOInputs {
    // ---------- Arm Motor Inputs ----------
    /** Whether the arm motor controller is connected. */
    public boolean armConnected = false;
    /** The current absolute position of the intake arm. */
    public Angle armPosition = Radians.of(0.0);
    /** The current angular velocity of the intake arm. */
    public AngularVelocity armVelocity = RadiansPerSecond.of(0.0);
    /** The voltage currently being applied to the arm motor. */
    public Voltage armAppliedVoltage = Volts.of(0.0);
    /** The current drawn by the arm motor. */
    public Current armAppliedCurrent = Amps.of(0.0);

    // ---------- Spinner Motor Inputs ----------
    /** Whether the spinner motor controller is connected. */
    public boolean spinnerConnected = false;
    /** The current angular velocity of the intake spinner rollers. */
    public AngularVelocity spinnerVelocity = RadiansPerSecond.of(0.0);
    /** The voltage currently being applied to the spinner motor. */
    public Voltage spinnerAppliedVoltage = Volts.of(0.0);
    /** The current drawn by the spinner motor. */
    public Current spinnerAppliedCurrent = Amps.of(0.0);
  }

  /**
   * Updates the set of loggable inputs with the latest data from the intake hardware.
   *
   * @param inputs The inputs object to update.
   */
  public default void updateInputs(IntakeIOInputs inputs) {}

  /**
   * Commands the arm motor to move to a specified position using closed-loop control.
   *
   * @param angle The target angle for the intake arm.
   */
  public default void setArmPosition(Angle angle) {}

  /**
   * Commands the spinner motor to run at a specific voltage.
   *
   * @param voltage The target voltage for the spinner rollers.
   */
  public default void setSpinnerVoltage(Voltage voltage) {}
}
