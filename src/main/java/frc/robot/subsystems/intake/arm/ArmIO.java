// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.intake.arm;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

/** IO interface for the arm hardware abstraction. */
public interface ArmIO {
  /** Contains all of the inputs received from the arm hardware. */
  @AutoLog
  public static class ArmIOInputs {
    // ---------- Arm Motor Inputs ----------
    /** Whether the arm motor controller is connected. */
    public boolean connected = false;

    /** The current absolute position of the intake arm. */
    public Angle position = Radians.zero();

    /** The current angular velocity of the intake arm. */
    public AngularVelocity velocity = RadiansPerSecond.zero();

    /** The voltage currently being applied to the arm motor. */
    public Voltage appliedVoltage = Volts.zero();

    /** The current drawn by the arm motor. */
    public Current appliedCurrent = Amps.zero();

    /** Whether the arm motor is functioning correctly. */
    public boolean healthy = true;

    /** The full status of the arm motor controller. */
    public int[] status = new int[] {0, 0, 0, 0};
  }

  /**
   * Updates the set of loggable inputs with the latest data from the intake hardware.
   *
   * @param inputs The inputs object to update.
   */
  public default void updateInputs(ArmIOInputs inputs) {}

  /**
   * Commands the arm motor to move to a specified position using closed-loop control.
   *
   * @param angle The target angle for the arm.
   */
  public default void setPosition(Angle angle) {}

  /** Tells the motor to stop trying to reach its setpoint */
  public default void stop() {}

  /** Clears all faults and warnings. */
  public default void clearFaults() {}
}
