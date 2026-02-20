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

/** IO interface for the intake hardware. */
public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    // Arm motor
    public boolean armConnected = false;
    public Angle armPosition = Radians.of(0.0);
    public AngularVelocity armVelocity = RadiansPerSecond.of(0.0);
    public Voltage armAppliedVoltage = Volts.of(0.0);
    public Current armAppliedCurrent = Amps.of(0.0);

    // Spinner motor
    public boolean spinnerConnected = false;
    public Voltage spinnerAppliedVoltage = Volts.of(0.0);
    public Current spinnerAppliedCurrent = Amps.of(0.0);
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(IntakeIOInputs inputs) {}

  /** Run the arm motor to the specified position. */
  public default void setArmPosition(Angle angle) {}

  /** Run the spinner motor at the specified open loop voltage. */
  public default void setSpinnerVoltage(Voltage voltage) {}
}
