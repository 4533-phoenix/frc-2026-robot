// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

/** IO interface for the intake hardware. */
public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    // Arm motor
    public boolean armConnected = false;
    public Rotation2d armPosition = Rotation2d.kZero;
    public double armVelocityRadPerSec = 0.0;
    public double armAppliedVolts = 0.0;
    public double armCurrentAmps = 0.0;

    // Spinner motor
    public boolean spinnerConnected = false;
    public double spinnerAppliedVolts = 0.0;
    public double spinnerCurrentAmps = 0.0;

    // Duty cycle encoder (absolute)
    public boolean dutyCycleConnected = false;
    public Rotation2d dutyCyclePosition = Rotation2d.kZero;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(IntakeIOInputs inputs) {}

  /** Run the arm motor to the specified position with arbitrary feedforward. */
  public default void setArmPosition(Rotation2d position, double arbFeedforwardVolts) {}

  /** Run the spinner motor at the specified open loop voltage. */
  public default void setSpinnerVoltage(double volts) {}
}
