// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.intake.spinner;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

/** Hardware and tuning constants for the intake subsystem. */
public final class SpinnerConstants {
  // ---------- CAN IDs ----------
  /** CAN ID for the roller spinner motor. */
  public static final int CAN_ID = 16;

  // ---------- Gear Ratios ----------
  /** Gear ratio: 2.13 motor rotations = 1 output shaft rotation for the spinner. */
  public static final double REDUCTION = 6.39;

  // ---------- Encoder Conversion Factors ----------
  /** Converts internal motor rotations to radians for the spinner. */
  public static final double INTERNAL_ENCODER_POSITION_FACTOR = (2.0 * Math.PI) / REDUCTION;
  /** Converts internal motor velocity (RPM) to radians per second for the spinner. */
  public static final double INTERNAL_ENCODER_VELOCITY_FACTOR =
      ((2.0 * Math.PI) / 60.0) / REDUCTION;

  // ---------- Motor Current Limits ----------
  /** Max current draw for the spinner motor. */
  public static final Current MOTOR_CURRENT_LIMIT = Amps.of(30);

  // ---------- Spinner Voltages ----------
  /** Voltage for pulling game pieces in. */
  public static final Voltage INTAKE_VOLTAGE = Volts.of(12.0);
  /** Voltage for pushing game pieces out. */
  public static final Voltage EXTAKE_VOLTAGE = Volts.of(-12.0);

  /** The motor type and quantity used by the spinner gearbox. */
  public static final DCMotor GEARBOX = DCMotor.getNEO(1);
}
