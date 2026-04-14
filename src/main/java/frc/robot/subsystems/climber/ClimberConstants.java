// Copyright (c) 2026 FRC Team 4533 (Phoenix)
//
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Force;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.hardware.ExtraMotors;

/**
 * Hardware and tuning constants for the climb subsystem.
 *
 * <p>Contains CAN IDs, current limits, and gearbox models for the climb mechanism.
 */
public final class ClimberConstants {
  // CAN IDs
  /** CAN ID for the lift mechanism motor. */
  public static final int CAN_ID = 19;

  // Motor and current limits
  /** Maximum current limit for the lift motor to prevent thermal damage. */
  public static final Current MOTOR_CURRENT_LIMIT = Amps.of(30);

  /** Inversion setting for the lift motor. */
  public static final boolean MOTOR_INVERTED = true;

  /** The gearbox model for the lift motor. */
  public static final DCMotor GEARBOX = ExtraMotors.getSnowBlower(1);

  /** Default voltage used for lifting and lowering the mechanism. */
  public static final Voltage DEFAULT_VOLTAGE = Volts.of(12.0);

  /** Simulated upper travel limit. */
  public static final Distance UPPER_HEIGHT = Centimeters.of(22.0);

  /** Simulated lower travel limit. */
  public static final Distance LOWER_HEIGHT = Centimeters.of(0.0);

  /** The radius of the winch drum when empty. */
  public static final Distance DRUM_RADIUS = Centimeters.of(1.0);

  /** Gear reduction on the climber */
  public static final double REDUCTION = 2.0;

  /** The mass of the climber mechanism in kilograms. */
  public static final Mass CLIMBER_MASS = Kilograms.of(0.46 + 0.227 + 0.151 + 0.15);

  /** The spring constant for the climber mechanism in Newtons per meter. */
  public static final Force SPRING_CONSTANT = Newtons.of(52.8449);
}
