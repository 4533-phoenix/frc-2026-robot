// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.ExtraMotors;

/**
 * Hardware and tuning constants for the climb subsystem.
 *
 * <p>Contains CAN IDs, current limits, and gearbox models for the climb mechanism.
 */
public final class ClimbConstants {
  // CAN IDs
  /** CAN ID for the lift mechanism motor. */
  public static final int CAN_ID = 19;

  // Motor and current limits
  /** Maximum current limit for the lift motor to prevent thermal damage. */
  public static final Current MOTOR_CURRENT_LIMIT = Amps.of(30);
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
}
