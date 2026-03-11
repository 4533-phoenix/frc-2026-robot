// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.*;

/**
 * Hardware and tuning constants for the indexer subsystem.
 *
 * <p>Contains gearbox models, current limits, and voltage settings for the indexer motor.
 */
public final class IndexerConstants {
  /** CAN ID for the indexer motor controller. */
  public static final int CAN_ID = 17;
  /** The gearbox model for the indexer motor. */
  public static final DCMotor GEARBOX = DCMotor.getNeo550(1);
  /** The gear reduction between the indexer motor and the conveyor mechanism. */
  public static final double REDUCTION = 17.0;
  /** The moment of inertia of the indexer mechanism, used for simulation. */
  public static final MomentOfInertia MOI = KilogramSquareMeters.of(0.0005);
  /** Maximum current limit for the indexer motor to prevent overheating. */
  public static final Current MOTOR_CURRENT_LIMIT = Amps.of(20.0);
  /** The voltage applied to run the indexer at full speed. */
  public static final Voltage DEFAULT_VOLTAGE = Volts.of(12.0);
}
