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
  /** The gearbox model for the indexer motor (e.g., REV NEO 550). */
  public static final DCMotor indexerGearbox = DCMotor.getNeo550(1);
  /** The gear reduction between the indexer motor and the conveyor mechanism. */
  public static final double indexerReduction = 1.0; // TODO: Verify this ratio
  /** The moment of inertia of the indexer mechanism, used for simulation. */
  public static final MomentOfInertia indexerMOI = KilogramSquareMeters.of(0.0005);
  /** Maximum current limit for the indexer motor to prevent overheating. */
  public static final Current indexerMotorCurrentLimit = Amps.of(20.0);
  /** The voltage applied to run the indexer at full speed. */
  public static final Voltage indexerOnVoltage = Volts.of(12.0);
  /** CAN ID for the indexer motor controller. */
  public static final int indexerMotorId = 17;
}
