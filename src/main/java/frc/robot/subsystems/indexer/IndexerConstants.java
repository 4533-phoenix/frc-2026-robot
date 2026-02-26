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

public final class IndexerConstants {
  public static final DCMotor indexerGearbox = DCMotor.getNeo550(1);
  public static final double indexerReduction = 1.0; // TODO: Is this correct?
  public static final MomentOfInertia indexerMOI = KilogramSquareMeters.of(0.0005);
  public static final Current indexerMotorCurrentLimit = Amps.of(20.0);
  public static final Voltage indexerOnVoltage = Volts.of(12.0);
  public static final int indexerMotorId = 17;
}
