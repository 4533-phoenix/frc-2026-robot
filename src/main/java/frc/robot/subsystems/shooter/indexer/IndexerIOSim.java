// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.shooter.indexer;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.shooter.ShooterConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IndexerIOSim implements IndexerIO {
  private final DCMotorSim sim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(indexerGearbox, indexerMOI, indexerReduction),
          indexerGearbox);

  private Voltage appliedVoltage = Volts.of(0.0);

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    sim.update(0.020);

    inputs.connected = true;
    inputs.appliedVoltage = appliedVoltage;
    inputs.appliedCurrent = Amps.of(sim.getCurrentDrawAmps());
  }

  @Override
  public void setVoltage(Voltage voltage) {
    this.appliedVoltage = Volts.of(MathUtil.clamp(voltage.in(Volts), -12.0, 12.0));
    sim.setInputVoltage(appliedVoltage.in(Volts));
  }
}
