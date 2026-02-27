// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.indexer.IndexerConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/**
 * Physics simulation implementation of {@link IndexerIO}.
 *
 * <p>This class uses {@link DCMotorSim} to model the indexer motor based on physical constants
 * defined in {@link IndexerConstants}. It updates the simulation state on every {@link
 * #updateInputs(IndexerIOInputs)} call.
 */
public class IndexerIOSim implements IndexerIO {
  // Simulates the physical mechanics of the indexer
  private final DCMotorSim sim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              indexerGearbox, indexerMOI.in(KilogramSquareMeters), indexerReduction),
          indexerGearbox);

  private Voltage appliedVoltage = Volts.of(0.0);

  /**
   * Updates the simulation state and updates loggable inputs.
   *
   * @param inputs The inputs object to update with simulated data.
   */
  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    // Advance simulation by 20ms (standard robot loop time)
    sim.update(0.020);

    // Update loggable inputs with simulated data
    inputs.connected = true;
    inputs.appliedVoltage = appliedVoltage;
    inputs.appliedCurrent = Amps.of(sim.getCurrentDrawAmps());
  }

  /**
   * Sets the voltage to be applied to the simulated motor.
   *
   * @param voltage The voltage to apply, clamped between -12V and 12V.
   */
  @Override
  public void setVoltage(Voltage voltage) {
    // Clamp voltage to battery limits
    this.appliedVoltage = Volts.of(MathUtil.clamp(voltage.in(Volts), -12.0, 12.0));
    sim.setInputVoltage(appliedVoltage.in(Volts));
  }
}
