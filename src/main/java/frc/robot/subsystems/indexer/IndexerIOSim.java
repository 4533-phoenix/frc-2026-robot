// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.indexer.IndexerConstants.*;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;

/**
 * Physics simulation implementation of {@link IndexerIO}.
 *
 * <p>This class uses {@link SparkMaxSim} to simulate the Spark MAX motor controller for the
 * indexer, backed by a {@link DCMotorSim} physics model. Configuration mirrors {@link
 * IndexerIOSpark}.
 */
public class IndexerIOSim implements IndexerIO {
  private final SparkMax spark;
  private final SparkMaxSim sparkSim;
  private final DCMotorSim physicsSim;

  /** Creates a new IndexerIOSim and initializes the simulated Spark MAX. */
  public IndexerIOSim() {
    spark = new SparkMax(CAN_ID, MotorType.kBrushless);
    sparkSim = new SparkMaxSim(spark, GEARBOX);

    physicsSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(GEARBOX, MOI.in(KilogramSquareMeters), REDUCTION),
            GEARBOX);

    // Configure Spark MAX (mirrors IndexerIOSpark)
    var config = new SparkMaxConfig();
    config
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit((int) MOTOR_CURRENT_LIMIT.in(Amps))
        .voltageCompensation(12.0);
    spark.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  /**
   * Updates the simulation state and updates loggable inputs.
   *
   * @param inputs The inputs object to update with simulated data.
   */
  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    // Update physics model with voltage from Spark sim
    physicsSim.setInputVoltage(sparkSim.getAppliedOutput() * RoboRioSim.getVInVoltage());

    // Advance simulation by 20ms
    physicsSim.update(0.02);

    // Update SparkMaxSim with physics results
    // No conversion factor set, so iterate() expects RPM (default)
    sparkSim.iterate(physicsSim.getAngularVelocityRPM(), RoboRioSim.getVInVoltage(), 0.02);

    // Update loggable inputs
    inputs.connected = true;
    inputs.appliedVoltage = Volts.of(spark.getAppliedOutput() * spark.getBusVoltage());
    inputs.appliedCurrent = Amps.of(spark.getOutputCurrent());
  }

  /**
   * Sets the voltage to be applied to the simulated motor.
   *
   * @param voltage The voltage to apply.
   */
  @Override
  public void setVoltage(Voltage voltage) {
    spark.setVoltage(voltage.in(Volts));
  }
}
