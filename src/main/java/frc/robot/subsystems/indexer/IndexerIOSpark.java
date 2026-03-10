// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.*;
import static frc.lib.SparkUtil.*;
import static frc.robot.subsystems.indexer.IndexerConstants.*;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Voltage;
import java.util.function.DoubleSupplier;

/**
 * Real IO implementation for the indexer subsystem using a Spark Max motor controller.
 *
 * <p>This implementation configures the Spark Max with specific current limits and idle modes, and
 * monitors connectivity via a debouncer.
 */
public class IndexerIOSpark implements IndexerIO {
  private final SparkMax spark = new SparkMax(indexerMotorId, MotorType.kBrushless);

  // Debouncer to prevent rapidly toggling connection status
  private final Debouncer sparkDebouncer = new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  /** Creates a new IndexerIOSpark and configures the Spark Max. */
  public IndexerIOSpark() {
    var config = new SparkMaxConfig();
    config
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit((int) indexerMotorCurrentLimit.in(Amps))
        .voltageCompensation(12.0);
    config.signals.appliedOutputPeriodMs(20).busVoltagePeriodMs(20).outputCurrentPeriodMs(20);
    tryUntilOk(
        5,
        () ->
            spark.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  /** Updates hardware inputs and monitors connectivity. */
  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    boolean sparkOk = true;

    // Safely retrieve telemetry from the motor controller
    sparkOk &=
        ifOk(
            spark,
            new DoubleSupplier[] {spark::getAppliedOutput, spark::getBusVoltage},
            (values) -> inputs.appliedVoltage = Volts.of(values[0] * values[1]));
    sparkOk &=
        ifOk(spark, spark::getOutputCurrent, (value) -> inputs.appliedCurrent = Amps.of(value));

    // Debounce the connection status to ensure stability
    inputs.connected = sparkDebouncer.calculate(sparkOk);
  }

  /**
   * Sets the indexer motor voltage.
   *
   * @param voltage The requested voltage to apply.
   */
  @Override
  public void setVoltage(Voltage voltage) {
    spark.setVoltage(voltage.in(Volts));
  }
}
