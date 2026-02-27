// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.indexer.IndexerConstants.*;
import static frc.robot.util.SparkUtil.*;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Voltage;
import java.util.function.DoubleSupplier;

public class IndexerIOSpark implements IndexerIO {
  private final SparkMax spark = new SparkMax(indexerMotorId, MotorType.kBrushless);

  private final Debouncer sparkDebouncer = new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  public IndexerIOSpark() {
    var config = new SparkMaxConfig();
    config
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit((int) indexerMotorCurrentLimit.in(Amps))
        .voltageCompensation(12.0);
    config.signals.appliedOutputPeriodMs(40).busVoltagePeriodMs(40).outputCurrentPeriodMs(40);

    tryUntilOk(
        5,
        () ->
            spark.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    boolean sparkOk = true;
    sparkOk &=
        ifOk(
            spark,
            new DoubleSupplier[] {spark::getAppliedOutput, spark::getBusVoltage},
            (values) -> inputs.appliedVoltage = Volts.of(values[0] * values[1]));
    sparkOk &=
        ifOk(spark, spark::getOutputCurrent, (value) -> inputs.appliedCurrent = Amps.of(value));
    inputs.connected = sparkDebouncer.calculate(sparkOk);
  }

  @Override
  public void setVoltage(Voltage voltage) {
    spark.setVoltage(voltage.in(Volts));
  }
}
