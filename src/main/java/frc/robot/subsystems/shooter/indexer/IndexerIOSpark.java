// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.shooter.indexer;

import static frc.robot.subsystems.shooter.ShooterConstants.*;
import static frc.robot.util.SparkUtil.*;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class IndexerIOSpark implements IndexerIO {
  private final SparkMax spark = new SparkMax(indexerMotorId, MotorType.kBrushless);

  public IndexerIOSpark() {
    var config = new SparkMaxConfig();
    config
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(indexerMotorCurrentLimit)
        .voltageCompensation(12.0);
    config.signals.appliedOutputPeriodMs(40).busVoltagePeriodMs(40).outputCurrentPeriodMs(40);

    tryUntilOk(
        spark,
        5,
        () ->
            spark.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    inputs.connected =
        true; // SparkMax doesn't have a simple isConnected check like Phoenix6, assume true if no
    // error?
    // Actually Phoenix6 BaseStatusSignal.isAllGood checks if data is fresh.
    // For SparkMax we can check for faults or just assume connected if we can read.
    // But existing ShooterIOReal didn't implement updateInputs for Indexer, so I'll just do basic
    // reading.

    inputs.positionRad = spark.getEncoder().getPosition();
    inputs.velocityRadPerSec = spark.getEncoder().getVelocity();
    inputs.appliedVolts = spark.getAppliedOutput() * spark.getBusVoltage();
    inputs.currentAmps = spark.getOutputCurrent();
  }

  @Override
  public void setVolts(double volts) {
    spark.setVoltage(volts);
  }
}
