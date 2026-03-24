// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.*;
import static frc.lib.util.SparkUtil.*;
import static frc.robot.subsystems.indexer.IndexerConstants.*;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.lowlevel.SparkTap;
import frc.lib.lowlevel.SparkTap.MotorView;

/**
 * Real IO implementation for the indexer subsystem using a Spark Max motor controller.
 *
 * <p>This implementation configures the Spark Max with specific current limits and idle modes, and
 * monitors connectivity via a debouncer.
 */
public class IndexerIOSpark implements IndexerIO {
  private final SparkMax spark = new SparkMax(CAN_ID, MotorType.kBrushless);
  private final MotorView motorView = SparkTap.getInstance().getMotor(CAN_ID);

  // Debouncer to prevent rapidly toggling connection status
  private final Debouncer connectedDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  /** Creates a new IndexerIOSpark and configures the Spark Max. */
  public IndexerIOSpark() {
    var config = new SparkMaxConfig();
    config
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit((int) MOTOR_CURRENT_LIMIT.in(Amps))
        .voltageCompensation(12.0);
    config
        .signals
        .appliedOutputPeriodMs(50)
        .busVoltagePeriodMs(50)
        .outputCurrentPeriodMs(50)
        .faultsAlwaysOn(true)
        .warningsAlwaysOn(true);
    tryUntilOk(
        5,
        () ->
            spark.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters));
  }

  /** Updates hardware inputs and monitors connectivity. */
  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    inputs.connected = connectedDebounce.calculate(motorView.isConnected());

    // Power Telemetry
    inputs.appliedVoltage = Volts.of(motorView.getAppliedOutput() * motorView.getBusVoltage());
    inputs.appliedCurrent = Amps.of(motorView.getOutputCurrent());

    // Health
    inputs.status[0] = motorView.getActiveFaults();
    inputs.status[1] = motorView.getStickyFaults();
    inputs.status[2] = motorView.getActiveWarnings();
    inputs.status[3] = motorView.getStickyWarnings();
    inputs.healthy = inputs.status[0] == 0;
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

  @Override
  public void clearFaults() {
    spark.clearFaults();
  }
}
