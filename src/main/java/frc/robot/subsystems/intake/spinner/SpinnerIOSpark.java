// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.intake.spinner;

import static edu.wpi.first.units.Units.*;
import static frc.lib.SparkUtil.*;
import static frc.robot.subsystems.intake.spinner.SpinnerConstants.*;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.SparkTap;
import frc.robot.util.SparkTap.MotorView;

/**
 * Real IO implementation for the intake using REV SparkMax controllers.
 *
 * <p>This implementation configures the arm motor for position closed-loop control using motion
 * profiling and the spinner motor for velocity control. It optimizes CAN bus traffic by setting
 * specific update frequencies for status signals.
 */
public class SpinnerIOSpark implements SpinnerIO {
  private final SparkMax spark = new SparkMax(CAN_ID, MotorType.kBrushless);
  private final MotorView motorView = SparkTap.getInstance().getMotor(CAN_ID);

  // Debouncers to prevent rapid flickering of connection status
  private final Debouncer connectedDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  /** Creates a new SpinnerIOSpark and configures the SparkMax controllers. */
  public SpinnerIOSpark() {
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
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(SpinnerIOInputs inputs) {
    inputs.connected = connectedDebounce.calculate(motorView.isConnected());

    // Safely retrieve telemetry from the motor controller
    inputs.appliedVoltage = Volts.of(motorView.getAppliedOutput() * motorView.getBusVoltage());
    inputs.appliedCurrent = Amps.of(motorView.getOutputCurrent());

    // Health
    inputs.status[0] = motorView.getActiveFaults();
    inputs.status[1] = motorView.getStickyFaults();
    inputs.status[2] = motorView.getActiveWarnings();
    inputs.status[3] = motorView.getStickyWarnings();
    inputs.healthy = inputs.status[0] == 0;
  }

  @Override
  public void setVoltage(Voltage voltage) {
    spark.setVoltage(voltage.in(Volts));
  }

  @Override
  public void clearFaults() {
    spark.clearFaults();
  }
}
