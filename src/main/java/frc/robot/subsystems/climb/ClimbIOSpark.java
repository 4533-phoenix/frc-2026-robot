// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.*;
import static frc.lib.util.SparkUtil.*;
import static frc.robot.subsystems.climb.ClimbConstants.*;

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

/** Real IO implementation for the climb subsystem using a Spark Max motor controller. */
public class ClimbIOSpark implements ClimbIO {
  private final SparkMax spark = new SparkMax(CAN_ID, MotorType.kBrushed);
  private final MotorView motorView = SparkTap.getInstance().getMotor(CAN_ID);

  // Debouncer to prevent rapidly toggling connection status
  private final Debouncer connectedDebounce = new Debouncer(0.1, Debouncer.DebounceType.kFalling);

  /** Creates a new ClimbIOSpark and configures the Spark Max. */
  public ClimbIOSpark() {
    var liftCfg = new SparkMaxConfig();
    liftCfg
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit((int) MOTOR_CURRENT_LIMIT.in(Amps))
        .inverted(true)
        .voltageCompensation(12.0);
    liftCfg
        .signals
        .appliedOutputPeriodMs(50)
        .busVoltagePeriodMs(50)
        .outputCurrentPeriodMs(50)
        .limitsPeriodMs(20)
        .faultsAlwaysOn(true)
        .warningsAlwaysOn(true);

    tryUntilOk(
        5,
        () ->
            spark.configure(
                liftCfg, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters));
  }

  @Override
  public void updateInputs(ClimbIOInputs inputs) {
    inputs.connected = connectedDebounce.calculate(motorView.isConnected());

    // Power Telemetry
    inputs.appliedVoltage = Volts.of(motorView.getAppliedOutput() * motorView.getBusVoltage());
    inputs.appliedCurrent = Amps.of(motorView.getOutputCurrent());

    // Limit Switches
    inputs.upperLimit = motorView.getForwardLimit();
    inputs.lowerLimit = motorView.getReverseLimit();

    // Health
    inputs.status[0] = motorView.getActiveFaults();
    inputs.status[1] = motorView.getStickyFaults();
    inputs.status[2] = motorView.getActiveWarnings();
    inputs.status[3] = motorView.getStickyWarnings();
    inputs.healthy = inputs.status[0] == 0;
  }

  @Override
  public void setLiftVoltage(Voltage voltage) {
    if ((voltage.gt(Volts.zero()) && motorView.getForwardLimit())
        || (voltage.lt(Volts.zero()) && motorView.getReverseLimit())) {
      spark.setVoltage(0.0);
    } else {
      spark.setVoltage(voltage.magnitude());
    }
  }

  @Override
  public void clearFaults() {
    spark.clearFaults();
  }
}
