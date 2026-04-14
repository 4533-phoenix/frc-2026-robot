// Copyright (c) 2026 FRC Team 4533 (Phoenix)
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.*;
import static frc.lib.util.SparkUtil.*;
import static frc.robot.subsystems.climber.ClimberConstants.*;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.lowlevel.SparkTap;
import frc.lib.lowlevel.SparkTap.MotorView;

/** Real IO implementation for the climb subsystem using a Spark Max motor controller. */
public class ClimberIOSpark implements ClimberIO {
  private final SparkMax spark = new SparkMax(CAN_ID, MotorType.kBrushed);
  private final MotorView motorView = SparkTap.getInstance().getMotor(CAN_ID);

  // Debouncer to prevent rapidly toggling connection status
  private final Debouncer connectedDebounce = new Debouncer(0.1, Debouncer.DebounceType.kFalling);

  private Voltage sentVoltage = null;

  /** Creates a new ClimbIOSpark and configures the Spark Max. */
  public ClimberIOSpark() {
    var config = createBaseConfig(MOTOR_CURRENT_LIMIT, MOTOR_INVERTED);

    tryUntilOk(
        5,
        () ->
            spark.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters));
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
    Voltage targetVoltage = voltage;
    if ((voltage.gt(Volts.zero()) && motorView.getForwardLimit())
        || (voltage.lt(Volts.zero()) && motorView.getReverseLimit())) {
      targetVoltage = Volts.zero();
    }

    if (sentVoltage != null && targetVoltage.isEquivalent(sentVoltage)) return;
    spark.setVoltage(targetVoltage.in(Volts));
    sentVoltage = targetVoltage;
  }

  @Override
  public void clearFaults() {
    spark.clearFaults();
  }
}
