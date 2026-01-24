// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.spinner;

import static frc.robot.subsystems.spinner.SpinnerConstants.*;
import static frc.robot.util.SparkUtil.*;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;
import java.util.function.DoubleSupplier;

public class SpinnerIOSpark implements SpinnerIO {
  private final SparkMax sparkMax;

  private final Debouncer connectedDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  public SpinnerIOSpark() {
    sparkMax = new SparkMax(canId, MotorType.kBrushless);

    var config = new SparkMaxConfig();
    config.idleMode(IdleMode.kBrake).smartCurrentLimit(motorCurrentLimit).voltageCompensation(12.0);
    config.signals.appliedOutputPeriodMs(20).busVoltagePeriodMs(20).outputCurrentPeriodMs(20);
    tryUntilOk(
        sparkMax,
        5,
        () ->
            sparkMax.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(SpinnerIOInputs inputs) {
    sparkStickyFault = false;
    ifOk(
        sparkMax,
        new DoubleSupplier[] {sparkMax::getAppliedOutput, sparkMax::getBusVoltage},
        (values) -> inputs.appliedVolts = values[0] * values[1]);
    inputs.connected = connectedDebounce.calculate(!sparkStickyFault);
  }

  @Override
  public void setVoltage(double appliedVolts) {
    sparkMax.setVoltage(appliedVolts);
  }
}
