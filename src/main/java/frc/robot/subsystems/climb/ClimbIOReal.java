// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.climb.ClimbConstants.*;
import static frc.robot.util.SparkUtil.*;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Voltage;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

// TODO: Need to verify how spark max limits work

public class ClimbIOReal implements ClimbIO {
  private final SparkMax spark = new SparkMax(liftMotorCanId, MotorType.kBrushed);

  private final SparkLimitSwitch upperLimit = spark.getForwardLimitSwitch();
  private final SparkLimitSwitch lowerLimit = spark.getReverseLimitSwitch();

  private final Debouncer liftConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  private boolean sparkStickyFault = false;

  public ClimbIOReal() {
    var liftCfg = new SparkMaxConfig();
    liftCfg
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit((int) liftMotorCurrentLimit.in(Amps))
        .voltageCompensation(12.0);
    liftCfg.signals.appliedOutputPeriodMs(20).busVoltagePeriodMs(20).outputCurrentPeriodMs(20);
    tryUntilOk(
        spark,
        5,
        () ->
            spark.configure(
                liftCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(ClimbIOInputs inputs) {
    sparkStickyFault = false;
    ifOk(
        spark,
        new DoubleSupplier[] {spark::getAppliedOutput, spark::getBusVoltage},
        (vals) -> inputs.appliedVoltage = Volts.of(vals[0] * vals[1]));
    ifOk(spark, spark::getOutputCurrent, (v) -> inputs.appliedCurrent = Amps.of(v));
    ifOk(
        spark,
        new BooleanSupplier[] {upperLimit::isPressed, lowerLimit::isPressed},
        (vals) -> {
          inputs.upperLimit = vals[0];
          inputs.lowerLimit = vals[1];
        });
    inputs.connected = liftConnectedDebounce.calculate(!sparkStickyFault);
  }

  @Override
  public void setLiftVoltage(Voltage voltage) {
    boolean atUpper = !upperLimit.isPressed();
    boolean atLower = !lowerLimit.isPressed();
    if ((voltage.gt(Volts.of(0.0)) && atUpper) || (voltage.lt(Volts.of(0.0)) && atLower)) {
      spark.setVoltage(0.0);
    } else {
      spark.setVoltage(voltage);
    }
  }
}
