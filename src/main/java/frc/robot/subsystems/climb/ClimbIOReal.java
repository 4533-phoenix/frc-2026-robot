// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.climb;

import static frc.robot.subsystems.climb.ClimbConstants.*;
import static frc.robot.util.SparkUtil.*;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;

// TODO: Need to verify how spark max limits work

public class ClimbIOReal implements ClimbIO {
  private final SparkMax liftSpark = new SparkMax(liftMotorCanId, MotorType.kBrushed);
  private final SparkMax rotateSpark = new SparkMax(rotateMotorCanId, MotorType.kBrushed);

  private final SparkLimitSwitch liftUpperLimit = liftSpark.getForwardLimitSwitch();
  private final SparkLimitSwitch liftLowerLimit = liftSpark.getReverseLimitSwitch();

  private final SparkLimitSwitch rotateMaxLimit = liftSpark.getForwardLimitSwitch();
  private final SparkLimitSwitch rotateMinLimit = liftSpark.getReverseLimitSwitch();

  private final Debouncer liftConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer rotateConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  private boolean sparkStickyFault = false;

  public ClimbIOReal() {
    var liftCfg = new SparkMaxConfig();
    liftCfg
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(liftMotorCurrentLimit)
        .voltageCompensation(12.0);
    liftCfg.signals.appliedOutputPeriodMs(20).busVoltagePeriodMs(20).outputCurrentPeriodMs(20);
    tryUntilOk(
        liftSpark,
        5,
        () ->
            liftSpark.configure(
                liftCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    var rotateCfg = new SparkMaxConfig();
    rotateCfg
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(rotateMotorCurrentLimit)
        .voltageCompensation(12.0);
    rotateCfg.signals.appliedOutputPeriodMs(20).busVoltagePeriodMs(20).outputCurrentPeriodMs(20);
    tryUntilOk(
        rotateSpark,
        5,
        () ->
            rotateSpark.configure(
                rotateCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(ClimbIOInputs inputs) {
    sparkStickyFault = false;
    ifOk(
        liftSpark,
        new DoubleSupplier[] {
          liftSpark::getAppliedOutput, liftSpark::getBusVoltage
        },
        (vals) -> inputs.liftAppliedVolts = vals[0] * vals[1]);
    ifOk(liftSpark, liftSpark::getOutputCurrent, (v) -> inputs.liftCurrentAmps = v);
    ifOk(liftSpark, new BooleanSupplier[] {
          liftUpperLimit::isPressed, liftLowerLimit::isPressed
        }, (vals) -> {
          inputs.liftUpperLimit = vals[0];
          inputs.liftLowerLimit = vals[1];
        });
    inputs.liftConnected = liftConnectedDebounce.calculate(!sparkStickyFault);

    sparkStickyFault = false;
    ifOk(
        rotateSpark,
        new DoubleSupplier[] {
          rotateSpark::getAppliedOutput, rotateSpark::getBusVoltage
        },
        (vals) -> inputs.rotateAppliedVolts = vals[0] * vals[1]);
    ifOk(rotateSpark, rotateSpark::getOutputCurrent, (v) -> inputs.rotateCurrentAmps = v);
    ifOk(rotateSpark, new BooleanSupplier[] {
          rotateMaxLimit::isPressed, rotateMinLimit::isPressed
        }, (vals) -> {
          inputs.rotateMinLimit = vals[0];
          inputs.rotateMaxLimit = vals[1];
        });
    inputs.rotateConnected = rotateConnectedDebounce.calculate(!sparkStickyFault);
  }

  @Override
  public void setLiftOpenLoop(double volts) {
    boolean atUpper = !liftUpperLimit.isPressed();
    boolean atLower = !liftLowerLimit.isPressed();
    if ((volts > 0.0 && atUpper) || (volts < 0.0 && atLower)) {
      liftSpark.setVoltage(0.0);
    } else {
      liftSpark.setVoltage(volts);
    }
  }

  @Override
  public void setRotateOpenLoop(double volts) {
    boolean atMax = !rotateMaxLimit.isPressed();
    boolean atMin = !rotateMinLimit.isPressed();
    if ((volts > 0.0 && atMax) || (volts < 0.0 && atMin)) {
      rotateSpark.setVoltage(0.0);
    } else {
      rotateSpark.setVoltage(volts);
    }
  }
}
