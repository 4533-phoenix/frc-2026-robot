// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.climb;

import static frc.robot.subsystems.climb.ClimbConstants.*;
import static frc.robot.util.SparkUtil.*;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;
import com.revrobotics.spark.SparkMaxLimitSwitch;
import com.revrobotics.spark.SparkMaxLimitSwitch.Type;

public class ClimbIOReal implements ClimbIO {
  private final SparkMax liftA = new SparkMax(liftMotorACanId, MotorType.kBrushless);
  private final SparkMax liftB = new SparkMax(liftMotorBCanId, MotorType.kBrushless);

  private final SparkMax rotateA = new SparkMax(rotateMotorACanId, MotorType.kBrushless);
  private final SparkMax rotateB = new SparkMax(rotateMotorBCanId, MotorType.kBrushless);

  private final SparkMaxLimitSwitch liftForwardLimit = liftA.getForwardLimitSwitch(Type.kNormallyOpen);
  private final SparkMaxLimitSwitch liftReverseLimit = liftA.getReverseLimitSwitch(Type.kNormallyOpen);
  private final SparkMaxLimitSwitch rotateForwardLimit = rotateA.getForwardLimitSwitch(Type.kNormallyOpen);
  private final SparkMaxLimitSwitch rotateReverseLimit = rotateA.getReverseLimitSwitch(Type.kNormallyOpen);

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
    // No encoders configured for open-loop operation
    liftCfg.signals.appliedOutputPeriodMs(20).busVoltagePeriodMs(20).outputCurrentPeriodMs(20);
    tryUntilOk(
        liftA,
        5,
        () ->
            liftA.configure(
                liftCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    tryUntilOk(
        liftB,
        5,
        () ->
            liftB.configure(
                liftCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    var rotateCfg = new SparkMaxConfig();
    rotateCfg
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(rotateMotorCurrentLimit)
        .voltageCompensation(12.0);
    // No encoders configured for open-loop operation
    rotateCfg.signals.appliedOutputPeriodMs(20).busVoltagePeriodMs(20).outputCurrentPeriodMs(20);
    tryUntilOk(
        rotateA,
        5,
        () ->
            rotateA.configure(
                rotateCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    tryUntilOk(
        rotateB,
        5,
        () ->
            rotateB.configure(
                rotateCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(ClimbIOInputs inputs) {
    sparkStickyFault = false;
    ifOk(
        liftA,
        new java.util.function.DoubleSupplier[] {liftA::getAppliedOutput, liftA::getBusVoltage},
        (vals) -> inputs.liftAppliedVolts = vals[0] * vals[1]);
    ifOk(liftA, liftA::getOutputCurrent, (v) -> inputs.liftCurrentAmps = v);
    inputs.liftConnected = liftConnectedDebounce.calculate(!sparkStickyFault);

    // Rotate: read applied volts and current from motor controllers
    sparkStickyFault = false;
    ifOk(
        rotateA,
        new java.util.function.DoubleSupplier[] {rotateA::getAppliedOutput, rotateA::getBusVoltage},
        (vals) -> inputs.rotateAppliedVolts = vals[0] * vals[1]);
    ifOk(rotateA, rotateA::getOutputCurrent, (v) -> inputs.rotateCurrentAmps = v);
    inputs.rotateConnected = rotateConnectedDebounce.calculate(!sparkStickyFault);

  inputs.liftLowerLimit = liftReverseLimit.isPressed();
  inputs.liftUpperLimit = liftForwardLimit.isPressed();
  inputs.rotateMinLimit = rotateReverseLimit.isPressed();
  inputs.rotateMaxLimit = rotateForwardLimit.isPressed();
  }

  @Override
  public void setLiftOpenLoop(double volts) {
    boolean atUpper = liftForwardLimit.isPressed();
    boolean atLower = liftReverseLimit.isPressed();
    if ((volts > 0.0 && atUpper) || (volts < 0.0 && atLower)) {
      liftA.setVoltage(0.0);
      liftB.setVoltage(0.0);
    } else {
      liftA.setVoltage(volts);
      liftB.setVoltage(volts);
    }
  }

  @Override
  public void setRotateOpenLoop(double volts) {
    boolean atMax = rotateForwardLimit.isPressed();
    boolean atMin = rotateReverseLimit.isPressed();
    if ((volts > 0.0 && atMax) || (volts < 0.0 && atMin)) {
      rotateA.setVoltage(0.0);
      rotateB.setVoltage(0.0);
    } else {
      rotateA.setVoltage(volts);
      rotateB.setVoltage(volts);
    }
  }
}
