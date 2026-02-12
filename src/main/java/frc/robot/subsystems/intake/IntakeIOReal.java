// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.*;
import static frc.robot.util.SparkUtil.*;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.function.DoubleSupplier;

public class IntakeIOReal implements IntakeIO {
  private final SparkMax armSpark = new SparkMax(armMotorCanId, MotorType.kBrushless);
  private final SparkMax spinnerSpark = new SparkMax(spinnerMotorCanId, MotorType.kBrushless);

  private final AbsoluteEncoder armEncoder;
  private final SparkClosedLoopController armController;

  private final Debouncer armConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer spinnerConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  public IntakeIOReal() {
    armEncoder = armSpark.getAbsoluteEncoder();
    armController = armSpark.getClosedLoopController();

    // Configure arm motor
    var armConfig = new SparkMaxConfig();
    armConfig
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(armMotorCurrentLimit)
        .voltageCompensation(12.0)
        .inverted(true);
    armConfig
        .absoluteEncoder
        .positionConversionFactor(2.0 * Math.PI)
        .velocityConversionFactor((2.0 * Math.PI) / 60.0)
        .zeroOffset(globalEncoderOffsetRad / (2.0 * Math.PI));
    armConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .pid(armKp, 0.0, armKd)
        .positionWrappingEnabled(true)
        .positionWrappingInputRange(0, 2.0 * Math.PI);
    armConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs(20)
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(50)
        .busVoltagePeriodMs(50)
        .outputCurrentPeriodMs(50);
    tryUntilOk(
        armSpark,
        5,
        () ->
            armSpark.configure(
                armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    // Configure spinner motor
    var spinnerConfig = new SparkMaxConfig();
    spinnerConfig
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(spinnerMotorCurrentLimit)
        .voltageCompensation(12.0)
        .inverted(true);
    spinnerConfig
        .signals
        .appliedOutputPeriodMs(50)
        .busVoltagePeriodMs(50)
        .outputCurrentPeriodMs(50);
    tryUntilOk(
        spinnerSpark,
        5,
        () ->
            spinnerSpark.configure(
                spinnerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    // Arm motor
    sparkStickyFault = false;
    ifOk(armSpark, armEncoder::getPosition, (value) -> inputs.armPosition = new Rotation2d(value));
    ifOk(armSpark, armEncoder::getVelocity, (value) -> inputs.armVelocityRadPerSec = value);
    ifOk(
        armSpark,
        new DoubleSupplier[] {armSpark::getAppliedOutput, armSpark::getBusVoltage},
        (values) -> inputs.armAppliedVolts = values[0] * values[1]);
    ifOk(armSpark, armSpark::getOutputCurrent, (value) -> inputs.armCurrentAmps = value);
    inputs.armConnected = armConnectedDebounce.calculate(!sparkStickyFault);

    // Spinner motor
    sparkStickyFault = false;
    ifOk(
        spinnerSpark,
        new DoubleSupplier[] {spinnerSpark::getAppliedOutput, spinnerSpark::getBusVoltage},
        (values) -> inputs.spinnerAppliedVolts = values[0] * values[1]);
    ifOk(
        spinnerSpark, spinnerSpark::getOutputCurrent, (value) -> inputs.spinnerCurrentAmps = value);
    inputs.spinnerConnected = spinnerConnectedDebounce.calculate(!sparkStickyFault);
  }

  @Override
  public void setArmPosition(Rotation2d position, double arbFeedforwardVolts) {
    armController.setSetpoint(
        position.getRadians(), ControlType.kPosition, ClosedLoopSlot.kSlot0, arbFeedforwardVolts);
  }

  @Override
  public void setSpinnerVoltage(double volts) {
    spinnerSpark.setVoltage(volts);
  }
}
