// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.*;
import static frc.robot.util.SparkUtil.*;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
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
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import java.util.function.DoubleSupplier;

public class IntakeIOReal implements IntakeIO {
  private final SparkMax armSpark = new SparkMax(armMotorCanId, MotorType.kBrushless);
  private final SparkMax spinnerSpark = new SparkMax(spinnerMotorCanId, MotorType.kBrushless);

  private final RelativeEncoder armEncoder;
  private final SparkClosedLoopController armController;

  private final DutyCycleEncoder dutyCycleEncoder = new DutyCycleEncoder(dutyCycleEncoderPin);

  private final Debouncer armConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer spinnerConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  public IntakeIOReal() {
    armEncoder = armSpark.getEncoder();
    armController = armSpark.getClosedLoopController();

    // Configure arm motor
    var armConfig = new SparkMaxConfig();
    armConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(armMotorCurrentLimit)
        .voltageCompensation(12.0)
        .inverted(true);
    armConfig
        .encoder
        .positionConversionFactor(armEncoderPositionFactor)
        .velocityConversionFactor(armEncoderVelocityFactor)
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2);
    armConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(armKp, 0.0, armKd);
    armConfig
        .softLimit
        .forwardSoftLimitEnabled(true)
        .forwardSoftLimit(armForwardSoftLimitRad)
        .reverseSoftLimitEnabled(true)
        .reverseSoftLimit(armReverseSoftLimitRad);
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

    // Seed the SparkMax internal encoder with the duty cycle encoder reading
    seedArmEncoder();

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

  // Seeds the sparks internal encoder with the external encoder. TODO: WE NEED THE GLOBAL ENCODER ADAPTER.
  private void seedArmEncoder() {
    tryUntilOk(
        armSpark,
        5,
        () -> {
          if (dutyCycleEncoder.isConnected()) {
            double absolutePositionRad =
                (dutyCycleEncoder.get() * 2.0 * Math.PI) - globalEncoderOffsetRad;
            return armEncoder.setPosition(absolutePositionRad);
          } else {
            return armEncoder.setPosition(0.0);
          }
        });
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

    // Duty cycle encoder
    inputs.dutyCycleConnected = dutyCycleEncoder.isConnected();
    if (inputs.dutyCycleConnected) {
      double rawRad = dutyCycleEncoder.get() * 2.0 * Math.PI;
      inputs.dutyCyclePosition = new Rotation2d(rawRad - globalEncoderOffsetRad);
    }
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
