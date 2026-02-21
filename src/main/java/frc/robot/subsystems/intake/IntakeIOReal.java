// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;
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
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import java.util.function.DoubleSupplier;

public class IntakeIOReal implements IntakeIO {
  private final SparkMax armSpark = new SparkMax(armMotorCanId, MotorType.kBrushless);
  private final SparkMax spinnerSpark = new SparkMax(spinnerMotorCanId, MotorType.kBrushless);

  private final AbsoluteEncoder armEncoder;
  private final SparkClosedLoopController armController;

  private final AbsoluteEncoder spinnerEncoder;
  private final SparkClosedLoopController spinnerController;

  private final Debouncer armConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer spinnerConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  public IntakeIOReal() {
    armEncoder = armSpark.getAbsoluteEncoder();
    armController = armSpark.getClosedLoopController();

    spinnerEncoder = spinnerSpark.getAbsoluteEncoder();
    spinnerController = spinnerSpark.getClosedLoopController();

    // Configure arm motor
    var armConfig = new SparkMaxConfig();
    armConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit((int) armMotorCurrentLimit.in(Amps))
        .voltageCompensation(12.0)
        .inverted(false);
    armConfig
        .encoder
        .positionConversionFactor(armInternalEncoderPositionFactor)
        .velocityConversionFactor(armInternalEncoderVelocityFactor);
    armConfig
        .absoluteEncoder
        .positionConversionFactor(2.0 * Math.PI)
        .zeroOffset(globalEncoderOffset.in(Rotations))
        .inverted(true);
    armConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(armKp, 0.0, armKd);
    armConfig
        .closedLoop
        .feedForward
        .kV(armKv)
        .kA(armKa)
        .kS(armKs)
        .kCos(armKg)
        .kCosRatio(1.0 / (2.0 * Math.PI));
    armConfig
        .closedLoop
        .maxMotion
        .allowedProfileError(armPositionPIDTolerance.in(Radians))
        .cruiseVelocity(armCruiseVelocity.in(RadiansPerSecond))
        .maxAcceleration(armMaxAcceleration.in(RadiansPerSecondPerSecond));
    armConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs(20)
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(50)
        .busVoltagePeriodMs(50)
        .outputCurrentPeriodMs(50);
    armConfig
        .softLimit
        .forwardSoftLimitEnabled(true)
        .forwardSoftLimit(armRetractedPosition.plus(armPositionSoftLimitTolerance).in(Radians))
        .reverseSoftLimitEnabled(true)
        .reverseSoftLimit(armDeployedPosition.minus(armPositionSoftLimitTolerance).in(Radians));
    tryUntilOk(
        armSpark,
        5,
        () ->
            armSpark.configure(
                armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    tryUntilOk(
        armSpark,
        5,
        () -> {
          double initialPos = armEncoder.getPosition();
          return armSpark.getEncoder().setPosition(initialPos);
        });

    // Configure spinner motor
    var spinnerConfig = new SparkMaxConfig();
    spinnerConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit((int) spinnerMotorCurrentLimit.in(Amps))
        .voltageCompensation(12.0)
        .inverted(true);
    spinnerConfig
        .encoder
        .positionConversionFactor(spinnerInternalEncoderPositionFactor)
        .velocityConversionFactor(spinnerInternalEncoderVelocityFactor);
    spinnerConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(spinnerKp, 0.0, spinnerKd);
    spinnerConfig.closedLoop.feedForward.kV(spinnerKv).kA(spinnerKa).kS(spinnerKs);
    spinnerConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs(20)
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(50)
        .busVoltagePeriodMs(50)
        .outputCurrentPeriodMs(50);
    tryUntilOk(
        spinnerSpark,
        5,
        () ->
            spinnerSpark.configure(
                spinnerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    tryUntilOk(
        spinnerSpark,
        5,
        () -> {
          double initialPos = spinnerEncoder.getPosition();
          return spinnerSpark.getEncoder().setPosition(initialPos);
        });
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    // Arm motor
    sparkStickyFault = false;
    ifOk(armSpark, armEncoder::getPosition, (value) -> inputs.armPosition = Radians.of(value));
    ifOk(
        armSpark,
        armEncoder::getVelocity,
        (value) -> inputs.armVelocity = RadiansPerSecond.of(value));
    ifOk(
        armSpark,
        new DoubleSupplier[] {armSpark::getAppliedOutput, armSpark::getBusVoltage},
        (values) -> inputs.armAppliedVoltage = Volts.of(values[0] * values[1]));
    ifOk(
        armSpark, armSpark::getOutputCurrent, (value) -> inputs.armAppliedCurrent = Amps.of(value));
    inputs.armConnected = armConnectedDebounce.calculate(!sparkStickyFault);

    // Spinner motor
    sparkStickyFault = false;
    ifOk(
        spinnerSpark,
        spinnerEncoder::getPosition,
        (value) -> inputs.spinnerPosition = Radians.of(value));
    ifOk(
        spinnerSpark,
        spinnerEncoder::getVelocity,
        (value) -> inputs.spinnerVelocity = RadiansPerSecond.of(value));
    ifOk(
        spinnerSpark,
        new DoubleSupplier[] {spinnerSpark::getAppliedOutput, spinnerSpark::getBusVoltage},
        (values) -> inputs.spinnerAppliedVoltage = Volts.of(values[0] * values[1]));
    ifOk(
        spinnerSpark,
        spinnerSpark::getOutputCurrent,
        (value) -> inputs.spinnerAppliedCurrent = Amps.of(value));
    inputs.spinnerConnected = spinnerConnectedDebounce.calculate(!sparkStickyFault);
  }

  @Override
  public void setArmPosition(Angle angle) {
    armController.setSetpoint(
        angle.in(Radians), ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);
  }

  @Override
  public void setSpinnerAngularVelocity(AngularVelocity velocity) {
    spinnerController.setSetpoint(
        velocity.in(RadiansPerSecond), ControlType.kVelocity, ClosedLoopSlot.kSlot0);
  }
}
