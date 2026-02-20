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
import edu.wpi.first.units.measure.Voltage;
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
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit((int) armMotorCurrentLimit.in(Amps))
        .voltageCompensation(12.0)
        .inverted(true);
    armConfig
        .absoluteEncoder
        .positionConversionFactor(2.0 * Math.PI)
        .velocityConversionFactor((2.0 * Math.PI) / 60.0)
        .zeroOffset(globalEncoderOffset.in(Radians) / (2.0 * Math.PI));
    armConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .pid(armKp, 0.0, armKd)
        .positionWrappingEnabled(true)
        .positionWrappingInputRange(0, 2.0 * Math.PI)
        .maxMotion
        .allowedProfileError(armPositionTolerance.in(Radians))
        .cruiseVelocity(armMaxVelocity.in(RadiansPerSecond))
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
        .smartCurrentLimit((int) spinnerMotorCurrentLimit.in(Amps))
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
  public void setSpinnerVoltage(Voltage voltage) {
    spinnerSpark.setVoltage(voltage.in(Volts));
  }
}
