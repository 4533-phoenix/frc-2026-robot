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
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import java.util.function.DoubleSupplier;

/**
 * Real IO implementation for the intake using REV SparkMax controllers.
 *
 * <p>This implementation configures the arm motor for position closed-loop control using motion
 * profiling and the spinner motor for velocity control. It optimizes CAN bus traffic by setting
 * specific update frequencies for status signals.
 */
public class IntakeIOReal implements IntakeIO {
  private final SparkMax armSpark = new SparkMax(armMotorCanId, MotorType.kBrushless);
  private final SparkMax spinnerSpark = new SparkMax(spinnerMotorCanId, MotorType.kBrushless);

  private final AbsoluteEncoder armEncoder;
  private final SparkClosedLoopController armController;

  private final RelativeEncoder spinnerEncoder;
  private final SparkClosedLoopController spinnerController;

  // Debouncers to prevent rapid flickering of connection status
  private final Debouncer armConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer spinnerConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  /**
   * Creates a new IntakeIOReal and configures the SparkMax controllers.
   */
  public IntakeIOReal() {
    armEncoder = armSpark.getAbsoluteEncoder();
    armController = armSpark.getClosedLoopController();

    spinnerEncoder = spinnerSpark.getEncoder();
    spinnerController = spinnerSpark.getClosedLoopController();

    // ---------- Configure Arm Motor ----------
    var armConfig = new SparkMaxConfig();
    armConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit((int) armMotorCurrentLimit.in(Amps))
        .voltageCompensation(12.0)
        .inverted(false);
    
    // Set conversion factors for internal motor encoder
    armConfig
        .encoder
        .positionConversionFactor(armInternalEncoderPositionFactor)
        .velocityConversionFactor(armInternalEncoderVelocityFactor);
    
    // Configure absolute encoder feedback for arm position
    armConfig
        .absoluteEncoder
        .positionConversionFactor(2.0 * Math.PI)
        .zeroOffset(globalEncoderOffset.in(Rotations))
        .inverted(true);
    
    // Set PID and Feedforward gains for arm control
    armConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(armKp, 0.0, armKd);
    armConfig
        .closedLoop
        .feedForward
        .kV(armKv)
        .kA(armKa)
        .kS(armKs)
        .kCos(armKg)
        .kCosRatio(1.0 / (2.0 * Math.PI));
    
    // Configure motion profiling for smooth movement
    armConfig
        .closedLoop
        .maxMotion
        .allowedProfileError(armPositionPIDTolerance.in(Radians))
        .cruiseVelocity(armCruiseVelocity.in(RadiansPerSecond))
        .maxAcceleration(armMaxAcceleration.in(RadiansPerSecondPerSecond));
    
    // Optimize CAN traffic: set periodic frame rates for required signals
    armConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs(20)
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(50)
        .busVoltagePeriodMs(50)
        .outputCurrentPeriodMs(50);
    
    // Define soft limits for arm motion safety
    armConfig
        .softLimit
        .forwardSoftLimitEnabled(true)
        .forwardSoftLimit(armRetractedPosition.plus(armPositionSoftLimitTolerance).in(Radians))
        .reverseSoftLimitEnabled(true)
        .reverseSoftLimit(armDeployedPosition.minus(armPositionSoftLimitTolerance).in(Radians));
    
    // Apply configuration to spark with retry logic
    tryUntilOk(
        5,
        () ->
            armSpark.configure(
                armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    
    // Initialize internal encoder position from absolute encoder
    tryUntilOk(
        5,
        () -> {
          double initialPos = armEncoder.getPosition();
          return armSpark.getEncoder().setPosition(initialPos);
        });

    // ---------- Configure Spinner Motor ----------
    var spinnerConfig = new SparkMaxConfig();
    spinnerConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit((int) spinnerMotorCurrentLimit.in(Amps))
        .voltageCompensation(12.0)
        .inverted(true);
    
    // Set conversion factors for spinner encoder
    spinnerConfig
        .encoder
        .positionConversionFactor(spinnerInternalEncoderPositionFactor)
        .velocityConversionFactor(spinnerInternalEncoderVelocityFactor);
    
    // Set PID and Feedforward gains for spinner velocity control
    spinnerConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(spinnerKp, 0.0, spinnerKd);
    spinnerConfig.closedLoop.feedForward.kV(spinnerKv).kA(spinnerKa).kS(spinnerKs);
    
    // Optimize CAN traffic for spinner signals
    spinnerConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs(20)
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(50)
        .busVoltagePeriodMs(50)
        .outputCurrentPeriodMs(50);
    
    // Apply configuration with retry logic
    tryUntilOk(
        5,
        () ->
            spinnerSpark.configure(
                spinnerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    
    // Initialize spinner encoder position
    tryUntilOk(
        5,
        () -> {
          double initialPos = spinnerEncoder.getPosition();
          return spinnerSpark.getEncoder().setPosition(initialPos);
        });
  }

  /**
   * Updates inputs by refreshing data from the SparkMax controllers.
   *
   * @param inputs The inputs object to update.
   */
  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    // ---------- Arm Motor Inputs ----------
    boolean armSparkOk = true;
    armSparkOk &=
        ifOk(armSpark, armEncoder::getPosition, (value) -> inputs.armPosition = Radians.of(value));
    armSparkOk &=
        ifOk(
            armSpark,
            armEncoder::getVelocity,
            (value) -> inputs.armVelocity = RadiansPerSecond.of(value));
    armSparkOk &=
        ifOk(
            armSpark,
            new DoubleSupplier[] {armSpark::getAppliedOutput, armSpark::getBusVoltage},
            (values) -> inputs.armAppliedVoltage = Volts.of(values[0] * values[1]));
    armSparkOk &=
        ifOk(
            armSpark,
            armSpark::getOutputCurrent,
            (value) -> inputs.armAppliedCurrent = Amps.of(value));
    
    // Debounce the connection status
    inputs.armConnected = armConnectedDebounce.calculate(armSparkOk);

    // ---------- Spinner Motor Inputs ----------
    boolean spinnerSparkOk = true;
    spinnerSparkOk &=
        ifOk(
            spinnerSpark,
            spinnerEncoder::getVelocity,
            (value) -> inputs.spinnerVelocity = RadiansPerSecond.of(value));
    spinnerSparkOk &=
        ifOk(
            spinnerSpark,
            new DoubleSupplier[] {spinnerSpark::getAppliedOutput, spinnerSpark::getBusVoltage},
            (values) -> inputs.spinnerAppliedVoltage = Volts.of(values[0] * values[1]));
    spinnerSparkOk &=
        ifOk(
            spinnerSpark,
            spinnerSpark::getOutputCurrent,
            (value) -> inputs.spinnerAppliedCurrent = Amps.of(value));
    
    // Debounce the connection status
    inputs.spinnerConnected = spinnerConnectedDebounce.calculate(spinnerSparkOk);
  }

  /**
   * Commands the arm motor to move to a specified position using Motion Profiling.
   *
   * @param angle The target angle for the intake arm.
   */
  @Override
  public void setArmPosition(Angle angle) {
    armController.setSetpoint(
        angle.in(Radians), ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);
  }

  /**
   * Commands the spinner motor to run at a specific angular velocity using closed-loop control.
   *
   * @param velocity The target velocity for the spinner rollers.
   */
  @Override
  public void setSpinnerAngularVelocity(AngularVelocity velocity) {
    spinnerController.setSetpoint(
        velocity.in(RadiansPerSecond), ControlType.kVelocity, ClosedLoopSlot.kSlot0);
  }
}
