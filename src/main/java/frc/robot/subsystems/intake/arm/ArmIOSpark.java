// Copyright (c) 2026 FRC Team 4533 (Phoenix)
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.intake.arm;

import static edu.wpi.first.units.Units.*;
import static frc.lib.util.SparkUtil.*;
import static frc.robot.subsystems.intake.arm.ArmConstants.*;

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
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import frc.lib.lowlevel.SparkTap;
import frc.lib.lowlevel.SparkTap.MotorView;

/**
 * Real IO implementation for the intake using REV SparkMax controllers.
 *
 * <p>This implementation configures the arm motor for position closed-loop control using motion
 * profiling and the internal relative encoder, while continuously syncing to the absolute encoder
 * to prevent drift and handle startup seeding.
 */
public class ArmIOSpark implements ArmIO {
  private final SparkMax spark = new SparkMax(CAN_ID, MotorType.kBrushless);
  private final MotorView motorView = SparkTap.getInstance().getMotor(CAN_ID);
  private final AbsoluteEncoder absoluteEncoder;
  private final RelativeEncoder internalEncoder;
  private final SparkClosedLoopController controller;

  // Debouncers to prevent rapid flickering of connection status
  private final Debouncer connectedDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  private Angle sentPosition = null;

  /** Creates a new ArmIOSpark and configures the SparkMax controllers. */
  public ArmIOSpark() {
    absoluteEncoder = spark.getAbsoluteEncoder();
    internalEncoder = spark.getEncoder();
    controller = spark.getClosedLoopController();

    var armConfig = createBaseConfig(MOTOR_CURRENT_LIMIT, MOTOR_INVERTED);
    armConfig
        .encoder
        .positionConversionFactor(INTERNAL_ENCODER_POSITION_FACTOR)
        .velocityConversionFactor(INTERNAL_ENCODER_VELOCITY_FACTOR);
    armConfig
        .absoluteEncoder
        .positionConversionFactor(GLOBAL_ENCODER_POSITION_FACTOR)
        .velocityConversionFactor(GLOBAL_ENCODER_VELOCITY_FACTOR)
        .zeroOffset(GLOBAL_ENCODER_OFFSET)
        .inverted(true);
    armConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(KP, 0.0, KD);
    armConfig.closedLoop.feedForward.kV(KV).kA(KA).kS(KS).kCos(KG).kCosRatio(1.0 / (2.0 * Math.PI));
    armConfig
        .closedLoop
        .maxMotion
        .allowedProfileError(PID_TOLERANCE.in(Radians))
        .cruiseVelocity(CRUISE_VELOCITY.in(RadiansPerSecond))
        .maxAcceleration(MAX_ACCELERATION.in(RadiansPerSecondPerSecond));
    armConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs(50)
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(50)
        .absoluteEncoderPositionAlwaysOn(true)
        .absoluteEncoderPositionPeriodMs(50);
    armConfig
        .softLimit
        .forwardSoftLimitEnabled(true)
        .forwardSoftLimit(RETRACTED_POSITION.plus(SOFT_LIMIT_TOLERANCE).in(Radians))
        .reverseSoftLimitEnabled(true)
        .reverseSoftLimit(DEPLOYED_POSITION.minus(SOFT_LIMIT_TOLERANCE).in(Radians));

    tryUntilOk(
        5,
        () ->
            spark.configure(
                armConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters));
    tryUntilOk(5, () -> spark.getEncoder().setPosition(absoluteEncoder.getPosition()));
  }

  /**
   * Updates inputs by refreshing data from the SparkMax controllers.
   *
   * @param inputs The inputs object to update.
   */
  @Override
  public void updateInputs(ArmIOInputs inputs) {
    inputs.connected = connectedDebounce.calculate(motorView.isConnected());

    double absPos = motorView.getAbsoluteEncoderPosition();
    double internalPos = motorView.getPosition();
    double currentVel = motorView.getVelocity();

    if (inputs.connected) {
      boolean absEncoderReady = (absPos != 0.0);
      boolean isStill = Math.abs(currentVel) < VELOCITY_GATE.in(RadiansPerSecond);
      double armError = Math.abs(internalPos - absPos);
      if (absEncoderReady && isStill && armError > ERROR_THRESHOLD.in(Radians)) {
        internalEncoder.setPosition(absPos);
        internalPos = absPos;
      }
    }

    // Log the internal encoder data
    inputs.position = Radians.of(internalPos);
    inputs.velocity = RadiansPerSecond.of(currentVel);

    // Power Telemetry
    inputs.appliedVoltage = Volts.of(motorView.getAppliedOutput() * motorView.getBusVoltage());
    inputs.appliedCurrent = Amps.of(motorView.getOutputCurrent());

    // Health
    inputs.status[0] = motorView.getActiveFaults();
    inputs.status[1] = motorView.getStickyFaults();
    inputs.status[2] = motorView.getActiveWarnings();
    inputs.status[3] = motorView.getStickyWarnings();
    inputs.healthy = inputs.status[0] == 0;
  }

  @Override
  public void setPosition(Angle angle) {
    if (sentPosition != null && angle.isEquivalent(sentPosition)) return;
    controller.setSetpoint(
        angle.in(Radians), ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);
    sentPosition = angle;
  }

  @Override
  public void stop() {
    spark.stopMotor();
    sentPosition = null;
  }

  @Override
  public void clearFaults() {
    spark.clearFaults();
  }
}
