// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.intake.arm;

import static edu.wpi.first.units.Units.*;
import static frc.lib.SparkUtil.*;
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
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import java.util.function.DoubleSupplier;

/**
 * Real IO implementation for the intake using REV SparkMax controllers.
 *
 * <p>This implementation configures the arm motor for position closed-loop control using motion
 * profiling and the internal relative encoder, while continuously syncing to the absolute encoder
 * to prevent drift and handle startup seeding.
 */
public class ArmIOSpark implements ArmIO {
  private final SparkMax spark = new SparkMax(CAN_ID, MotorType.kBrushless);
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

    var armConfig = new SparkMaxConfig();
    armConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit((int) MOTOR_CURRENT_LIMIT.in(Amps))
        .voltageCompensation(12.0)
        .inverted(false);
    armConfig
        .encoder
        .positionConversionFactor(INTERNAL_ENCODER_POSITION_FACTOR)
        .velocityConversionFactor(INTERNAL_ENCODER_VELOCITY_FACTOR);
    armConfig
        .absoluteEncoder
        .positionConversionFactor(2.0 * Math.PI)
        .zeroOffset(GLOBAL_ENCODER_OFFSET.in(Rotations))
        .inverted(true);

    // PID runs off the primary internal encoder for maximum smoothness and high D-gains
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
        .appliedOutputPeriodMs(50)
        .busVoltagePeriodMs(50)
        .outputCurrentPeriodMs(50);
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
                armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    tryUntilOk(
        5,
        () -> {
          double initialPos = absoluteEncoder.getPosition();
          return spark.getEncoder().setPosition(initialPos);
        });
    tryUntilOk(5, () -> spark.getEncoder().setPosition(absoluteEncoder.getPosition()));
  }

  /**
   * Updates inputs by refreshing data from the SparkMax controllers.
   *
   * @param inputs The inputs object to update.
   */
  @Override
  public void updateInputs(ArmIOInputs inputs) {
    boolean armSparkOk = true;

    // Temporary containers to extract values from ifOk checks
    final double[] absPosContainer = new double[1];
    final double[] intPosContainer = new double[1];
    final double[] velContainer = new double[1];

    boolean absOk = ifOk(spark, absoluteEncoder::getPosition, (val) -> absPosContainer[0] = val);
    boolean intPosOk = ifOk(spark, internalEncoder::getPosition, (val) -> intPosContainer[0] = val);
    boolean velOk = ifOk(spark, internalEncoder::getVelocity, (val) -> velContainer[0] = val);

    armSparkOk &= (absOk && intPosOk && velOk);

    // If CAN communications for position/velocity were successful, process drift compensation
    if (armSparkOk) {
      double absPos = absPosContainer[0];
      double internalPos = intPosContainer[0];
      double currentVel = velContainer[0];
      boolean absEncoderReady = (absPos != 0.0);

      boolean isStill = Math.abs(currentVel) < VELOCITY_GATE.in(RadiansPerSecond);
      double armError = Math.abs(internalPos - absPos);

      // Apply Synchronization
      if (absEncoderReady && isStill && armError > ERROR_THRESHOLD.in(Radians)) {
        internalEncoder.setPosition(absPos);
        internalPos = absPos;
      }

      // Log the internal encoder data
      inputs.position = Radians.of(internalPos);
      inputs.velocity = RadiansPerSecond.of(currentVel);

      // Health
      inputs.status[0] = spark.getFaults().rawBits;
      inputs.healthy = inputs.status[0] == 0;
      inputs.status[1] = spark.getStickyFaults().rawBits;
      inputs.status[2] = spark.getWarnings().rawBits;
      inputs.status[3] = spark.getStickyWarnings().rawBits;
    }

    // Power and current inputs
    armSparkOk &=
        ifOk(
            spark,
            new DoubleSupplier[] {spark::getAppliedOutput, spark::getBusVoltage},
            (values) -> inputs.appliedVoltage = Volts.of(values[0] * values[1]));
    armSparkOk &=
        ifOk(spark, spark::getOutputCurrent, (value) -> inputs.appliedCurrent = Amps.of(value));

    // Debounce the connection status
    inputs.connected = connectedDebounce.calculate(armSparkOk);
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
