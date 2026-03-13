// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.shooter.flywheel;

import static edu.wpi.first.units.Units.*;
import static frc.lib.SparkUtil.*;
import static frc.robot.subsystems.shooter.ShooterConstants.*;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.AngularVelocity;
import java.util.function.DoubleSupplier;

/**
 * Real IO implementation for the shooter flywheel using a SparkFlex motor controller (Neo Vortex).
 *
 * <p>This implementation configures the motor for MAXMotion velocity control, which generates a
 * smooth trapezoidal acceleration profile to reach the target velocity. Feedforward gains (kS, kV,
 * kA) are configured directly in the controller, eliminating the need for manual feedforward
 * calculations.
 */
public class FlywheelIOSpark implements FlywheelIO {
  private final SparkFlex spark = new SparkFlex(CAN_ID, MotorType.kBrushless);
  private final RelativeEncoder encoder = spark.getEncoder();
  private final SparkClosedLoopController controller = spark.getClosedLoopController();

  // Debouncer to prevent rapidly toggling connection status
  private final Debouncer connectedDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  // Cache the last sent velocity to avoid redundant CAN writes
  private AngularVelocity sentVelocity = null;

  /** Creates a new FlywheelIOSpark and configures the motor controller. */
  public FlywheelIOSpark() {
    var config = new SparkFlexConfig();
    config
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit((int) MOTOR_CURRENT_LIMIT.in(Amps))
        .voltageCompensation(12.0);
    config
        .encoder
        .positionConversionFactor(FLYWHEEL_ENCODER_POSITION_FACTOR)
        .velocityConversionFactor(FLYWHEEL_ENCODER_VELOCITY_FACTOR)
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2);
    config
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(FLYWHEEL_KP, FLYWHEEL_KI, FLYWHEEL_KD);
    config.closedLoop.feedForward.kS(FLYWHEEL_KS).kV(FLYWHEEL_KV).kA(FLYWHEEL_KA);
    config
        .closedLoop
        .maxMotion
        .maxAcceleration(FLYWHEEL_MAX_ACCELERATION.in(RadiansPerSecondPerSecond))
        .allowedProfileError(ANGULAR_TOLERANCE.in(RadiansPerSecond));
    config
        .signals
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20)
        .isAtSetpointPeriodMs(20);
    tryUntilOk(
        5,
        () ->
            spark.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  /**
   * Updates inputs by reading telemetry from the SparkFlex.
   *
   * @param inputs The inputs object to update.
   */
  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    boolean sparkOk = true;

    sparkOk &=
        ifOk(spark, encoder::getVelocity, (value) -> inputs.velocity = RadiansPerSecond.of(value));
    sparkOk &=
        ifOk(
            spark,
            new DoubleSupplier[] {spark::getAppliedOutput, spark::getBusVoltage},
            (values) -> inputs.appliedVoltage = Volts.of(values[0] * values[1]));
    sparkOk &=
        ifOk(spark, spark::getOutputCurrent, (value) -> inputs.appliedCurrent = Amps.of(value));
    sparkOk &= ifOk(spark, controller::isAtSetpoint, (value) -> inputs.atSetpoint = value);

    inputs.connected = connectedDebounce.calculate(sparkOk);

    // Health
    inputs.status[0] = spark.getFaults().rawBits;
    inputs.healthy = inputs.status[0] == 0;
    inputs.status[1] = spark.getStickyFaults().rawBits;
    inputs.status[2] = spark.getWarnings().rawBits;
    inputs.status[3] = spark.getStickyWarnings().rawBits;
  }

  /**
   * Commands the SparkFlex to spin at a specific angular velocity using MAXMotion velocity control.
   *
   * <p>The controller internally applies the configured feedforward (kS, kV, kA) and generates a
   * smooth acceleration profile to reach the target velocity.
   *
   * @param velocity The target angular velocity.
   */
  @Override
  public void setAngularVelocity(AngularVelocity velocity) {
    if (sentVelocity != null && velocity.isEquivalent(sentVelocity)) return;
    controller.setSetpoint(
        velocity.in(RadiansPerSecond),
        ControlType.kMAXMotionVelocityControl,
        ClosedLoopSlot.kSlot0);
    sentVelocity = velocity;
  }

  /** Stops the flywheel motor by setting voltage output to zero. */
  @Override
  public void stop() {
    spark.setVoltage(0.0);
    sentVelocity = null;
  }
}
