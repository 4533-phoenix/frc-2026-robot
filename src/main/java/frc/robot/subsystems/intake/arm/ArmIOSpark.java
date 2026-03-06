// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.intake.arm;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.intake.arm.ArmConstants.*;
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
import frc.robot.util.HardwareConfigManager;
import java.util.function.DoubleSupplier;

/**
 * Real IO implementation for the intake using REV SparkMax controllers.
 *
 * <p>This implementation configures the arm motor for position closed-loop control using motion
 * profiling and the spinner motor for velocity control. It optimizes CAN bus traffic by setting
 * specific update frequencies for status signals.
 */
public class ArmIOSpark implements ArmIO {
  private final SparkMax spark = new SparkMax(canId, MotorType.kBrushless);
  private final AbsoluteEncoder encoder;
  private final SparkClosedLoopController controller;

  // Debouncers to prevent rapid flickering of connection status
  private final Debouncer connectedDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  // Target the user requested but not necessarily sent to the hardware yet
  private Angle pendingTarget = null;
  // Last target that was actually sent to the IO layer
  private Angle lastSentTarget = null;

  /** Creates a new ArmIOSpark and configures the SparkMax controllers. */
  public ArmIOSpark() {
    encoder = spark.getAbsoluteEncoder();
    controller = spark.getClosedLoopController();

    // Register async config task
    HardwareConfigManager.registerTask(this::configureHardware);
  }

  // Runs on background thread
  private void configureHardware() {
    // Configure Arm Motor
    var armConfig = new SparkMaxConfig();
    armConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit((int) motorCurrentLimit.in(Amps))
        .voltageCompensation(12.0)
        .inverted(false);
    armConfig
        .encoder
        .positionConversionFactor(internalEncoderPositionFactor)
        .velocityConversionFactor(internalEncoderVelocityFactor);
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
        .allowedProfileError(positionPIDTolerance.in(Radians))
        .cruiseVelocity(cruiseVelocity.in(RadiansPerSecond))
        .maxAcceleration(maxAcceleration.in(RadiansPerSecondPerSecond));
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
        .forwardSoftLimit(retractedPosition.plus(softLimitTolerance).in(Radians))
        .reverseSoftLimitEnabled(true)
        .reverseSoftLimit(deployedPosition.minus(softLimitTolerance).in(Radians));
    tryUntilOk(
        5,
        () ->
            spark.configure(
                armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    tryUntilOk(
        5,
        () -> {
          double initialPos = encoder.getPosition();
          return spark.getEncoder().setPosition(initialPos);
        });
  }

  /**
   * Updates inputs by refreshing data from the SparkMax controllers.
   *
   * @param inputs The inputs object to update.
   */
  @Override
  public void updateInputs(ArmIOInputs inputs) {
    if (!HardwareConfigManager.isReady()) return;

    if (pendingTarget != null) {
      if (lastSentTarget == null || !pendingTarget.equals(lastSentTarget)) {
        controller.setSetpoint(
            pendingTarget.in(Radians),
            ControlType.kMAXMotionPositionControl,
            ClosedLoopSlot.kSlot0);
        lastSentTarget = pendingTarget;
      }
    }

    // Arm Motor Inputs
    boolean armSparkOk = true;
    armSparkOk &= ifOk(spark, encoder::getPosition, (value) -> inputs.position = Radians.of(value));
    armSparkOk &=
        ifOk(spark, encoder::getVelocity, (value) -> inputs.velocity = RadiansPerSecond.of(value));
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

  /**
   * Commands the arm motor to move to a specified position using Motion Profiling.
   *
   * @param angle The target angle for the intake arm.
   */
  @Override
  public void setPosition(Angle angle) {
    pendingTarget = angle;
    if (!HardwareConfigManager.isReady()) return;
    controller.setSetpoint(
        angle.in(Radians), ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);
    lastSentTarget = angle;
  }
}
