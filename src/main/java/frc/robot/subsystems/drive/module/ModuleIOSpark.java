// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Modified by FRC Team 4533 (Phoenix) 2026
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive.module;

import static edu.wpi.first.units.Units.*;
import static frc.lib.SparkUtil.*;
import static frc.robot.subsystems.drive.DriveConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.SparkOdometryThread;
import java.util.Queue;
import java.util.function.DoubleSupplier;

/**
 * Real IO implementation for a swerve drive module using Spark Max motor controllers and a CANcoder
 * for absolute positioning.
 *
 * <p>This implementation configures CAN devices, registers high-frequency signals with the {@link
 * SparkOdometryThread}, and handles drift compensation between the absolute encoder and the
 * internal motor encoder.
 */
public class ModuleIOSpark implements ModuleIO {
  private final Rotation2d zeroRotation;
  private Rotation2d currentTurnPosition = new Rotation2d();

  private final SparkMax driveSpark;
  private final SparkMax turnSpark;
  private final RelativeEncoder driveEncoder;
  private final RelativeEncoder turnInternalEncoder;
  private final CANcoder turnEncoder;
  private final StatusSignal<Angle> turnAbsolutePositionSignal;
  private final StatusSignal<AngularVelocity> turnVelocitySignal;

  private final SparkClosedLoopController driveController;
  private final SparkClosedLoopController turnController;

  // Queues for high-frequency data from the asynchronous thread
  private final Queue<Double> timestampQueue;
  private final Queue<Double> drivePositionQueue;
  private final Queue<Double> turnPositionQueue;

  // Debouncers for connectivity monitoring
  private final Debouncer driveConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer turnConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer turnEncoderConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  // Pre-allocated buffers for draining odometry queues without GC pressure (sized to queue
  // capacity)
  private static final int ODOMETRY_BUFFER_SIZE = 20;
  private final double[] odometryTimestampBuffer = new double[ODOMETRY_BUFFER_SIZE];
  private final double[] odometryDrivePositionBuffer = new double[ODOMETRY_BUFFER_SIZE];
  private final double[] odometryTurnPositionBuffer = new double[ODOMETRY_BUFFER_SIZE];

  /**
   * Creates a new ModuleIOSpark.
   *
   * @param module The module index (0 for front-left, 1 for front-right, 2 for back-left, 3 for
   *     back-right).
   */
  public ModuleIOSpark(int module) {
    var config = MODULE_CONFIGS[module];

    zeroRotation = config.zeroOffset();
    driveSpark = new SparkMax(config.driveCanId(), MotorType.kBrushless);
    turnSpark = new SparkMax(config.turnCanId(), MotorType.kBrushless);
    turnEncoder = new CANcoder(config.encoderCanId());

    driveEncoder = driveSpark.getEncoder();
    turnInternalEncoder = turnSpark.getEncoder();
    driveController = driveSpark.getClosedLoopController();
    turnController = turnSpark.getClosedLoopController();
    turnAbsolutePositionSignal = turnEncoder.getPosition();
    turnVelocitySignal = turnEncoder.getVelocity();

    // Register signals with the asynchronous odometry thread immediately
    timestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();
    drivePositionQueue =
        SparkOdometryThread.getInstance().registerSignal(driveSpark, driveEncoder::getPosition);
    turnPositionQueue =
        SparkOdometryThread.getInstance()
            .registerSignal(
                () ->
                    turnAbsolutePositionSignal.refresh().getValueAsDouble()
                        * TURN_ENCODER_POSITION_FACTOR);

    var driveConfig = new SparkMaxConfig();
    driveConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit((int) DRIVE_MOTOR_CURRENT_LIMIT.in(Amps))
        .secondaryCurrentLimit((int) DRIVE_MOTOR_SECONDARY_CURRENT_LIMIT.in(Amps))
        .voltageCompensation(12.0);
    driveConfig
        .encoder
        .positionConversionFactor(DRIVE_ENCODER_POSITION_FACTOR)
        .velocityConversionFactor(DRIVE_ENCODER_VELOCITY_FACTOR)
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2);
    driveConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(DRIVE_KP, 0.0, DRIVE_KD);
    driveConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs((int) (1000.0 / ODOMETRY_FREQUENCY.in(Hertz)))
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(50)
        .busVoltagePeriodMs(50)
        .outputCurrentPeriodMs(50);
    tryUntilOk(
        5,
        () ->
            driveSpark.configure(
                driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    tryUntilOk(5, () -> driveEncoder.setPosition(0.0));

    // Configure CANcoder
    var turnEncoderConfig = new CANcoderConfiguration();
    turnEncoderConfig.MagnetSensor.MagnetOffset = 0.0;
    turnEncoderConfig.MagnetSensor.SensorDirection =
        TURN_ENCODER_INVERTED
            ? SensorDirectionValue.Clockwise_Positive
            : SensorDirectionValue.CounterClockwise_Positive;
    turnEncoder.getConfigurator().apply(turnEncoderConfig);
    turnAbsolutePositionSignal.setUpdateFrequency(ODOMETRY_FREQUENCY);
    turnVelocitySignal.setUpdateFrequency(ODOMETRY_LOW_FREQUENCY);

    // Configure Turn Spark Max
    var turnConfig = new SparkMaxConfig();
    turnConfig
        .inverted(TURN_INVERTED)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit((int) TURN_MOTOR_CURRENT_LIMIT.in(Amps))
        .secondaryCurrentLimit((int) TURN_MOTOR_SECONDARY_CURRENT_LIMIT.in(Amps))
        .voltageCompensation(12.0);
    turnConfig
        .encoder
        .positionConversionFactor((2.0 * Math.PI) / TURN_MOTOR_REDUCTION)
        .velocityConversionFactor(((2.0 * Math.PI) / 60.0) / TURN_MOTOR_REDUCTION);
    turnConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(TURN_KP, 0.0, TURN_KD)
        .positionWrappingEnabled(true)
        .positionWrappingInputRange(TURN_PID_MIN_INPUT, TURN_PID_MAX_INPUT);
    turnConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs(20)
        .primaryEncoderVelocityAlwaysOn(false)
        .appliedOutputPeriodMs(50)
        .busVoltagePeriodMs(50)
        .outputCurrentPeriodMs(50);
    tryUntilOk(
        5,
        () ->
            turnSpark.configure(
                turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    // Sync internal turn encoder with CANcoder absolute position on startup
    tryUntilOk(
        5,
        () -> {
          turnAbsolutePositionSignal.refresh();
          if (turnAbsolutePositionSignal.getStatus().isOK()) {
            Rotation2d currentRot =
                new Rotation2d(
                        turnAbsolutePositionSignal.getValueAsDouble() * TURN_ENCODER_POSITION_FACTOR)
                    .minus(zeroRotation);
            return turnInternalEncoder.setPosition(currentRot.getRadians());
          } else {
            return turnInternalEncoder.setPosition(0.0);
          }
        });
  }

  /** Updates hardware inputs, monitors connectivity, and manages turn encoder drift. */
  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    // Drive Motor Inputs
    boolean driveOk = true;
    driveOk &=
        ifOk(
            driveSpark,
            driveEncoder::getPosition,
            (value) -> inputs.drivePosition = Radians.of(value));
    driveOk &=
        ifOk(
            driveSpark,
            driveEncoder::getVelocity,
            (value) -> inputs.driveVelocity = RadiansPerSecond.of(value));
    driveOk &=
        ifOk(
            driveSpark,
            new DoubleSupplier[] {driveSpark::getAppliedOutput, driveSpark::getBusVoltage},
            (values) -> inputs.driveAppliedVoltage = Volts.of(values[0] * values[1]));
    driveOk &=
        ifOk(
            driveSpark,
            driveSpark::getOutputCurrent,
            (value) -> inputs.driveCurrent = Amps.of(value));

    inputs.driveConnected = driveConnectedDebounce.calculate(driveOk);

    // Turn Encoder Inputs
    BaseStatusSignal.refreshAll(turnAbsolutePositionSignal, turnVelocitySignal);
    boolean turnEncoderOk = turnAbsolutePositionSignal.getStatus().isOK();

    if (turnEncoderOk) {
      // Calculate position relative to the zero offset
      currentTurnPosition =
          new Rotation2d(turnAbsolutePositionSignal.getValueAsDouble() * TURN_ENCODER_POSITION_FACTOR)
              .minus(zeroRotation);
      inputs.turnVelocity =
          RadiansPerSecond.of(turnVelocitySignal.getValueAsDouble() * TURN_ENCODER_VELOCITY_FACTOR);

      // Sync internal encoder to CANcoder if still and error is high
      double internalPos = turnInternalEncoder.getPosition();
      double absolutePos = currentTurnPosition.getRadians();
      double turnError = Math.abs(MathUtil.angleModulus(internalPos - absolutePos));
      boolean isStill =
          inputs.turnVelocity.abs(RadiansPerSecond) < VELOCITY_GATE.in(RadiansPerSecond);

      if (isStill && turnError > ERROR_THRESHOLD.in(Radians)) {
        turnInternalEncoder.setPosition(absolutePos);
      }
    } else {
      // Fallback to internal encoder if CANcoder disconnects
      currentTurnPosition = new Rotation2d(turnInternalEncoder.getPosition());
      inputs.turnVelocity = RadiansPerSecond.of(turnInternalEncoder.getVelocity());
    }
    inputs.turnPosition = Radians.of(currentTurnPosition.getRadians());

    // Turn Motor Inputs
    boolean turnSparkOk = true;
    turnSparkOk &=
        ifOk(
            turnSpark,
            new DoubleSupplier[] {turnSpark::getAppliedOutput, turnSpark::getBusVoltage},
            (values) -> inputs.turnAppliedVoltage = Volts.of(values[0] * values[1]));
    turnSparkOk &=
        ifOk(
            turnSpark, turnSpark::getOutputCurrent, (value) -> inputs.turnCurrent = Amps.of(value));

    inputs.turnConnected = turnConnectedDebounce.calculate(turnSparkOk);
    inputs.turnEncoderConnected = turnEncoderConnectedDebounce.calculate(turnEncoderOk);

    // Empty queues into the inputs object for odometry processing
    Drive.odometryLock.lock();
    try {
      // Drain queues into pre-allocated buffers to avoid ArrayList/boxing allocations
      int count = 0;
      Double val;
      while (count < ODOMETRY_BUFFER_SIZE && (val = timestampQueue.poll()) != null) {
        odometryTimestampBuffer[count++] = val;
      }

      // Drain drive and turn queues to match the timestamp count
      for (int i = 0; i < count; i++) {
        val = drivePositionQueue.poll();
        odometryDrivePositionBuffer[i] = (val != null) ? val : 0.0;
      }
      for (int i = 0; i < count; i++) {
        val = turnPositionQueue.poll();
        odometryTurnPositionBuffer[i] = (val != null) ? val : 0.0;
      }

      // Copy from buffers into inputs arrays (arrays must be exact-sized for downstream code)
      inputs.odometryTimestamps = new double[count];
      inputs.odometryDrivePositionsRad = new double[count];
      inputs.odometryTurnPositions = new Rotation2d[count];

      for (int i = 0; i < count; i++) {
        inputs.odometryTimestamps[i] = odometryTimestampBuffer[i];
        inputs.odometryDrivePositionsRad[i] = odometryDrivePositionBuffer[i];
        inputs.odometryTurnPositions[i] =
            new Rotation2d(odometryTurnPositionBuffer[i]).minus(zeroRotation);
      }

    } finally {
      Drive.odometryLock.unlock();
    }
  }

  @Override
  public void setDriveOpenLoop(Voltage output) {
    driveSpark.setVoltage(output);
  }

  @Override
  public void setTurnOpenLoop(Voltage output) {
    turnSpark.setVoltage(output);
  }

  @Override
  public void setDriveVelocity(AngularVelocity velocity) {
    // Calculate Feedforward voltage based on velocity
    double ffVolts =
        DRIVE_KS * Math.signum(velocity.in(RadiansPerSecond))
            + DRIVE_KV * velocity.in(RadiansPerSecond);
    // Use closed-loop velocity control with feedforward
    driveController.setSetpoint(
        velocity.in(RadiansPerSecond),
        ControlType.kVelocity,
        ClosedLoopSlot.kSlot0,
        ffVolts,
        ArbFFUnits.kVoltage);
  }

  @Override
  public void setTurnPosition(Angle rotation) {
    // Use closed-loop position control
    turnController.setSetpoint(rotation.in(Radians), ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }
}
