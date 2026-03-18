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
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.drive.SparkOdometryThread;
import frc.robot.subsystems.drive.SparkOdometryThread.PrimitiveQueue;
import frc.robot.util.SparkTap;
import frc.robot.util.SparkTap.MotorView;

/**
 * Real IO implementation for a swerve drive module using Spark Max motor controllers and a CANcoder
 * for absolute positioning.
 */
public class ModuleIOSpark implements ModuleIO {
  private final Angle zeroRotation;

  private final SparkMax driveSpark;
  private final SparkMax turnSpark;
  private final RelativeEncoder driveEncoder;
  private final RelativeEncoder turnInternalEncoder;
  private final CANcoder turnEncoder;
  private final StatusSignal<Angle> turnAbsolutePositionSignal;
  private final StatusSignal<AngularVelocity> turnVelocitySignal;

  private final SparkClosedLoopController driveController;
  private final SparkClosedLoopController turnController;

  private final MotorView driveTap;
  private final MotorView turnTap;

  // Primitive Zero-GC Queues for high-frequency data
  private final PrimitiveQueue timestampQueue;
  private final PrimitiveQueue drivePositionQueue = new PrimitiveQueue();
  private final PrimitiveQueue turnPositionQueue = new PrimitiveQueue();
  private final PrimitiveQueue driveVelocityQueue = new PrimitiveQueue();
  private final PrimitiveQueue turnVelocityQueue = new PrimitiveQueue();

  private final Debouncer driveConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer turnConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer turnEncoderConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  /**
   * Creates a new ModuleIOSpark for the specified module index and configures the Spark Max motor
   * controllers and CANcoder.
   *
   * @param module The index of the module (0-3) to determine which configuration to use from the
   *     configs.
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

    driveTap = SparkTap.getInstance().getMotor(config.driveCanId());
    turnTap = SparkTap.getInstance().getMotor(config.turnCanId());

    SparkOdometryThread odometry = SparkOdometryThread.getInstance();
    timestampQueue = odometry.makeTimestampQueue();
    odometry.registerSignal(
        () -> {
          // Snapshot mathematically time-aligned positions
          drivePositionQueue.offer(driveTap.getLatencyCompensatedPosition());
          turnPositionQueue.offer(turnTap.getLatencyCompensatedPosition());
          driveVelocityQueue.offer(driveTap.getVelocity());
          turnVelocityQueue.offer(turnTap.getVelocity());
        });

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
    driveConfig.closedLoop.feedForward.kS(DRIVE_KS).kV(DRIVE_KV);
    driveConfig.closedLoop.maxMotion.maxAcceleration(200.0).allowedProfileError(0.0);
    driveConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs((int) (1000.0 / ODOMETRY_FREQUENCY.in(Hertz)))
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs((int) (1000.0 / ODOMETRY_FREQUENCY.in(Hertz)))
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
    turnVelocitySignal.setUpdateFrequency(ODOMETRY_FREQUENCY);

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
        .primaryEncoderPositionPeriodMs((int) (1000.0 / ODOMETRY_FREQUENCY.in(Hertz)))
        .primaryEncoderVelocityAlwaysOn(false)
        .primaryEncoderVelocityPeriodMs((int) (1000.0 / ODOMETRY_FREQUENCY.in(Hertz)))
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
            return turnInternalEncoder.setPosition(
                (turnAbsolutePositionSignal.getValueAsDouble() * TURN_ENCODER_POSITION_FACTOR)
                    - zeroRotation.in(Radians));
          } else {
            return turnInternalEncoder.setPosition(0.0);
          }
        });
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    // Drive Motor Inputs
    boolean driveOk = driveTap.isConnected();
    inputs.driveAppliedVoltage = Volts.of(driveTap.getAppliedOutput() * driveTap.getBusVoltage());
    inputs.driveCurrent = Amps.of(driveTap.getOutputCurrent());

    // Turn Motor Inputs
    boolean turnSparkOk = turnTap.isConnected();
    inputs.turnAppliedVoltage = Volts.of(turnTap.getAppliedOutput() * turnTap.getBusVoltage());
    inputs.turnCurrent = Amps.of(turnTap.getOutputCurrent());

    // Transfer primitive data to AdvantageKit inputs
    int count = timestampQueue.size;
    if (inputs.odometryTimestamps == null || inputs.odometryTimestamps.length != count) {
      inputs.odometryTimestamps = new double[count];
      inputs.odometryDrivePositionsRad = new double[count];
      inputs.odometryTurnPositionsRad = new double[count];
    }

    System.arraycopy(timestampQueue.data, 0, inputs.odometryTimestamps, 0, count);
    System.arraycopy(drivePositionQueue.data, 0, inputs.odometryDrivePositionsRad, 0, count);
    System.arraycopy(turnPositionQueue.data, 0, inputs.odometryTurnPositionsRad, 0, count);

    if (count > 0) {
      inputs.drivePosition = Radians.of(drivePositionQueue.data[count - 1]);
      inputs.turnPosition = Radians.of(turnPositionQueue.data[count - 1]);
      inputs.driveVelocity =
          RadiansPerSecond.of(driveVelocityQueue.data[count - 1] * DRIVE_ENCODER_VELOCITY_FACTOR);
      inputs.turnVelocity =
          RadiansPerSecond.of(turnVelocityQueue.data[count - 1] * TURN_ENCODER_VELOCITY_FACTOR);
    }

    timestampQueue.clear();
    drivePositionQueue.clear();
    turnPositionQueue.clear();
    driveVelocityQueue.clear();
    turnVelocityQueue.clear();

    inputs.driveConnected = driveConnectedDebounce.calculate(driveOk);

    // Turn Encoder Inputs
    BaseStatusSignal.refreshAll(turnAbsolutePositionSignal, turnVelocitySignal);
    boolean turnEncoderOk = turnAbsolutePositionSignal.getStatus().isOK();

    if (turnEncoderOk) {
      // Sync internal encoder to CANcoder if still and error is high
      double internalPos = inputs.turnPosition.in(Radians);
      double absolutePos =
          (turnAbsolutePositionSignal.getValueAsDouble() * TURN_ENCODER_POSITION_FACTOR)
              - zeroRotation.in(Radians);
      double turnError = Math.abs(MathUtil.angleModulus(internalPos - absolutePos));
      boolean isStill =
          inputs.turnVelocity.abs(RadiansPerSecond) < VELOCITY_GATE.in(RadiansPerSecond);

      if (isStill && turnError > ERROR_THRESHOLD.in(Radians)) {
        turnInternalEncoder.setPosition(absolutePos);
      }
    }

    inputs.turnConnected = turnConnectedDebounce.calculate(turnSparkOk);
    inputs.turnEncoderConnected = turnEncoderConnectedDebounce.calculate(turnEncoderOk);

    // Drive Health
    inputs.driveStatus[0] = driveTap.getActiveFaults();
    inputs.driveStatus[1] = driveTap.getStickyFaults();
    inputs.driveStatus[2] = driveTap.getActiveWarnings();
    inputs.driveStatus[3] = driveTap.getStickyWarnings();
    inputs.driveHealthy = inputs.driveStatus[0] == 0;

    // Turn Health
    inputs.turnStatus[0] = turnTap.getActiveFaults();
    inputs.turnStatus[1] = turnTap.getStickyFaults();
    inputs.turnStatus[2] = turnTap.getActiveWarnings();
    inputs.turnStatus[3] = turnTap.getStickyWarnings();
    inputs.turnHealthy = inputs.turnStatus[0] == 0;
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
    driveController.setSetpoint(
        velocity.in(RadiansPerSecond),
        ControlType.kMAXMotionVelocityControl,
        ClosedLoopSlot.kSlot0);
  }

  @Override
  public void setTurnPosition(Angle rotation) {
    turnController.setSetpoint(rotation.in(Radians), ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  @Override
  public void clearFaults() {
    driveSpark.clearFaults();
    turnSpark.clearFaults();
  }
}
