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
import static frc.lib.util.SparkUtil.*;
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
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.HighFreqBuffer;
import frc.lib.lowlevel.SparkTap;
import frc.lib.lowlevel.SparkTap.MotorView;

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

  // High-frequency data tracking
  private final HighFreqBuffer moduleBuffer = new HighFreqBuffer(2);

  // Single variables instead of queues (we only need the latest velocity/position for standard
  // telemetry)
  private double latestDrivePosition = 0.0;
  private double latestTurnPosition = 0.0;
  private double latestDriveVelocity = 0.0;
  private double latestTurnVelocity = 0.0;

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

    // Configure Drive Spark Max using centralized base config
    var driveConfig = createBaseConfig(DRIVE_MOTOR_CURRENT_LIMIT, false);
    driveConfig.secondaryCurrentLimit((int) DRIVE_MOTOR_SECONDARY_CURRENT_LIMIT.in(Amps));
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
    driveConfig.closedLoop.feedForward.kS(DRIVE_KS).kV(DRIVE_KV).kA(DRIVE_KA);
    driveConfig.closedLoop.maxMotion.maxAcceleration(200.0).allowedProfileError(0.0);
    driveConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs((int) (1000.0 / ODOMETRY_FREQUENCY.in(Hertz)))
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs((int) (1000.0 / ODOMETRY_FREQUENCY.in(Hertz)));

    tryUntilOk(
        5,
        () ->
            driveSpark.configure(
                driveConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters));
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

    // Configure Turn Spark Max using centralized base config
    var turnConfig = createBaseConfig(TURN_MOTOR_CURRENT_LIMIT, TURN_INVERTED);
    turnConfig.secondaryCurrentLimit((int) TURN_MOTOR_SECONDARY_CURRENT_LIMIT.in(Amps));
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
        .primaryEncoderVelocityPeriodMs((int) (1000.0 / ODOMETRY_FREQUENCY.in(Hertz)));

    tryUntilOk(
        5,
        () ->
            turnSpark.configure(
                turnConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters));

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
  public void updateHighFreq(double timestampSec) {
    double drivePos = driveTap.getLatencyCompensatedPosition();
    double turnPos = turnTap.getLatencyCompensatedPosition();

    moduleBuffer.offer(timestampSec, drivePos, turnPos);

    latestDrivePosition = drivePos;
    latestTurnPosition = turnPos;
    latestDriveVelocity = driveTap.getVelocity();
    latestTurnVelocity = turnTap.getVelocity();
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

    // Transfer high-frequency data to AdvantageKit inputs
    double[][] tsRef = {inputs.odometryTimestamps};
    double[][] driveRef = {inputs.odometryDrivePositionsRad};
    double[][] turnRef = {inputs.odometryTurnPositionsRad};
    moduleBuffer.drain(tsRef, driveRef, turnRef);
    inputs.odometryTimestamps = tsRef[0];
    inputs.odometryDrivePositionsRad = driveRef[0];
    inputs.odometryTurnPositionsRad = turnRef[0];

    // Assign standard telemetry from the last read in the high-frequency thread
    if (inputs.odometryTimestamps.length > 0) {
      inputs.drivePosition = Radians.of(latestDrivePosition);
      inputs.turnPosition = Radians.of(latestTurnPosition);
      inputs.driveVelocity = RadiansPerSecond.of(latestDriveVelocity);
      inputs.turnVelocity = RadiansPerSecond.of(latestTurnVelocity);
    }

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
        velocity.in(RadiansPerSecond), ControlType.kVelocity, ClosedLoopSlot.kSlot0);
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
