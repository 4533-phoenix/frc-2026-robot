// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Modified by FRC Team 4533 (Phoenix) 2026
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import static frc.robot.subsystems.drive.DriveConstants.*;
import static frc.robot.util.SparkUtil.*;

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
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import java.util.Queue;
import java.util.function.DoubleSupplier;

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

  private final Queue<Double> timestampQueue;
  private final Queue<Double> drivePositionQueue;
  private final Queue<Double> turnPositionQueue;

  private final Debouncer driveConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer turnConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  public ModuleIOSpark(int module) {
    zeroRotation =
        switch (module) {
          case 0 -> frontLeftZeroRotation;
          case 1 -> frontRightZeroRotation;
          case 2 -> backLeftZeroRotation;
          case 3 -> backRightZeroRotation;
          default -> Rotation2d.kZero;
        };
    driveSpark =
        new SparkMax(
            switch (module) {
              case 0 -> frontLeftDriveCanId;
              case 1 -> frontRightDriveCanId;
              case 2 -> backLeftDriveCanId;
              case 3 -> backRightDriveCanId;
              default -> 0;
            },
            MotorType.kBrushless);
    turnSpark =
        new SparkMax(
            switch (module) {
              case 0 -> frontLeftTurnCanId;
              case 1 -> frontRightTurnCanId;
              case 2 -> backLeftTurnCanId;
              case 3 -> backRightTurnCanId;
              default -> 0;
            },
            MotorType.kBrushless);
    turnEncoder =
        new CANcoder(
            switch (module) {
              case 0 -> frontLeftEncoderCanId;
              case 1 -> frontRightEncoderCanId;
              case 2 -> backLeftEncoderCanId;
              case 3 -> backRightEncoderCanId;
              default -> 0;
            });

    driveEncoder = driveSpark.getEncoder();
    turnInternalEncoder = turnSpark.getEncoder();
    driveController = driveSpark.getClosedLoopController();
    turnController = turnSpark.getClosedLoopController();
    turnAbsolutePositionSignal = turnEncoder.getPosition();
    turnVelocitySignal = turnEncoder.getVelocity();

    var driveConfig = new SparkMaxConfig();
    driveConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(driveMotorCurrentLimit)
        .secondaryCurrentLimit(driveMotorSecondaryCurrentLimit)
        .voltageCompensation(12.0);
    driveConfig
        .encoder
        .positionConversionFactor(driveEncoderPositionFactor)
        .velocityConversionFactor(driveEncoderVelocityFactor)
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2);
    driveConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(driveKp, 0.0, driveKd);
    driveConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs((int) (1000.0 / odometryFrequency))
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(50)
        .busVoltagePeriodMs(50)
        .outputCurrentPeriodMs(50);
    tryUntilOk(
        driveSpark,
        5,
        () ->
            driveSpark.configure(
                driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    tryUntilOk(driveSpark, 5, () -> driveEncoder.setPosition(0.0));

    var turnEncoderConfig = new CANcoderConfiguration();
    turnEncoderConfig.MagnetSensor.MagnetOffset = 0.0;
    turnEncoderConfig.MagnetSensor.SensorDirection =
        turnEncoderInverted
            ? SensorDirectionValue.Clockwise_Positive
            : SensorDirectionValue.CounterClockwise_Positive;
    turnEncoder.getConfigurator().apply(turnEncoderConfig);
    turnAbsolutePositionSignal.setUpdateFrequency(odometryFrequency);
    turnVelocitySignal.setUpdateFrequency(50);

    var turnConfig = new SparkMaxConfig();
    turnConfig
        .inverted(turnInverted)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(turnMotorCurrentLimit)
        .secondaryCurrentLimit(turnMotorSecondaryCurrentLimit)
        .voltageCompensation(12.0);
    turnConfig
        .encoder
        .positionConversionFactor((2.0 * Math.PI) / turnMotorReduction)
        .velocityConversionFactor(((2.0 * Math.PI) / 60.0) / turnMotorReduction);
    turnConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(turnKp, 0.0, turnKd)
        .positionWrappingEnabled(true)
        .positionWrappingInputRange(turnPIDMinInput, turnPIDMaxInput);
    turnConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs(20)
        .primaryEncoderVelocityAlwaysOn(false)
        .appliedOutputPeriodMs(50)
        .busVoltagePeriodMs(50)
        .outputCurrentPeriodMs(50);
    tryUntilOk(
        turnSpark,
        5,
        () ->
            turnSpark.configure(
                turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    tryUntilOk(
        turnSpark,
        5,
        () -> {
          turnAbsolutePositionSignal.refresh();
          if (turnAbsolutePositionSignal.getStatus().isOK()) {
            Rotation2d currentRot =
                new Rotation2d(
                        turnAbsolutePositionSignal.getValueAsDouble() * turnEncoderPositionFactor)
                    .minus(zeroRotation);
            return turnInternalEncoder.setPosition(currentRot.getRadians());
          } else {
            return turnInternalEncoder.setPosition(0.0);
          }
        });

    timestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();
    drivePositionQueue =
        SparkOdometryThread.getInstance().registerSignal(driveSpark, driveEncoder::getPosition);
    turnPositionQueue =
        SparkOdometryThread.getInstance()
            .registerSignal(
                () ->
                    turnAbsolutePositionSignal.refresh().getValueAsDouble()
                        * turnEncoderPositionFactor);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    sparkStickyFault = false;
    ifOk(driveSpark, driveEncoder::getPosition, (value) -> inputs.drivePositionRad = value);
    ifOk(driveSpark, driveEncoder::getVelocity, (value) -> inputs.driveVelocityRadPerSec = value);
    ifOk(
        driveSpark,
        new DoubleSupplier[] {driveSpark::getAppliedOutput, driveSpark::getBusVoltage},
        (values) -> inputs.driveAppliedVolts = values[0] * values[1]);
    ifOk(driveSpark, driveSpark::getOutputCurrent, (value) -> inputs.driveCurrentAmps = value);
    inputs.driveConnected = driveConnectedDebounce.calculate(!sparkStickyFault);

    BaseStatusSignal.refreshAll(turnAbsolutePositionSignal, turnVelocitySignal);

    if (turnAbsolutePositionSignal.getStatus().isOK()) {
      currentTurnPosition =
          new Rotation2d(turnAbsolutePositionSignal.getValueAsDouble() * turnEncoderPositionFactor)
              .minus(zeroRotation);
      inputs.turnVelocityRadPerSec =
          turnVelocitySignal.getValueAsDouble() * turnEncoderVelocityFactor;

      double internalPos = turnInternalEncoder.getPosition();
      double absolutePos = currentTurnPosition.getRadians();
      double turnError = Math.abs(internalPos - absolutePos);
      boolean isStill = Math.abs(inputs.turnVelocityRadPerSec) < velocityGateRadPerSec;

      if (isStill && turnError > errorThresholdRad) {
        turnInternalEncoder.setPosition(absolutePos);
      }
      inputs.turnConnected = turnConnectedDebounce.calculate(true);
    } else {
      currentTurnPosition = new Rotation2d(turnInternalEncoder.getPosition());
      inputs.turnVelocityRadPerSec = turnInternalEncoder.getVelocity();
      inputs.turnConnected = turnConnectedDebounce.calculate(false);
    }
    inputs.turnPosition = currentTurnPosition;

    sparkStickyFault = false;
    ifOk(
        turnSpark,
        new DoubleSupplier[] {turnSpark::getAppliedOutput, turnSpark::getBusVoltage},
        (values) -> inputs.turnAppliedVolts = values[0] * values[1]);
    ifOk(turnSpark, turnSpark::getOutputCurrent, (value) -> inputs.turnCurrentAmps = value);

    int count = timestampQueue.size();
    inputs.odometryTimestamps = new double[count];
    inputs.odometryDrivePositionsRad = new double[count];
    inputs.odometryTurnPositions = new Rotation2d[count];

    int i = 0;
    for (Double timestamp : timestampQueue) {
      inputs.odometryTimestamps[i++] = timestamp;
    }

    i = 0;
    for (Double position : drivePositionQueue) {
      inputs.odometryDrivePositionsRad[i++] = position;
    }

    i = 0;
    for (Double position : turnPositionQueue) {
      inputs.odometryTurnPositions[i++] = new Rotation2d(position).minus(zeroRotation);
    }

    timestampQueue.clear();
    drivePositionQueue.clear();
    turnPositionQueue.clear();
  }

  @Override
  public void setDriveOpenLoop(double output) {
    driveSpark.setVoltage(output);
  }

  @Override
  public void setTurnOpenLoop(double output) {
    turnSpark.setVoltage(output);
  }

  @Override
  public void setDriveVelocity(double velocityRadPerSec) {
    double ffVolts = driveKs * Math.signum(velocityRadPerSec) + driveKv * velocityRadPerSec;
    driveController.setSetpoint(
        velocityRadPerSec,
        ControlType.kVelocity,
        ClosedLoopSlot.kSlot0,
        ffVolts,
        ArbFFUnits.kVoltage);
  }

  @Override
  public void setTurnPosition(Rotation2d rotation) {
    turnController.setSetpoint(rotation.getRadians(), ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }
}
