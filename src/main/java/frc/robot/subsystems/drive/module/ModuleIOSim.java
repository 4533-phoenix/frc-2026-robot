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
import static frc.robot.subsystems.drive.DriveConstants.*;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;

/**
 * Physics simulation implementation of {@link ModuleIO}.
 *
 * <p>This class uses {@link SparkMaxSim} to simulate the Spark MAX motor controllers, backed by
 * {@link DCMotorSim} physics models for accurate motor behavior. The Spark's built-in closed-loop
 * PID is used for velocity and position control, matching the real robot's {@link ModuleIOSpark}.
 */
public class ModuleIOSim implements ModuleIO {
  private final SparkMax driveSpark;
  private final SparkMax turnSpark;
  private final SparkMaxSim driveSparkSim;
  private final SparkMaxSim turnSparkSim;
  private final RelativeEncoder driveEncoder;
  private final RelativeEncoder turnEncoder;
  private final SparkClosedLoopController driveController;
  private final SparkClosedLoopController turnController;
  private final DCMotorSim driveMotorSim;
  private final DCMotorSim turnMotorSim;

  /**
   * Creates a new ModuleIOSim and initializes the simulated Spark MAX motor controllers.
   *
   * @param module The module index (0 for front-left, 1 for front-right, 2 for back-left, 3 for
   *     back-right).
   */
  public ModuleIOSim(int module) {
    var config = moduleConfigs[module];

    // Create real Spark MAX objects (they run in sim mode automatically)
    driveSpark = new SparkMax(config.driveCanId(), MotorType.kBrushless);
    turnSpark = new SparkMax(config.turnCanId(), MotorType.kBrushless);

    driveEncoder = driveSpark.getEncoder();
    turnEncoder = turnSpark.getEncoder();
    driveController = driveSpark.getClosedLoopController();
    turnController = turnSpark.getClosedLoopController();

    // Create SparkMaxSim wrappers backed by DCMotor models
    driveSparkSim = new SparkMaxSim(driveSpark, driveGearbox);
    turnSparkSim = new SparkMaxSim(turnSpark, turnGearbox);

    // Create DCMotorSim physics models for motor dynamics
    driveMotorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(driveGearbox, 0.025, driveMotorReduction),
            driveGearbox);
    turnMotorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(turnGearbox, 0.004, turnMotorReduction),
            turnGearbox);

    // Configure drive Spark MAX
    var driveConfig = new SparkMaxConfig();
    driveConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit((int) driveMotorCurrentLimit.in(Amps))
        .voltageCompensation(12.0);
    driveConfig
        .encoder
        .positionConversionFactor(driveEncoderPositionFactor)
        .velocityConversionFactor(driveEncoderVelocityFactor);
    driveConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(driveKp, 0.0, driveKd);
    driveSpark.configure(
        driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Configure turn Spark MAX
    var turnConfig = new SparkMaxConfig();
    turnConfig
        .inverted(turnInverted)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit((int) turnMotorCurrentLimit.in(Amps))
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
    turnSpark.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  /**
   * Updates the simulation state using SparkMaxSim and physics models, then updates loggable
   * inputs.
   *
   * @param inputs The inputs object to update with simulated data.
   */
  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    // Update physics models with the voltage applied by the Spark sim
    driveMotorSim.setInputVoltage(driveSparkSim.getAppliedOutput() * RoboRioSim.getVInVoltage());
    turnMotorSim.setInputVoltage(turnSparkSim.getAppliedOutput() * RoboRioSim.getVInVoltage());

    // Advance physics simulation by 20ms (standard robot loop time)
    driveMotorSim.update(0.02);
    turnMotorSim.update(0.02);

    // Update SparkMaxSim with physics model results
    // iterate() expects velocity in units AFTER the encoder conversion factor.
    // Our conversion factors convert motor RPM to mechanism-side rad/s,
    // so we pass mechanism-side rad/s from the DCMotorSim.
    driveSparkSim.iterate(
        driveMotorSim.getAngularVelocityRadPerSec(), RoboRioSim.getVInVoltage(), 0.02);
    turnSparkSim.iterate(
        turnMotorSim.getAngularVelocityRadPerSec(), RoboRioSim.getVInVoltage(), 0.02);

    // Update drive inputs with simulated data
    inputs.driveConnected = true;
    inputs.drivePosition = Radians.of(driveEncoder.getPosition());
    inputs.driveVelocity = RadiansPerSecond.of(driveEncoder.getVelocity());
    inputs.driveAppliedVoltage =
        Volts.of(driveSpark.getAppliedOutput() * driveSpark.getBusVoltage());
    inputs.driveCurrent = Amps.of(driveSpark.getOutputCurrent());

    // Update turn inputs with simulated data
    inputs.turnConnected = true;
    inputs.turnPosition = Radians.of(turnEncoder.getPosition());
    inputs.turnVelocity = RadiansPerSecond.of(turnEncoder.getVelocity());
    inputs.turnAppliedVoltage = Volts.of(turnSpark.getAppliedOutput() * turnSpark.getBusVoltage());
    inputs.turnCurrent = Amps.of(turnSpark.getOutputCurrent());

    // Update odometry inputs (single sample per loop in sim)
    inputs.odometryTimestamps = new double[] {Timer.getFPGATimestamp()};
    inputs.odometryDrivePositionsRad = new double[] {driveEncoder.getPosition()};
    inputs.odometryTurnPositions =
        new Rotation2d[] {Rotation2d.fromRadians(turnEncoder.getPosition())};
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
        driveKs * Math.signum(velocity.in(RadiansPerSecond))
            + driveKv * velocity.in(RadiansPerSecond);
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
