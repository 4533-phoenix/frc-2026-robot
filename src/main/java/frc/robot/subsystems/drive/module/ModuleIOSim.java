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

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.lib.HighFreqBuffer;

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

  // High-frequency data tracking
  private final HighFreqBuffer moduleBuffer = new HighFreqBuffer(3);

  private double latestDrivePosition = 0.0;
  private double latestTurnPosition = 0.0;
  private double latestDriveVelocity = 0.0;
  private double latestTurnVelocity = 0.0;

  /**
   * Creates a new ModuleIOSim and initializes the simulated Spark MAX motor controllers.
   *
   * @param module The module index (0 for front-left, 1 for front-right, 2 for back-left, 3 for
   *     back-right).
   */
  public ModuleIOSim(int module) {
    var config = MODULE_CONFIGS[module];

    // Create real Spark MAX objects (they run in sim mode automatically)
    driveSpark = new SparkMax(config.driveCanId(), MotorType.kBrushless);
    turnSpark = new SparkMax(config.turnCanId(), MotorType.kBrushless);

    driveEncoder = driveSpark.getEncoder();
    turnEncoder = turnSpark.getEncoder();
    driveController = driveSpark.getClosedLoopController();
    turnController = turnSpark.getClosedLoopController();

    // Create SparkMaxSim wrappers backed by DCMotor models
    driveSparkSim = new SparkMaxSim(driveSpark, DRIVE_GEARBOX);
    turnSparkSim = new SparkMaxSim(turnSpark, TURN_GEARBOX);

    // Create DCMotorSim physics models for motor dynamics
    driveMotorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DRIVE_GEARBOX, 0.025, DRIVE_MOTOR_REDUCTION),
            DRIVE_GEARBOX);
    turnMotorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(TURN_GEARBOX, 0.004, TURN_MOTOR_REDUCTION),
            TURN_GEARBOX);

    // Configure drive Spark MAX
    var driveConfig = createBaseConfig(DRIVE_MOTOR_CURRENT_LIMIT, DRIVE_INVERTED);
    driveConfig
        .encoder
        .positionConversionFactor(DRIVE_ENCODER_POSITION_FACTOR)
        .velocityConversionFactor(DRIVE_ENCODER_VELOCITY_FACTOR);
    driveConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(DRIVE_KP, 0.0, DRIVE_KD);
    driveConfig.closedLoop.feedForward.kS(DRIVE_KS).kV(DRIVE_KV).kA(DRIVE_KA);
    driveConfig.closedLoop.maxMotion.maxAcceleration(200.0).allowedProfileError(0.0);
    driveSpark.configure(
        driveConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    // Configure turn Spark MAX
    var turnConfig = createBaseConfig(TURN_MOTOR_CURRENT_LIMIT, TURN_INVERTED);
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
    turnSpark.configure(
        turnConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void updateHighFreq(double timestampSec) {
    // Update physics models with the voltage applied by the Spark sim
    driveMotorSim.setInputVoltage(driveSparkSim.getAppliedOutput() * RoboRioSim.getVInVoltage());
    turnMotorSim.setInputVoltage(turnSparkSim.getAppliedOutput() * RoboRioSim.getVInVoltage());

    // Advance physics simulation by 5ms (200Hz loop)
    driveMotorSim.update(0.005);
    turnMotorSim.update(0.005);

    // Update SparkMaxSim with physics model results
    driveSparkSim.iterate(
        driveMotorSim.getAngularVelocityRadPerSec(), RoboRioSim.getVInVoltage(), 0.005);
    turnSparkSim.iterate(
        turnMotorSim.getAngularVelocityRadPerSec(), RoboRioSim.getVInVoltage(), 0.005);

    latestDrivePosition = driveEncoder.getPosition();
    latestTurnPosition = turnEncoder.getPosition();
    latestDriveVelocity = driveEncoder.getVelocity();
    latestTurnVelocity = turnEncoder.getVelocity();

    moduleBuffer.offer(timestampSec, latestDrivePosition, latestTurnPosition, latestDriveVelocity);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    inputs.driveConnected = true;
    inputs.turnConnected = true;
    inputs.turnEncoderConnected = true;

    inputs.driveAppliedVoltage =
        Volts.of(driveSpark.getAppliedOutput() * driveSpark.getBusVoltage());
    inputs.driveCurrent = Amps.of(driveSpark.getOutputCurrent());

    inputs.turnAppliedVoltage = Volts.of(turnSpark.getAppliedOutput() * turnSpark.getBusVoltage());
    inputs.turnCurrent = Amps.of(turnSpark.getOutputCurrent());

    // Transfer high-frequency data to AdvantageKit inputs
    double[][] tsRef = {inputs.odometryTimestamps};
    double[][] driveRef = {inputs.odometryDrivePositionsRad};
    double[][] turnRef = {inputs.odometryTurnPositionsRad};
    double[][] driveVelRef = {inputs.odometryDriveVelocitiesRadPerSec};
    moduleBuffer.drain(tsRef, driveRef, turnRef, driveVelRef);
    inputs.odometryTimestamps = tsRef[0];
    inputs.odometryDrivePositionsRad = driveRef[0];
    inputs.odometryTurnPositionsRad = turnRef[0];
    inputs.odometryDriveVelocitiesRadPerSec = driveVelRef[0];

    // Assign standard telemetry from the last read in the high-frequency thread
    if (inputs.odometryTimestamps.length > 0) {
      inputs.drivePosition = Radians.of(latestDrivePosition);
      inputs.turnPosition = Radians.of(latestTurnPosition);
      inputs.driveVelocity = RadiansPerSecond.of(latestDriveVelocity);
      inputs.turnVelocity = RadiansPerSecond.of(latestTurnVelocity);
    }

    inputs.driveHealthy = true;
    inputs.turnHealthy = true;
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
}
