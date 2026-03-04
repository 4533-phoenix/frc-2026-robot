// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.intake.IntakeConstants.*;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

/**
 * Simulation implementation of {@link IntakeIO}.
 *
 * <p>This class uses {@link SparkMaxSim} to simulate the Spark MAX motor controllers for the arm
 * and spinner, backed by WPILib physics models for accurate motor behavior. The Spark's built-in
 * closed-loop control is used, matching the real robot's {@link IntakeIOReal}.
 */
public class IntakeIOSim implements IntakeIO {
  // Arm motor and sim
  private final SparkMax armSpark;
  private final SparkMaxSim armSparkSim;
  private final SparkAbsoluteEncoderSim armAbsEncoderSim;
  private final AbsoluteEncoder armAbsEncoder;
  private final RelativeEncoder armEncoder;
  private final SparkClosedLoopController armController;
  private final SingleJointedArmSim armPhysicsSim;

  // Spinner motor and sim
  private final SparkMax spinnerSpark;
  private final SparkMaxSim spinnerSparkSim;
  private final RelativeEncoder spinnerEncoder;
  private final SparkClosedLoopController spinnerController;
  private final DCMotorSim spinnerPhysicsSim;

  /** Creates a new IntakeIOSim and initializes the simulated Spark MAX motor controllers. */
  public IntakeIOSim() {
    // Create Spark MAX objects
    armSpark = new SparkMax(armMotorCanId, MotorType.kBrushless);
    spinnerSpark = new SparkMax(spinnerMotorCanId, MotorType.kBrushless);

    armEncoder = armSpark.getEncoder();
    armAbsEncoder = armSpark.getAbsoluteEncoder();
    spinnerEncoder = spinnerSpark.getEncoder();
    armController = armSpark.getClosedLoopController();
    spinnerController = spinnerSpark.getClosedLoopController();

    // Create SparkMaxSim wrappers
    armSparkSim = new SparkMaxSim(armSpark, DCMotor.getNEO(1));
    spinnerSparkSim = new SparkMaxSim(spinnerSpark, DCMotor.getNEO(1));
    armAbsEncoderSim = armSparkSim.getAbsoluteEncoderSim();

    // Create physics models
    armPhysicsSim =
        new SingleJointedArmSim(
            DCMotor.getNEO(1),
            armMotorReduction,
            0.5, // Simulated Moment of Inertia
            0.5, // Simulated length in meters
            armDeployedPosition.in(Radians), // min angle
            armRetractedPosition.in(Radians), // max angle
            true, // simulate gravity
            armRetractedPosition.in(Radians) // starting angle
            );
    spinnerPhysicsSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), 0.005, spinnerMotorReduction),
            DCMotor.getNEO(1));

    // Configure arm Spark MAX (mirrors IntakeIOReal)
    var armConfig = new SparkMaxConfig();
    armConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit((int) armMotorCurrentLimit.in(Amps))
        .voltageCompensation(12.0)
        .inverted(false);
    armConfig
        .encoder
        .positionConversionFactor(armInternalEncoderPositionFactor)
        .velocityConversionFactor(armInternalEncoderVelocityFactor);
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
        .allowedProfileError(armPositionPIDTolerance.in(Radians))
        .cruiseVelocity(armCruiseVelocity.in(RadiansPerSecond))
        .maxAcceleration(armMaxAcceleration.in(RadiansPerSecondPerSecond));
    armConfig
        .softLimit
        .forwardSoftLimitEnabled(true)
        .forwardSoftLimit(armRetractedPosition.plus(armPositionSoftLimitTolerance).in(Radians))
        .reverseSoftLimitEnabled(true)
        .reverseSoftLimit(armDeployedPosition.minus(armPositionSoftLimitTolerance).in(Radians));
    armSpark.configure(armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    // Set initial encoder position to retracted
    armEncoder.setPosition(armRetractedPosition.in(Radians));

    // Configure spinner Spark MAX (mirrors IntakeIOReal)
    var spinnerConfig = new SparkMaxConfig();
    spinnerConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit((int) spinnerMotorCurrentLimit.in(Amps))
        .voltageCompensation(12.0)
        .inverted(true);
    spinnerConfig
        .encoder
        .positionConversionFactor(spinnerInternalEncoderPositionFactor)
        .velocityConversionFactor(spinnerInternalEncoderVelocityFactor);
    spinnerSpark.configure(
        spinnerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  /**
   * Updates inputs by simulating physics models and updating the SparkMaxSim state.
   *
   * @param inputs The inputs object to update.
   */
  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    // Update physics models with voltage from Spark sim
    armPhysicsSim.setInputVoltage(armSparkSim.getAppliedOutput() * RoboRioSim.getVInVoltage());
    spinnerPhysicsSim.setInputVoltage(
        spinnerSparkSim.getAppliedOutput() * RoboRioSim.getVInVoltage());

    // Advance physics
    armPhysicsSim.update(0.02);
    spinnerPhysicsSim.update(0.02);

    // Update SparkMaxSim with physics results
    // iterate() expects velocity in units AFTER the encoder conversion factor (rad/s mechanism)
    armSparkSim.iterate(armPhysicsSim.getVelocityRadPerSec(), RoboRioSim.getVInVoltage(), 0.02);
    spinnerSparkSim.iterate(
        spinnerPhysicsSim.getAngularVelocityRadPerSec(), RoboRioSim.getVInVoltage(), 0.02);

    // Update absolute encoder sim with arm mechanism velocity (rad/s)
    armAbsEncoderSim.iterate(armPhysicsSim.getVelocityRadPerSec(), 0.02);

    // Populate logged inputs from Spark encoders
    inputs.armConnected = true;
    inputs.armPosition = Radians.of(armAbsEncoder.getPosition());
    inputs.armVelocity = RadiansPerSecond.of(armAbsEncoder.getVelocity());
    inputs.armAppliedVoltage = Volts.of(armSpark.getAppliedOutput() * armSpark.getBusVoltage());
    inputs.armAppliedCurrent = Amps.of(armSpark.getOutputCurrent());

    inputs.spinnerConnected = true;
    inputs.spinnerVelocity = RadiansPerSecond.of(spinnerEncoder.getVelocity());
    inputs.spinnerAppliedVoltage =
        Volts.of(spinnerSpark.getAppliedOutput() * spinnerSpark.getBusVoltage());
    inputs.spinnerAppliedCurrent = Amps.of(spinnerSpark.getOutputCurrent());
  }

  @Override
  public void setArmPosition(Angle angle) {
    armController.setSetpoint(
        angle.in(Radians), ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);
  }

  @Override
  public void setSpinnerVoltage(Voltage voltage) {
    spinnerController.setSetpoint(voltage.in(Volts), ControlType.kVoltage, ClosedLoopSlot.kSlot0);
  }
}
