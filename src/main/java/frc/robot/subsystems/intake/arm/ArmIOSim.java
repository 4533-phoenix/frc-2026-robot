// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.intake.arm;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.intake.arm.ArmConstants.*;

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
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

/** Simulation implementation for the intake arm IO interface. */
public class ArmIOSim implements ArmIO {
  private final SparkMax armSpark;
  private final SparkMaxSim armSparkSim;
  private final SparkAbsoluteEncoderSim armAbsEncoderSim;
  private final AbsoluteEncoder armAbsEncoder;
  private final RelativeEncoder armEncoder;
  private final SparkClosedLoopController armController;
  private final SingleJointedArmSim armPhysicsSim;

  /** Creates a new ArmIOSim and initializes the simulated Spark MAX motor controllers. */
  public ArmIOSim() {
    // Create Spark MAX objects
    armSpark = new SparkMax(canId, MotorType.kBrushless);

    armEncoder = armSpark.getEncoder();
    armAbsEncoder = armSpark.getAbsoluteEncoder();
    armController = armSpark.getClosedLoopController();

    // Create SparkMaxSim wrappers
    armSparkSim = new SparkMaxSim(armSpark, gearbox);
    armAbsEncoderSim = armSparkSim.getAbsoluteEncoderSim();

    // Create physics models
    armPhysicsSim =
        new SingleJointedArmSim(
            gearbox,
            motorReduction,
            0.5,
            0.5,
            deployedPosition.in(Radians),
            retractedPosition.in(Radians),
            true,
            retractedPosition.in(Radians));

    // Configure arm Spark MAX (mirrors IntakeIOReal)
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
        .softLimit
        .forwardSoftLimitEnabled(true)
        .forwardSoftLimit(retractedPosition.plus(softLimitTolerance).in(Radians))
        .reverseSoftLimitEnabled(true)
        .reverseSoftLimit(deployedPosition.minus(softLimitTolerance).in(Radians));
    armSpark.configure(armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    // Set initial encoder position to retracted
    armEncoder.setPosition(retractedPosition.in(Radians));
  }

  /**
   * Updates inputs by simulating physics models and updating the SparkMaxSim state.
   *
   * @param inputs The inputs object to update.
   */
  @Override
  public void updateInputs(ArmIOInputs inputs) {
    // Update physics models with voltage from Spark sim
    armPhysicsSim.setInputVoltage(armSparkSim.getAppliedOutput() * RoboRioSim.getVInVoltage());

    // Advance physics
    armPhysicsSim.update(0.02);

    // Update SparkMaxSim with physics results
    // iterate() expects velocity in units AFTER the encoder conversion factor (rad/s mechanism)
    armSparkSim.iterate(armPhysicsSim.getVelocityRadPerSec(), RoboRioSim.getVInVoltage(), 0.02);

    // Update absolute encoder sim with arm mechanism velocity (rad/s)
    armAbsEncoderSim.iterate(armPhysicsSim.getVelocityRadPerSec(), 0.02);

    // Populate logged inputs from Spark encoders
    inputs.connected = true;
    inputs.position = Radians.of(armAbsEncoder.getPosition());
    inputs.velocity = RadiansPerSecond.of(armAbsEncoder.getVelocity());
    inputs.appliedVoltage = Volts.of(armSpark.getAppliedOutput() * armSpark.getBusVoltage());
    inputs.appliedCurrent = Amps.of(armSpark.getOutputCurrent());
  }

  @Override
  public void setPosition(Angle angle) {
    armController.setSetpoint(
        angle.in(Radians), ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);
  }
}
