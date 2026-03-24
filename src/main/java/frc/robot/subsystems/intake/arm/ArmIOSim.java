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

  private Angle sentPosition = null;

  /** Creates a new ArmIOSim and initializes the simulated Spark MAX motor controllers. */
  public ArmIOSim() {
    // Create Spark MAX objects
    armSpark = new SparkMax(CAN_ID, MotorType.kBrushless);

    armEncoder = armSpark.getEncoder();
    armAbsEncoder = armSpark.getAbsoluteEncoder();
    armController = armSpark.getClosedLoopController();

    // Create SparkMaxSim wrappers
    armSparkSim = new SparkMaxSim(armSpark, GEARBOX);
    armAbsEncoderSim = armSparkSim.getAbsoluteEncoderSim();

    // Create physics models
    armPhysicsSim =
        new SingleJointedArmSim(
            GEARBOX,
            TOTAL_REDUCTION,
            ARM_MOMENT_OF_INERTIA.in(KilogramSquareMeters),
            ARM_LENGTH.in(Meters),
            DEPLOYED_POSITION.in(Radians),
            RETRACTED_POSITION.in(Radians),
            true,
            RETRACTED_POSITION.in(Radians));

    // Configure arm Spark MAX
    var armConfig = new SparkMaxConfig();
    armConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit((int) MOTOR_CURRENT_LIMIT.in(Amps))
        .voltageCompensation(12.0)
        .inverted(false);
    armConfig
        .encoder
        .positionConversionFactor(INTERNAL_ENCODER_POSITION_FACTOR)
        .velocityConversionFactor(INTERNAL_ENCODER_VELOCITY_FACTOR);
    armConfig
        .absoluteEncoder
        .positionConversionFactor(GLOBAL_ENCODER_POSITION_FACTOR)
        .velocityConversionFactor(GLOBAL_ENCODER_VELOCITY_FACTOR)
        .zeroOffset(GLOBAL_ENCODER_OFFSET)
        .inverted(true);
    armConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(KP, 0.0, KD);
    armConfig.closedLoop.feedForward.kV(KV).kA(KA).kS(KS).kCos(KG).kCosRatio(1.0 / (2.0 * Math.PI));
    armConfig
        .closedLoop
        .maxMotion
        .allowedProfileError(PID_TOLERANCE.in(Radians))
        .cruiseVelocity(CRUISE_VELOCITY.in(RadiansPerSecond))
        .maxAcceleration(MAX_ACCELERATION.in(RadiansPerSecondPerSecond));
    armConfig
        .softLimit
        .forwardSoftLimitEnabled(true)
        .forwardSoftLimit(RETRACTED_POSITION.plus(SOFT_LIMIT_TOLERANCE).in(Radians))
        .reverseSoftLimitEnabled(true)
        .reverseSoftLimit(DEPLOYED_POSITION.minus(SOFT_LIMIT_TOLERANCE).in(Radians));
    armSpark.configure(armConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    armEncoder.setPosition(RETRACTED_POSITION.in(Radians));
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
    if (sentPosition != null && angle.isEquivalent(sentPosition)) return;
    armController.setSetpoint(
        angle.in(Radians), ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);
    sentPosition = angle;
  }

  @Override
  public void stop() {
    armSpark.stopMotor();
    sentPosition = null;
  }
}
