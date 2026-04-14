// Copyright (c) 2026 FRC Team 4533 (Phoenix)
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.intake.arm;

import static edu.wpi.first.units.Units.*;
import static frc.lib.util.SparkUtil.*;
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
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

/** Simulation implementation for the intake arm IO interface. */
public class ArmIOSim implements ArmIO {
  private final SparkMax spark;
  private final SparkMaxSim sparkSim;
  private final SparkAbsoluteEncoderSim absEncoderSim;
  private final AbsoluteEncoder absEncoder;
  private final RelativeEncoder encoder;
  private final SparkClosedLoopController controller;
  private final SingleJointedArmSim physicsSim;

  private Angle sentPosition = null;

  /** Creates a new ArmIOSim and initializes the simulated Spark MAX motor controllers. */
  public ArmIOSim() {
    // Create Spark MAX objects
    spark = new SparkMax(CAN_ID, MotorType.kBrushless);

    encoder = spark.getEncoder();
    absEncoder = spark.getAbsoluteEncoder();
    controller = spark.getClosedLoopController();

    // Create SparkMaxSim wrappers
    sparkSim = new SparkMaxSim(spark, GEARBOX);
    absEncoderSim = sparkSim.getAbsoluteEncoderSim();

    // Create physics models
    physicsSim =
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
    var config = createBaseConfig(MOTOR_CURRENT_LIMIT, MOTOR_INVERTED);
    config
        .encoder
        .positionConversionFactor(INTERNAL_ENCODER_POSITION_FACTOR)
        .velocityConversionFactor(INTERNAL_ENCODER_VELOCITY_FACTOR);
    config
        .absoluteEncoder
        .positionConversionFactor(GLOBAL_ENCODER_POSITION_FACTOR)
        .velocityConversionFactor(GLOBAL_ENCODER_VELOCITY_FACTOR)
        .zeroOffset(GLOBAL_ENCODER_OFFSET)
        .inverted(true);
    config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(KP, 0.0, KD);
    config.closedLoop.feedForward.kV(KV).kA(KA).kS(KS).kCos(KG).kCosRatio(1.0 / (2.0 * Math.PI));
    config
        .closedLoop
        .maxMotion
        .allowedProfileError(PID_TOLERANCE.in(Radians))
        .cruiseVelocity(CRUISE_VELOCITY.in(RadiansPerSecond))
        .maxAcceleration(MAX_ACCELERATION.in(RadiansPerSecondPerSecond));
    config
        .softLimit
        .forwardSoftLimitEnabled(true)
        .forwardSoftLimit(RETRACTED_POSITION.plus(SOFT_LIMIT_TOLERANCE).in(Radians))
        .reverseSoftLimitEnabled(true)
        .reverseSoftLimit(DEPLOYED_POSITION.minus(SOFT_LIMIT_TOLERANCE).in(Radians));
    spark.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    encoder.setPosition(RETRACTED_POSITION.in(Radians));
  }

  /**
   * Updates inputs by simulating physics models and updating the SparkMaxSim state.
   *
   * @param inputs The inputs object to update.
   */
  @Override
  public void updateInputs(ArmIOInputs inputs) {
    // Update physics models with voltage from Spark sim
    physicsSim.setInputVoltage(sparkSim.getAppliedOutput() * RoboRioSim.getVInVoltage());

    // Advance physics
    physicsSim.update(0.02);

    // Update SparkMaxSim with physics results
    // iterate() expects velocity in units AFTER the encoder conversion factor (rad/s mechanism)
    sparkSim.iterate(physicsSim.getVelocityRadPerSec(), RoboRioSim.getVInVoltage(), 0.02);

    // Update absolute encoder sim with arm mechanism velocity (rad/s)
    absEncoderSim.iterate(physicsSim.getVelocityRadPerSec(), 0.02);

    // Populate logged inputs from Spark encoders
    inputs.connected = true;
    inputs.position = Radians.of(absEncoder.getPosition());
    inputs.velocity = RadiansPerSecond.of(absEncoder.getVelocity());
    inputs.appliedVoltage = Volts.of(spark.getAppliedOutput() * spark.getBusVoltage());
    inputs.appliedCurrent = Amps.of(spark.getOutputCurrent());
  }

  @Override
  public void setPosition(Angle angle) {
    if (sentPosition != null && angle.isEquivalent(sentPosition)) return;
    controller.setSetpoint(
        angle.in(Radians), ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);
    sentPosition = angle;
  }

  @Override
  public void stop() {
    spark.stopMotor();
    sentPosition = null;
  }
}
