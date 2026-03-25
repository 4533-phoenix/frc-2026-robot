// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.shooter.flywheel;

import static edu.wpi.first.units.Units.*;
import static frc.lib.util.SparkUtil.*;
import static frc.robot.subsystems.shooter.ShooterConstants.*;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;

/**
 * Simulation implementation of {@link FlywheelIO}.
 *
 * <p>This class uses a {@link SparkFlex} with its {@link SparkFlexSim} wrapper to simulate the
 * flywheel motor controller, backed by a {@link DCMotorSim} physics model. The SparkFlex's built-in
 * MAXMotion velocity control is used, matching the real robot's {@link FlywheelIOSpark}.
 * Configuration is identical to the real implementation.
 */
public class FlywheelIOSim implements FlywheelIO {
  private final SparkFlex spark;
  private final SparkFlexSim sparkSim;
  private final RelativeEncoder encoder;
  private final SparkClosedLoopController controller;

  // Physics model for the flywheel
  private final DCMotorSim physicsSim;

  // Cache the last sent velocity to avoid redundant CAN writes
  private AngularVelocity sentVelocity = null;

  /**
   * Creates a new FlywheelIOSim and configures the SparkFlex identically to {@link
   * FlywheelIOSpark}.
   */
  public FlywheelIOSim() {
    spark = new SparkFlex(CAN_ID, MotorType.kBrushless);
    sparkSim = new SparkFlexSim(spark, GEARBOX);
    encoder = spark.getEncoder();
    controller = spark.getClosedLoopController();

    physicsSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(GEARBOX, MOI.in(KilogramSquareMeters), REDUCTION),
            GEARBOX);

    // Configuration mirrors FlywheelIOSparkFlex exactly
    var config = createBaseConfig(MOTOR_CURRENT_LIMIT, FLYWHEEL_INVERTED);
    config.idleMode(IdleMode.kCoast);
    config
        .encoder
        .positionConversionFactor(FLYWHEEL_ENCODER_POSITION_FACTOR)
        .velocityConversionFactor(FLYWHEEL_ENCODER_VELOCITY_FACTOR);
    config
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(FLYWHEEL_KP, FLYWHEEL_KI, FLYWHEEL_KD);
    config.closedLoop.feedForward.kS(FLYWHEEL_KS).kV(FLYWHEEL_KV).kA(FLYWHEEL_KA);
    spark.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  /**
   * Updates the physics simulation and populates inputs from the SparkFlex sim state.
   *
   * @param inputs The inputs object to update.
   */
  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    // Get the motor voltage output and feed it into the physics model
    physicsSim.setInputVoltage(sparkSim.getAppliedOutput() * RoboRioSim.getVInVoltage());
    physicsSim.update(0.02);
    sparkSim.iterate(physicsSim.getAngularVelocityRadPerSec(), RoboRioSim.getVInVoltage(), 0.02);

    // Populate inputs from simulated data
    inputs.connected = true;
    inputs.position = Radians.of(encoder.getPosition());
    inputs.velocity = RadiansPerSecond.of(encoder.getVelocity());
    inputs.appliedVoltage = Volts.of(spark.getAppliedOutput() * spark.getBusVoltage());
    inputs.appliedCurrent = Amps.of(spark.getOutputCurrent());
  }

  @Override
  public void runCharacterization(Voltage voltage) {
    spark.setVoltage(voltage.in(Volts));
  }

  @Override
  public void setAngularVelocity(AngularVelocity velocity) {
    if (sentVelocity != null && velocity.isEquivalent(sentVelocity)) return;
    controller.setSetpoint(
        velocity.in(RadiansPerSecond), ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    sentVelocity = velocity;
  }

  @Override
  public void stop() {
    spark.setVoltage(0.0);
    sentVelocity = null;
  }
}
