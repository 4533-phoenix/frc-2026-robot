// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.intake.spinner;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.intake.spinner.SpinnerConstants.*;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;

public class SpinnerIOSim implements SpinnerIO {
  // Spinner motor and sim
  private final SparkMax spinnerSpark;
  private final SparkMaxSim spinnerSparkSim;
  private final RelativeEncoder spinnerEncoder;
  private final DCMotorSim spinnerPhysicsSim;

  /** Creates a new SpinnerIOSim and initializes the simulated Spark MAX motor controllers. */
  public SpinnerIOSim() {
    // Create Spark MAX objects
    spinnerSpark = new SparkMax(canId, MotorType.kBrushless);
    spinnerEncoder = spinnerSpark.getEncoder();

    // Create SparkMaxSim wrappers
    spinnerSparkSim = new SparkMaxSim(spinnerSpark, gearbox);

    // Create physics models
    spinnerPhysicsSim =
        new DCMotorSim(LinearSystemId.createDCMotorSystem(gearbox, 0.005, motorReduction), gearbox);

    // Configure spinner Spark MAX (mirrors IntakeIOReal)
    var spinnerConfig = new SparkMaxConfig();
    spinnerConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit((int) motorCurrentLimit.in(Amps))
        .voltageCompensation(12.0)
        .inverted(true);
    spinnerConfig
        .encoder
        .positionConversionFactor(internalEncoderPositionFactor)
        .velocityConversionFactor(internalEncoderVelocityFactor);
    spinnerSpark.configure(
        spinnerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  /**
   * Updates inputs by simulating physics models and updating the SparkMaxSim state.
   *
   * @param inputs The inputs object to update.
   */
  @Override
  public void updateInputs(SpinnerIOInputs inputs) {
    // Advance physics
    spinnerPhysicsSim.update(0.02);

    // Update SparkMaxSim with physics results
    // iterate() expects velocity in units AFTER the encoder conversion factor (rad/s mechanism)
    spinnerSparkSim.iterate(
        spinnerPhysicsSim.getAngularVelocityRadPerSec(), RoboRioSim.getVInVoltage(), 0.02);

    // Populate logged inputs from Spark encoders
    inputs.connected = true;
    inputs.velocity = RadiansPerSecond.of(spinnerEncoder.getVelocity());
    inputs.appliedVoltage =
        Volts.of(spinnerSpark.getAppliedOutput() * spinnerSpark.getBusVoltage());
    inputs.appliedCurrent = Amps.of(spinnerSpark.getOutputCurrent());
  }

  @Override
  public void setVoltage(Voltage voltage) {
    spinnerSpark.setVoltage(voltage.in(Volts));
  }
}
