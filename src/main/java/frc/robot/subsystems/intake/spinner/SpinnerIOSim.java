// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.intake.spinner;

import static edu.wpi.first.units.Units.*;
import static frc.lib.util.SparkUtil.*;
import static frc.robot.subsystems.intake.spinner.SpinnerConstants.*;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;

/** Simulation implementation of the Spinner IO interface. */
public class SpinnerIOSim implements SpinnerIO {
  // Spinner motor and sim
  private final SparkMax spark;
  private final SparkMaxSim sparkSim;
  private final DCMotorSim physiscsSim;

  /** Creates a new SpinnerIOSim and initializes the simulated Spark MAX motor controllers. */
  public SpinnerIOSim() {
    // Create Spark MAX objects
    spark = new SparkMax(CAN_ID, MotorType.kBrushless);

    // Create SparkMaxSim wrappers
    sparkSim = new SparkMaxSim(spark, GEARBOX);

    // Create physics models
    physiscsSim =
        new DCMotorSim(LinearSystemId.createDCMotorSystem(GEARBOX, 0.005, REDUCTION), GEARBOX);

    // Configure spinner Spark MAX
    var config = createBaseConfig(MOTOR_CURRENT_LIMIT, MOTOR_INVERTED);
    config
        .encoder
        .positionConversionFactor(INTERNAL_ENCODER_POSITION_FACTOR)
        .velocityConversionFactor(INTERNAL_ENCODER_VELOCITY_FACTOR);
    spark.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  /**
   * Updates inputs by simulating physics models and updating the SparkMaxSim state.
   *
   * @param inputs The inputs object to update.
   */
  @Override
  public void updateInputs(SpinnerIOInputs inputs) {
    // Advance physics
    physiscsSim.update(0.02);

    // Update SparkMaxSim with physics results
    // iterate() expects velocity in units AFTER the encoder conversion factor (rad/s mechanism)
    sparkSim.iterate(physiscsSim.getAngularVelocityRadPerSec(), RoboRioSim.getVInVoltage(), 0.02);

    // Populate logged inputs from Spark encoders
    inputs.connected = true;
    inputs.appliedVoltage = Volts.of(spark.getAppliedOutput() * spark.getBusVoltage());
    inputs.appliedCurrent = Amps.of(spark.getOutputCurrent());
  }

  @Override
  public void setVoltage(Voltage voltage) {
    spark.setVoltage(voltage.in(Volts));
  }
}
