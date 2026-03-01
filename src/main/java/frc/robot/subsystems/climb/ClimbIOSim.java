// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.climb.ClimbConstants.*;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkLimitSwitchSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;

/**
 * Physics simulation implementation of {@link ClimbIO}.
 *
 * <p>This class uses {@link SparkMaxSim} to simulate the Spark MAX motor controller for the climb
 * mechanism, backed by a {@link DCMotorSim} physics model. Limit switch behavior is emulated via
 * {@link SparkLimitSwitchSim} based on the simulated position. Configuration mirrors {@link
 * ClimbIOReal}.
 */
public class ClimbIOSim implements ClimbIO {
  /** Simulated upper travel limit in radians (mechanism-side). */
  private static final double upperLimitRad = 50.0;
  /** Simulated lower travel limit in radians (mechanism-side). */
  private static final double lowerLimitRad = 0.0;

  private final SparkMax spark;
  private final SparkMaxSim sparkSim;
  private final SparkLimitSwitchSim forwardLimitSim;
  private final SparkLimitSwitchSim reverseLimitSim;
  private final SparkLimitSwitch forwardLimit;
  private final SparkLimitSwitch reverseLimit;
  private final DCMotorSim physicsSim;

  /** Creates a new ClimbIOSim and initializes the simulated Spark MAX. */
  public ClimbIOSim() {
    spark = new SparkMax(liftMotorCanId, MotorType.kBrushed);
    sparkSim = new SparkMaxSim(spark, liftGearbox);
    forwardLimitSim = sparkSim.getForwardLimitSwitchSim();
    reverseLimitSim = sparkSim.getReverseLimitSwitchSim();
    forwardLimit = spark.getForwardLimitSwitch();
    reverseLimit = spark.getReverseLimitSwitch();

    physicsSim =
        new DCMotorSim(LinearSystemId.createDCMotorSystem(liftGearbox, 0.02, 1.0), liftGearbox);

    // Configure Spark MAX (mirrors ClimbIOReal)
    // Disable hardware limit switch behavior so the Spark's internal firmware does not block output
    var liftCfg = new SparkMaxConfig();
    liftCfg
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit((int) liftMotorCurrentLimit.in(Amps))
        .voltageCompensation(12.0);
    liftCfg.limitSwitch.forwardLimitSwitchType(LimitSwitchConfig.Type.kNormallyOpen);
    liftCfg.limitSwitch.reverseLimitSwitchType(LimitSwitchConfig.Type.kNormallyOpen);
    spark.configure(liftCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  /**
   * Updates the simulation state and updates loggable inputs.
   *
   * @param inputs The inputs object to update with simulated data.
   */
  @Override
  public void updateInputs(ClimbIOInputs inputs) {
    // Update physics model with voltage from Spark sim
    physicsSim.setInputVoltage(sparkSim.getAppliedOutput() * RoboRioSim.getVInVoltage());

    // Advance simulation by 20ms
    physicsSim.update(0.02);

    // Update SparkMaxSim with physics results (no conversion factor, so RPM)
    sparkSim.iterate(physicsSim.getAngularVelocityRPM(), RoboRioSim.getVInVoltage(), 0.02);

    // Emulate limit switches based on simulated position
    double positionRad = physicsSim.getAngularPositionRad();
    boolean atUpper = positionRad >= upperLimitRad;
    boolean atLower = positionRad <= lowerLimitRad;
    forwardLimitSim.setPressed(atUpper);
    reverseLimitSim.setPressed(atLower);

    // If the mechanism has overshot a limit, reset the physics sim to the boundary
    // so the limit switch clears on the very next cycle once we reverse direction.
    if (atUpper) {
      physicsSim.setState(upperLimitRad, 0.0);
    } else if (atLower) {
      physicsSim.setState(lowerLimitRad, 0.0);
    }

    // Update loggable inputs
    inputs.connected = true;
    inputs.appliedVoltage = Volts.of(spark.getAppliedOutput() * spark.getBusVoltage());
    inputs.appliedCurrent = Amps.of(spark.getOutputCurrent());
    inputs.upperLimit = atUpper;
    inputs.lowerLimit = atLower;
  }

  @Override
  public void setLiftVoltage(Voltage voltage) {
    // Safety checks matching ClimbIOReal: stop motor if driving into a pressed limit switch
    boolean atUpper = forwardLimit.isPressed();
    boolean atLower = reverseLimit.isPressed();

    if ((voltage.gt(Volts.of(0.0)) && atUpper) || (voltage.lt(Volts.of(0.0)) && atLower)) {
      spark.setVoltage(0.0);
    } else {
      spark.setVoltage(voltage);
    }
  }
}
