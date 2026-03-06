// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meter;
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
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.simulation.ClimbSim;
import frc.robot.subsystems.drive.DriveConstants;

/**
 * Physics simulation implementation of {@link ClimbIO}.
 *
 * <p>This class uses {@link SparkMaxSim} to simulate the Spark MAX motor controller for the climb
 * mechanism, backed by a {@link DCMotorSim} physics model. Limit switch behavior is emulated via
 * {@link SparkLimitSwitchSim} based on the simulated position. Configuration mirrors {@link
 * ClimbIOReal}.
 */
public class ClimbIOSim implements ClimbIO {
  private final SparkMax spark;
  private final SparkMaxSim sparkSim;
  private final SparkLimitSwitchSim forwardLimitSim;
  private final SparkLimitSwitchSim reverseLimitSim;
  private final SparkLimitSwitch forwardLimit;
  private final SparkLimitSwitch reverseLimit;
  private final ClimbSim physicsSim;

  /** Creates a new ClimbIOSim and initializes the simulated Spark MAX. */
  public ClimbIOSim() {
    spark = new SparkMax(canId, MotorType.kBrushed);
    sparkSim = new SparkMaxSim(spark, gearbox);
    forwardLimitSim = sparkSim.getForwardLimitSwitchSim();
    reverseLimitSim = sparkSim.getReverseLimitSwitchSim();
    forwardLimit = spark.getForwardLimitSwitch();
    reverseLimit = spark.getReverseLimitSwitch();

    physicsSim =
        new ClimbSim(
            gearbox,
            gearReduction,
            1.5,
            DriveConstants.robotMass.in(Kilograms),
            drumRadius.in(Meter),
            lowerHeight.in(Meter),
            upperHeight.in(Meter),
            40.0 // TODO: Get actual arm mass and spring force values
            );

    // Disable hardware limit switch behavior so the Spark's internal firmware does not block output
    var liftCfg = new SparkMaxConfig();
    liftCfg
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit((int) motorCurrentLimit.in(Amps))
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

    // Calculate RPM of the motor (v_linear / r = omega_shaft -> omega_shaft * gear_ratio =
    // omega_motor)
    double velocityRadPerSec = physicsSim.getVelocityMetersPerSecond() / drumRadius.in(Meter);
    double motorRadPerSec = velocityRadPerSec * gearReduction;
    double velocityRPM = motorRadPerSec * 60.0 / (2.0 * Math.PI);

    // Update SparkMaxSim with physics results
    sparkSim.iterate(velocityRPM, RoboRioSim.getVInVoltage(), 0.02);

    // Emulate limit switches based on simulated position
    double positionMeters = physicsSim.getPositionMeters();
    boolean atUpper = positionMeters >= upperHeight.in(Meter);
    boolean atLower = positionMeters <= lowerHeight.in(Meter);
    forwardLimitSim.setPressed(atUpper);
    reverseLimitSim.setPressed(atLower);

    // If the mechanism has overshot a limit, reset the physics sim to the boundary
    // so the limit switch clears on the very next cycle once we reverse direction.
    if (atUpper) {
      physicsSim.setState(upperHeight.in(Meter), 0.0);
    } else if (atLower) {
      physicsSim.setState(lowerHeight.in(Meter), 0.0);
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
