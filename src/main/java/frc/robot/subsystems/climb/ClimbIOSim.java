// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.*;
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
 * ClimbIOSpark}.
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
    spark = new SparkMax(CAN_ID, MotorType.kBrushed);
    sparkSim = new SparkMaxSim(spark, GEARBOX);
    forwardLimitSim = sparkSim.getForwardLimitSwitchSim();
    reverseLimitSim = sparkSim.getReverseLimitSwitchSim();
    forwardLimit = spark.getForwardLimitSwitch();
    reverseLimit = spark.getReverseLimitSwitch();

    physicsSim =
        new ClimbSim(
            GEARBOX,
            REDUCTION,
            CLIMBER_MASS.in(Kilograms),
            DriveConstants.ROBOT_MASS.in(Kilograms),
            DRUM_RADIUS.in(Meter),
            LOWER_HEIGHT.in(Meter),
            UPPER_HEIGHT.in(Meter),
            SPRING_CONSTANT.in(Newtons));

    // Disable hardware limit switch behavior so the Spark's internal firmware does not block output
    var liftCfg = new SparkMaxConfig();
    liftCfg
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit((int) MOTOR_CURRENT_LIMIT.in(Amps))
        .voltageCompensation(12.0);
    liftCfg.limitSwitch.forwardLimitSwitchType(LimitSwitchConfig.Type.kNormallyOpen);
    liftCfg.limitSwitch.reverseLimitSwitchType(LimitSwitchConfig.Type.kNormallyOpen);
    spark.configure(liftCfg, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
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
    double velocityRadPerSec = physicsSim.getVelocityMetersPerSecond() / DRUM_RADIUS.in(Meter);
    double motorRadPerSec = velocityRadPerSec * REDUCTION;
    double velocityRPM = motorRadPerSec * 60.0 / (2.0 * Math.PI);

    // Update SparkMaxSim with physics results
    sparkSim.iterate(velocityRPM, RoboRioSim.getVInVoltage(), 0.02);

    // Emulate limit switches based on simulated position
    double positionMeters = physicsSim.getPositionMeters();
    boolean atUpper = positionMeters >= UPPER_HEIGHT.in(Meter);
    boolean atLower = positionMeters <= LOWER_HEIGHT.in(Meter);
    forwardLimitSim.setPressed(atUpper);
    reverseLimitSim.setPressed(atLower);

    // If the mechanism has overshot a limit, reset the physics sim to the boundary
    // so the limit switch clears on the very next cycle once we reverse direction.
    if (atUpper) {
      physicsSim.setState(UPPER_HEIGHT.in(Meter), 0.0);
    } else if (atLower) {
      physicsSim.setState(LOWER_HEIGHT.in(Meter), 0.0);
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
    boolean atUpper = forwardLimit.isPressed();
    boolean atLower = reverseLimit.isPressed();

    if ((voltage.gt(Volts.zero()) && atUpper) || (voltage.lt(Volts.zero()) && atLower)) {
      spark.setVoltage(0.0);
    } else {
      spark.setVoltage(voltage);
    }
  }
}
